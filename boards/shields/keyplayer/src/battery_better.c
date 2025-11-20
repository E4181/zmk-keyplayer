/*
 * Optimized Battery Monitoring for ZMK
 * 完全独立的电池驱动实现
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// 配置参数
#define MOVING_AVERAGE_SAMPLES   8
#define MEDIAN_FILTER_SAMPLES    5
#define FAST_SAMPLING_INTERVAL   5      // 秒
#define SLOW_SAMPLING_INTERVAL   30     // 秒
#define ACTIVITY_TIMEOUT         30000  // 毫秒

// ADC配置
#define ADC_RESOLUTION           12
#define ADC_GAIN                 ADC_GAIN_1_4
#define ADC_REFERENCE            ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME     ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)

// 电池电压到电量的映射表（针对典型锂电池）
static const struct battery_voltage_map {
    uint32_t voltage;
    uint8_t percentage;
} voltage_map[] = {
    {4200, 100}, {4150, 95}, {4100, 90}, {4050, 85},
    {4000, 80},  {3950, 75}, {3900, 70}, {3850, 65},
    {3800, 60},  {3750, 55}, {3700, 50}, {3650, 45},
    {3600, 40},  {3550, 35}, {3500, 30}, {3450, 25},
    {3400, 20},  {3350, 15}, {3300, 10}, {3250, 5},
    {3200, 0}
};
#define VOLTAGE_MAP_SIZE (sizeof(voltage_map) / sizeof(voltage_map[0]))

// 电池数据结构
struct battery_data {
    uint32_t voltage_history[MOVING_AVERAGE_SAMPLES];
    uint32_t median_buffer[MEDIAN_FILTER_SAMPLES];
    uint8_t mov_avg_index;
    uint8_t median_index;
    uint32_t last_activity_time;
    uint32_t current_voltage;
    uint8_t current_percentage;
    bool history_initialized;
    bool keyboard_active;
    
    // ADC相关
    const struct device *adc_device;
    uint16_t adc_buffer;
    struct adc_sequence adc_sequence;
    struct adc_channel_cfg adc_channel_cfg;
    int8_t adc_channel_id;
};

static struct battery_data batt_data = {
    .mov_avg_index = 0,
    .median_index = 0,
    .last_activity_time = 0,
    .current_voltage = 0,
    .current_percentage = 0,
    .history_initialized = false,
    .keyboard_active = false,
    .adc_device = NULL,
    .adc_channel_id = -1
};

// 函数声明
static int adc_init(void);
static uint32_t read_battery_voltage(void);
static uint32_t moving_average_filter(uint32_t new_voltage);
static uint32_t median_filter(uint32_t new_voltage);
static uint32_t load_compensation(uint32_t voltage);
static uint8_t calculate_battery_percentage(uint32_t voltage);
static void update_sampling_strategy(void);
static void battery_work_handler(struct k_work *work);
static int keyboard_activity_handler(const zmk_event_t *eh);
static void publish_battery_event(uint32_t voltage, uint8_t percentage);

// 工作队列
static K_WORK_DELAYABLE_DEFINE(battery_work, battery_work_handler);

// 事件监听器
ZMK_LISTENER(battery_optimized, keyboard_activity_handler);
ZMK_SUBSCRIPTION(battery_optimized, zmk_keycode_state_changed);

/**
 * ADC初始化
 */
static int adc_init(void)
{
    // 获取ADC设备 - 根据你的硬件配置调整
    batt_data.adc_device = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!device_is_ready(batt_data.adc_device)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }
    
    // 配置ADC通道 - 使用SAADC通道0，根据你的硬件调整
    batt_data.adc_channel_cfg = (struct adc_channel_cfg){
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 0,  // SAADC通道0
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + 2  // AIN2，根据硬件调整
    };
    
    int err = adc_channel_setup(batt_data.adc_device, &batt_data.adc_channel_cfg);
    if (err) {
        LOG_ERR("Failed to setup ADC channel: %d", err);
        return err;
    }
    
    // 配置ADC序列
    batt_data.adc_sequence = (struct adc_sequence){
        .channels = BIT(0),
        .buffer = &batt_data.adc_buffer,
        .buffer_size = sizeof(batt_data.adc_buffer),
        .resolution = ADC_RESOLUTION,
        .oversampling = 4,  // 4倍过采样提高精度
        .calibrate = true
    };
    
    LOG_INF("ADC initialized successfully");
    return 0;
}

/**
 * 读取电池电压
 */
static uint32_t read_battery_voltage(void)
{
    if (batt_data.adc_device == NULL) {
        LOG_ERR("ADC device not initialized");
        return 0;
    }
    
    int ret = adc_read(batt_data.adc_device, &batt_data.adc_sequence);
    if (ret != 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return 0;
    }
    
    // 将ADC值转换为电压（毫伏）
    // 假设分压电阻配置：2M + 806K，参考电压0.6V，增益1/4
    // 实际计算需要根据你的硬件分压电路调整
    
    // nRF52 SAADC参考电压为0.6V，增益1/4时量程为2.4V
    int32_t adc_value = batt_data.adc_buffer;
    int32_t adc_max = (1 << ADC_RESOLUTION) - 1;
    
    // 计算ADC输入电压（毫伏）
    // 参考电压600mV * 增益4 = 2400mV量程
    int32_t adc_voltage_mv = (adc_value * 2400) / adc_max;
    
    // 计算实际电池电压（考虑分压电阻）
    // 分压比例 = (R1 + R2) / R2 = (2000000 + 806000) / 806000 ≈ 3.48
    int32_t battery_voltage_mv = (adc_voltage_mv * 2806) / 806;
    
    LOG_DBG("ADC value: %d, ADC voltage: %dmV, Battery voltage: %dmV", 
            adc_value, adc_voltage_mv, battery_voltage_mv);
    
    return (uint32_t)battery_voltage_mv;
}

/**
 * 移动平均滤波器
 */
static uint32_t moving_average_filter(uint32_t new_voltage)
{
    if (!batt_data.history_initialized) {
        for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
            batt_data.voltage_history[i] = new_voltage;
        }
        batt_data.history_initialized = true;
        return new_voltage;
    }
    
    batt_data.voltage_history[batt_data.mov_avg_index] = new_voltage;
    batt_data.mov_avg_index = (batt_data.mov_avg_index + 1) % MOVING_AVERAGE_SAMPLES;
    
    uint32_t sum = 0;
    for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
        sum += batt_data.voltage_history[i];
    }
    
    return sum / MOVING_AVERAGE_SAMPLES;
}

/**
 * 中值滤波器
 */
static uint32_t median_filter(uint32_t new_voltage)
{
    batt_data.median_buffer[batt_data.median_index] = new_voltage;
    batt_data.median_index = (batt_data.median_index + 1) % MEDIAN_FILTER_SAMPLES;
    
    // 创建临时数组进行排序
    uint32_t temp[MEDIAN_FILTER_SAMPLES];
    memcpy(temp, batt_data.median_buffer, sizeof(temp));
    
    // 简单排序取中值
    for (int i = 0; i < MEDIAN_FILTER_SAMPLES - 1; i++) {
        for (int j = i + 1; j < MEDIAN_FILTER_SAMPLES; j++) {
            if (temp[i] > temp[j]) {
                uint32_t swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    
    return temp[MEDIAN_FILTER_SAMPLES / 2];
}

/**
 * 负载补偿
 */
static uint32_t load_compensation(uint32_t voltage)
{
    int32_t compensation = 0;
    uint32_t current_time = k_uptime_get();
    
    // 检查键盘是否处于活跃状态
    if (current_time - batt_data.last_activity_time < 5000) {
        // 5秒内有活动，认为是活跃状态
        compensation = -20; // -20mV补偿负载压降
        batt_data.keyboard_active = true;
    } else if (current_time - batt_data.last_activity_time < ACTIVITY_TIMEOUT) {
        // 30秒内有活动，中等补偿
        compensation = -10;
        batt_data.keyboard_active = true;
    } else {
        // 空闲状态
        compensation = 5;
        batt_data.keyboard_active = false;
    }
    
    int32_t result = (int32_t)voltage + compensation;
    return (result < 0) ? 0 : (uint32_t)result;
}

/**
 * 计算电池电量百分比
 */
static uint8_t calculate_battery_percentage(uint32_t voltage)
{
    // 边界检查
    if (voltage >= voltage_map[0].voltage) {
        return 100;
    }
    if (voltage <= voltage_map[VOLTAGE_MAP_SIZE-1].voltage) {
        return 0;
    }
    
    // 线性插值计算电量百分比
    for (int i = 0; i < VOLTAGE_MAP_SIZE - 1; i++) {
        if (voltage >= voltage_map[i+1].voltage && voltage <= voltage_map[i].voltage) {
            uint32_t voltage_range = voltage_map[i].voltage - voltage_map[i+1].voltage;
            uint32_t voltage_offset = voltage - voltage_map[i+1].voltage;
            uint8_t percentage_range = voltage_map[i].percentage - voltage_map[i+1].percentage;
            
            return voltage_map[i+1].percentage + 
                   (percentage_range * voltage_offset) / voltage_range;
        }
    }
    
    return 50; // 默认值
}

/**
 * 更新采样策略
 */
static void update_sampling_strategy(void)
{
    uint32_t current_time = k_uptime_get();
    k_timeout_t sampling_interval;
    
    if (current_time - batt_data.last_activity_time < ACTIVITY_TIMEOUT) {
        // 活跃状态，快速采样
        sampling_interval = K_SECONDS(FAST_SAMPLING_INTERVAL);
    } else {
        // 空闲状态，慢速采样以节省电量
        sampling_interval = K_SECONDS(SLOW_SAMPLING_INTERVAL);
    }
    
    k_work_reschedule(&battery_work, sampling_interval);
}

/**
 * 发布电池事件
 */
static void publish_battery_event(uint32_t voltage, uint8_t percentage)
{
    // 创建电池状态改变事件
    struct zmk_battery_state_changed *ev = new_zmk_battery_state_changed();
    if (ev) {
        // 设置电量百分比
        ev->level_of_charge = percentage;
        ZMK_EVENT_RAISE(ev);
        LOG_DBG("Published battery event: %d%% (%dmV)", percentage, voltage);
    } else {
        LOG_ERR("Failed to create battery event");
    }
}

/**
 * 电池工作处理函数
 */
static void battery_work_handler(struct k_work *work)
{
    // 读取原始电池电压
    uint32_t raw_voltage = read_battery_voltage();
    
    if (raw_voltage == 0) {
        LOG_WRN("Failed to read battery voltage");
        goto reschedule;
    }
    
    // 应用滤波算法
    uint32_t filtered_voltage = moving_average_filter(raw_voltage);
    filtered_voltage = median_filter(filtered_voltage);
    filtered_voltage = load_compensation(filtered_voltage);
    
    // 计算电量百分比
    uint8_t percentage = calculate_battery_percentage(filtered_voltage);
    
    // 更新电池数据
    batt_data.current_voltage = filtered_voltage;
    batt_data.current_percentage = percentage;
    
    LOG_DBG("Battery: %dmV -> %d%% (raw: %dmV)", 
            filtered_voltage, percentage, raw_voltage);
    
    // 发布电池更新事件
    publish_battery_event(filtered_voltage, percentage);

reschedule:
    update_sampling_strategy();
}

/**
 * 键盘活动事件处理
 */
static int keyboard_activity_handler(const zmk_event_t *eh)
{
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL) {
        return -ENOTSUP;
    }
    
    batt_data.last_activity_time = k_uptime_get();
    
    // 如果从空闲变为活跃，立即采样一次
    if (!batt_data.keyboard_active) {
        k_work_reschedule(&battery_work, K_MSEC(100));
    }
    
    return 0;
}

/**
 * 获取优化后的电池电压
 */
uint32_t battery_better_get_voltage(void)
{
    return batt_data.current_voltage;
}

/**
 * 获取优化后的电池电量百分比
 */
uint8_t battery_better_get_percentage(void)
{
    return batt_data.current_percentage;
}

/**
 * 获取电池状态字符串
 */
const char* battery_better_get_status_string(void)
{
    uint8_t percentage = batt_data.current_percentage;
    
    if (percentage > 90) return "Full";
    if (percentage > 70) return "High";
    if (percentage > 40) return "Medium";
    if (percentage > 20) return "Low";
    if (percentage > 5) return "Very Low";
    return "Critical";
}

/**
 * 初始化优化电池监测
 */
int battery_better_init(void)
{
    LOG_INF("Initializing optimized battery monitoring");
    
    // 初始化ADC
    int ret = adc_init();
    if (ret != 0) {
        LOG_ERR("ADC initialization failed: %d", ret);
        return ret;
    }
    
    // 初始化历史数据
    uint32_t initial_voltage = read_battery_voltage();
    if (initial_voltage > 0) {
        for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
            batt_data.voltage_history[i] = initial_voltage;
        }
        for (int i = 0; i < MEDIAN_FILTER_SAMPLES; i++) {
            batt_data.median_buffer[i] = initial_voltage;
        }
        batt_data.history_initialized = true;
        batt_data.current_voltage = initial_voltage;
        batt_data.current_percentage = calculate_battery_percentage(initial_voltage);
        
        // 发布初始电池状态
        publish_battery_event(initial_voltage, batt_data.current_percentage);
    }
    
    // 设置初始活动时间
    batt_data.last_activity_time = k_uptime_get();
    
    // 启动第一次电池采样
    k_work_reschedule(&battery_work, K_SECONDS(2));
    
    LOG_INF("Optimized battery monitoring initialized");
    return 0;
}

/**
 * 电池校准函数（用于手动校准）
 */
int battery_better_calibrate(uint32_t measured_voltage)
{
    if (measured_voltage < 3000 || measured_voltage > 4500) {
        LOG_ERR("Invalid calibration voltage: %dmV", measured_voltage);
        return -EINVAL;
    }
    
    LOG_INF("Battery calibration: measured %dmV, current reading %dmV", 
            measured_voltage, batt_data.current_voltage);
    
    return 0;
}

// 初始化电池系统
SYS_INIT(battery_better_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);