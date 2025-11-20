/*
 * Optimized Battery Monitoring for ZMK
 * 简化版本 - 只实现滤波功能
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

#include "battery_common.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// 配置参数
#define MOVING_AVERAGE_SAMPLES   8
#define MEDIAN_FILTER_SAMPLES    5
#define FAST_SAMPLING_INTERVAL   5      // 秒
#define SLOW_SAMPLING_INTERVAL   30     // 秒
#define ACTIVITY_TIMEOUT         30000  // 毫秒

// 优化电池数据结构
struct optimized_battery_data {
    // 滤波数据
    uint16_t voltage_history[MOVING_AVERAGE_SAMPLES];
    uint16_t median_buffer[MEDIAN_FILTER_SAMPLES];
    uint8_t mov_avg_index;
    uint8_t median_index;
    uint32_t last_activity_time;
    uint16_t filtered_millivolts;
    uint8_t filtered_state_of_charge;
    bool history_initialized;
    bool keyboard_active;
    
    // 工作队列
    struct k_work_delayable battery_work;
};

static struct optimized_battery_data opt_batt_data = {
    .mov_avg_index = 0,
    .median_index = 0,
    .last_activity_time = 0,
    .filtered_millivolts = 0,
    .filtered_state_of_charge = 0,
    .history_initialized = false,
    .keyboard_active = false
};

// 函数声明
static uint16_t moving_average_filter(uint16_t new_voltage);
static uint16_t median_filter(uint16_t new_voltage);
static uint16_t load_compensation(uint16_t voltage);
static void update_sampling_strategy(void);
static void battery_work_handler(struct k_work *work);
static int keyboard_activity_handler(const zmk_event_t *eh);

// 事件监听器
ZMK_LISTENER(battery_optimized, keyboard_activity_handler);
ZMK_SUBSCRIPTION(battery_optimized, zmk_keycode_state_changed);

/**
 * 移动平均滤波器
 */
static uint16_t moving_average_filter(uint16_t new_voltage)
{
    if (!opt_batt_data.history_initialized) {
        for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
            opt_batt_data.voltage_history[i] = new_voltage;
        }
        opt_batt_data.history_initialized = true;
        return new_voltage;
    }
    
    opt_batt_data.voltage_history[opt_batt_data.mov_avg_index] = new_voltage;
    opt_batt_data.mov_avg_index = (opt_batt_data.mov_avg_index + 1) % MOVING_AVERAGE_SAMPLES;
    
    uint32_t sum = 0;
    for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
        sum += opt_batt_data.voltage_history[i];
    }
    
    return (uint16_t)(sum / MOVING_AVERAGE_SAMPLES);
}

/**
 * 中值滤波器
 */
static uint16_t median_filter(uint16_t new_voltage)
{
    opt_batt_data.median_buffer[opt_batt_data.median_index] = new_voltage;
    opt_batt_data.median_index = (opt_batt_data.median_index + 1) % MEDIAN_FILTER_SAMPLES;
    
    // 创建临时数组进行排序
    uint16_t temp[MEDIAN_FILTER_SAMPLES];
    memcpy(temp, opt_batt_data.median_buffer, sizeof(temp));
    
    // 简单排序取中值
    for (int i = 0; i < MEDIAN_FILTER_SAMPLES - 1; i++) {
        for (int j = i + 1; j < MEDIAN_FILTER_SAMPLES; j++) {
            if (temp[i] > temp[j]) {
                uint16_t swap = temp[i];
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
static uint16_t load_compensation(uint16_t voltage)
{
    int32_t compensation = 0;
    uint32_t current_time = k_uptime_get();
    
    // 检查键盘是否处于活跃状态
    if (current_time - opt_batt_data.last_activity_time < 5000) {
        // 5秒内有活动，认为是活跃状态
        compensation = -20; // -20mV补偿负载压降
        opt_batt_data.keyboard_active = true;
    } else if (current_time - opt_batt_data.last_activity_time < ACTIVITY_TIMEOUT) {
        // 30秒内有活动，中等补偿
        compensation = -10;
        opt_batt_data.keyboard_active = true;
    } else {
        // 空闲状态
        compensation = 5;
        opt_batt_data.keyboard_active = false;
    }
    
    int32_t result = (int32_t)voltage + compensation;
    return (result < 0) ? 0 : (uint16_t)result;
}

/**
 * 更新采样策略
 */
static void update_sampling_strategy(void)
{
    uint32_t current_time = k_uptime_get();
    k_timeout_t sampling_interval;
    
    if (current_time - opt_batt_data.last_activity_time < ACTIVITY_TIMEOUT) {
        // 活跃状态，快速采样
        sampling_interval = K_SECONDS(FAST_SAMPLING_INTERVAL);
    } else {
        // 空闲状态，慢速采样以节省电量
        sampling_interval = K_SECONDS(SLOW_SAMPLING_INTERVAL);
    }
    
    k_work_reschedule(&opt_batt_data.battery_work, sampling_interval);
}

/**
 * 从原始电池传感器读取数据
 */
static int read_raw_battery_data(uint16_t *millivolts, uint8_t *percentage)
{
    // 获取电池传感器设备
    const struct device *battery_sensor = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    
    if (!device_is_ready(battery_sensor)) {
        LOG_ERR("Battery sensor device is not ready");
        return -ENODEV;
    }
    
    // 获取传感器数据
    int ret = sensor_sample_fetch(battery_sensor);
    if (ret != 0) {
        LOG_ERR("Failed to fetch battery sample: %d", ret);
        return ret;
    }
    
    // 读取电压值
    struct sensor_value voltage_val;
    ret = sensor_channel_get(battery_sensor, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage_val);
    if (ret != 0) {
        LOG_ERR("Failed to get battery voltage: %d", ret);
        return ret;
    }
    
    // 读取电量百分比
    struct sensor_value soc_val;
    ret = sensor_channel_get(battery_sensor, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &soc_val);
    if (ret != 0) {
        LOG_ERR("Failed to get battery state of charge: %d", ret);
        return ret;
    }
    
    // 转换为整数
    *millivolts = voltage_val.val1 * 1000 + voltage_val.val2 / 1000;
    *percentage = soc_val.val1;
    
    LOG_DBG("Raw battery data: %dmV, %d%%", *millivolts, *percentage);
    
    return 0;
}

/**
 * 电池工作处理函数
 */
static void battery_work_handler(struct k_work *work)
{
    uint16_t raw_millivolts;
    uint8_t raw_percentage;
    
    // 读取原始电池数据
    int ret = read_raw_battery_data(&raw_millivolts, &raw_percentage);
    
    if (ret != 0) {
        LOG_WRN("Failed to read raw battery data: %d", ret);
        goto reschedule;
    }
    
    // 应用滤波算法
    uint16_t filtered_voltage = moving_average_filter(raw_millivolts);
    filtered_voltage = median_filter(filtered_voltage);
    filtered_voltage = load_compensation(filtered_voltage);
    
    // 使用ZMK的标准算法计算电量百分比
    uint8_t filtered_percentage = lithium_ion_mv_to_pct((int16_t)filtered_voltage);
    
    // 更新电池数据
    opt_batt_data.filtered_millivolts = filtered_voltage;
    opt_batt_data.filtered_state_of_charge = filtered_percentage;
    
    LOG_DBG("Optimized battery: %dmV -> %d%% (raw: %dmV -> %d%%)", 
            filtered_voltage, filtered_percentage, raw_millivolts, raw_percentage);

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
    
    opt_batt_data.last_activity_time = k_uptime_get();
    
    // 如果从空闲变为活跃，立即采样一次
    if (!opt_batt_data.keyboard_active) {
        k_work_reschedule(&opt_batt_data.battery_work, K_MSEC(100));
    }
    
    return 0;
}

/**
 * 获取优化后的电池电压
 */
uint16_t battery_better_get_voltage(void)
{
    return opt_batt_data.filtered_millivolts;
}

/**
 * 获取优化后的电池电量百分比
 */
uint8_t battery_better_get_percentage(void)
{
    return opt_batt_data.filtered_state_of_charge;
}

/**
 * 获取电池状态字符串
 */
const char* battery_better_get_status_string(void)
{
    uint8_t percentage = opt_batt_data.filtered_state_of_charge;
    
    if (percentage > 90) return "Full";
    if (percentage > 70) return "High";
    if (percentage > 40) return "Medium";
    if (percentage > 20) return "Low";
    if (percentage > 5) return "Very Low";
    return "Critical";
}

/**
 * 处理原始电池读数并返回优化后的值
 * 这个函数可以在现有的电池驱动中调用
 */
uint16_t battery_better_filter_voltage(uint16_t raw_voltage)
{
    // 应用滤波算法
    uint16_t filtered_voltage = moving_average_filter(raw_voltage);
    filtered_voltage = median_filter(filtered_voltage);
    filtered_voltage = load_compensation(filtered_voltage);
    
    // 更新电池数据
    opt_batt_data.filtered_millivolts = filtered_voltage;
    opt_batt_data.filtered_state_of_charge = lithium_ion_mv_to_pct((int16_t)filtered_voltage);
    
    return filtered_voltage;
}

/**
 * 初始化优化电池监测
 */
int battery_better_init(void)
{
    LOG_INF("Initializing optimized battery monitoring");
    
    // 初始化工作队列
    k_work_init_delayable(&opt_batt_data.battery_work, battery_work_handler);
    
    // 读取初始电池数据
    uint16_t initial_voltage;
    uint8_t initial_percentage;
    
    int ret = read_raw_battery_data(&initial_voltage, &initial_percentage);
    if (ret == 0) {
        // 初始化历史数据
        for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
            opt_batt_data.voltage_history[i] = initial_voltage;
        }
        for (int i = 0; i < MEDIAN_FILTER_SAMPLES; i++) {
            opt_batt_data.median_buffer[i] = initial_voltage;
        }
        opt_batt_data.history_initialized = true;
        opt_batt_data.filtered_millivolts = initial_voltage;
        opt_batt_data.filtered_state_of_charge = initial_percentage;
    }
    
    // 设置初始活动时间
    opt_batt_data.last_activity_time = k_uptime_get();
    
    // 启动第一次电池采样
    k_work_reschedule(&opt_batt_data.battery_work, K_SECONDS(2));
    
    LOG_INF("Optimized battery monitoring initialized");
    return 0;
}

// 初始化电池系统
SYS_INIT(battery_better_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);