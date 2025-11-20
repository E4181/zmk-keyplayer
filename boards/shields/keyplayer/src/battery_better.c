/*
 * Optimized Battery Monitoring for ZMK
 * 修正版本 - 使用ZMK实际存在的API
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zmk/battery.h>
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
};

static struct battery_data batt_data = {
    .mov_avg_index = 0,
    .median_index = 0,
    .last_activity_time = 0,
    .current_voltage = 0,
    .current_percentage = 0,
    .history_initialized = false,
    .keyboard_active = false
};

// 函数声明
static uint32_t moving_average_filter(uint32_t new_voltage);
static uint32_t median_filter(uint32_t new_voltage);
static uint32_t load_compensation(uint32_t voltage);
static uint8_t calculate_battery_percentage(uint32_t voltage);
static void update_sampling_strategy(void);
static void battery_work_handler(struct k_work *work);
static void keyboard_activity_handler(const struct zmk_keycode_state_changed *ev);

// 工作队列
static K_WORK_DELAYABLE_DEFINE(battery_work, battery_work_handler);

// 事件监听器
ZMK_LISTENER(battery_optimized, keyboard_activity_handler);
ZMK_SUBSCRIPTION(battery_optimized, zmk_keycode_state_changed);

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
    uint32_t sampling_interval;
    
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
 * 电池工作处理函数
 */
static void battery_work_handler(struct k_work *work)
{
    // 获取原始电池电压读数
    uint32_t raw_voltage = zmk_battery_get_voltage();
    
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
    struct zmk_battery_state_changed *ev = new_zmk_battery_state_changed();
    if (ev) {
        ev->voltage = filtered_voltage;
        ZMK_EVENT_RAISE(ev);
    }

reschedule:
    update_sampling_strategy();
}

/**
 * 键盘活动事件处理
 */
static void keyboard_activity_handler(const struct zmk_keycode_state_changed *ev)
{
    batt_data.last_activity_time = k_uptime_get();
    
    // 如果从空闲变为活跃，立即采样一次
    if (!batt_data.keyboard_active) {
        k_work_reschedule(&battery_work, K_MSEC(100));
    }
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
    
    // 初始化历史数据
    uint32_t initial_voltage = zmk_battery_get_voltage();
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