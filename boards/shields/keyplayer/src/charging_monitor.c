#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 硬编码GPIO配置：使用P1.09 (GPIO1 pin 9)
#define CHARGING_GPIO_PORT      DT_NODELABEL(gpio1)  // GPIO1设备
#define CHARGING_GPIO_PIN       9                    // P1.09
#define CHARGING_GPIO_FLAGS     (GPIO_ACTIVE_LOW | GPIO_PULL_UP)  // 低电平有效，上拉

// 轮询间隔配置（毫秒）- 直接硬编码
#define POLL_INTERVAL_CHARGING_MS   2000   // 充电中：2秒
#define POLL_INTERVAL_FULL_MS      10000   // 充满：10秒
#define POLL_INTERVAL_ERROR_MS     30000   // 错误：30秒

// 空闲检测配置
#define IDLE_TIMEOUT_MS           30000    // 30秒无活动视为空闲
#define IDLE_MULTIPLIER           2        // 空闲时轮询间隔乘数

// 最大连续错误次数
#define MAX_CONSECUTIVE_ERRORS    5

// 防抖时间（毫秒）
#define DEBOUNCE_TIME_MS          1000     // 状态变化防抖时间

// 充电监控器私有数据结构（简化版）
struct charging_monitor_data {
    // 状态变量（移除原子操作，使用普通变量）
    charging_state_t current_state;
    
    // 工作队列
    struct k_work_delayable status_check_work;
    struct k_work callback_work;
    
    // 回调函数
    charging_state_changed_cb_t callback;
    
    // GPIO设备
    const struct device *gpio_dev;
    
    // 统计和控制标志
    uint32_t consecutive_errors;
    int64_t last_activity_time;
    int64_t last_state_change_time;  // 状态变化时间戳，用于防抖
    bool initialized : 1;
    bool polling_active : 1;
    bool system_idle : 1;
};

// 获取私有数据实例
static struct charging_monitor_data *get_data(void)
{
    static struct charging_monitor_data data = {
        .current_state = CHARGING_STATE_ERROR,
        .callback = NULL,
        .gpio_dev = NULL,
        .consecutive_errors = 0,
        .last_activity_time = 0,
        .last_state_change_time = 0,
        .initialized = false,
        .polling_active = true,
        .system_idle = false,
    };
    return &data;
}

// 异步回调工作函数
static void callback_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    struct charging_monitor_data *data = get_data();
    
    // 直接读取当前状态
    charging_state_t current_state = data->current_state;
    
    // 执行回调（不需要锁，回调在初始化后不会改变）
    if (data->callback) {
        data->callback(current_state);
    }
}

// 状态变化防抖检查
static bool should_process_state_change(struct charging_monitor_data *data, 
                                       charging_state_t new_state)
{
    int64_t now = k_uptime_get();
    
    // 如果是错误状态，总是处理（需要尽快恢复）
    if (new_state == CHARGING_STATE_ERROR || 
        data->current_state == CHARGING_STATE_ERROR) {
        return true;
    }
    
    // 防抖：相同状态变化至少间隔DEBOUNCE_TIME_MS
    if (now - data->last_state_change_time < DEBOUNCE_TIME_MS) {
        LOG_DBG("State change debounced: %d -> %d", 
                data->current_state, new_state);
        return false;
    }
    
    return true;
}

// 智能轮询间隔计算（简化版）
static uint32_t calculate_polling_interval(charging_state_t state, bool system_idle, 
                                          uint32_t consecutive_errors)
{
    uint32_t base_interval;
    
    // 根据状态选择基础间隔
    switch (state) {
    case CHARGING_STATE_CHARGING:
        base_interval = POLL_INTERVAL_CHARGING_MS;
        break;
    case CHARGING_STATE_FULL:
        base_interval = POLL_INTERVAL_FULL_MS;
        break;
    case CHARGING_STATE_ERROR:
        // 错误状态使用退避算法
        base_interval = POLL_INTERVAL_ERROR_MS * (1 + (consecutive_errors / 2));
        if (base_interval > 120000) base_interval = 120000; // 最大2分钟
        break;
    default:
        base_interval = POLL_INTERVAL_FULL_MS;
    }
    
    // 应用空闲乘数
    if (system_idle && state != CHARGING_STATE_CHARGING) {
        base_interval *= IDLE_MULTIPLIER;
    }
    
    return base_interval;
}

// 记录活动时间
static void record_activity(void)
{
    struct charging_monitor_data *data = get_data();
    data->last_activity_time = k_uptime_get();
    
    // 如果从空闲状态恢复，记录日志
    if (data->system_idle) {
        data->system_idle = false;
        LOG_DBG("Activity detected, exiting idle mode");
    }
}

// 检查系统是否空闲
static bool check_system_idle(void)
{
    struct charging_monitor_data *data = get_data();
    int64_t now = k_uptime_get();
    bool is_idle = ((now - data->last_activity_time) > IDLE_TIMEOUT_MS);
    
    // 只有状态变化时才记录日志
    if (is_idle != data->system_idle) {
        data->system_idle = is_idle;
        LOG_DBG("System %s", is_idle ? "idle" : "active");
    }
    
    return is_idle;
}

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized || !data->gpio_dev) {
        LOG_WRN("Charging monitor not initialized");
        k_work_reschedule(dwork, K_MSEC(POLL_INTERVAL_ERROR_MS));
        return;
    }
    
    // 检查轮询是否激活
    if (!data->polling_active) {
        LOG_DBG("Polling paused");
        return;
    }
    
    // 更新空闲状态
    bool system_idle = check_system_idle();
    
    // 读取CHRG引脚状态
    int pin_state = gpio_pin_get(data->gpio_dev, CHARGING_GPIO_PIN);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        
        // 增加连续错误计数
        if (data->consecutive_errors < MAX_CONSECUTIVE_ERRORS) {
            data->consecutive_errors++;
        }
        
        // 设置为错误状态
        data->current_state = CHARGING_STATE_ERROR;
        
        // 智能调度下一次检查
        uint32_t interval = calculate_polling_interval(CHARGING_STATE_ERROR, 
                                                      system_idle, 
                                                      data->consecutive_errors);
        k_work_reschedule(dwork, K_MSEC(interval));
        return;
    }
    
    // 成功读取，清除错误计数
    data->consecutive_errors = 0;
    
    // TP4056 CHRG引脚逻辑：
    // - pin_state == 1: 引脚有效（正在充电）
    // - pin_state == 0: 引脚无效（已充满）
    charging_state_t new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    
    // 获取当前状态进行比较
    charging_state_t current_state = data->current_state;
    
    // 状态变化检测（带防抖）
    if (new_state != current_state) {
        // 检查是否需要处理这个状态变化（防抖）
        if (should_process_state_change(data, new_state)) {
            const char *old_state_str = (current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : 
                                       (current_state == CHARGING_STATE_FULL) ? "FULL" : "ERROR";
            const char *new_state_str = (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL";
            
            LOG_INF("Charging state changed: %s -> %s", old_state_str, new_state_str);
            
            // 更新状态和时间戳
            data->current_state = new_state;
            data->last_state_change_time = k_uptime_get();
            
            // 触发异步回调
            k_work_submit(&data->callback_work);
        } else {
            // 防抖过滤掉的状态变化，但仍然记录调试信息
            LOG_DBG("State change filtered by debounce: %d -> %d", 
                    current_state, new_state);
        }
    }
    
    // 智能调度下一次检查
    uint32_t interval = calculate_polling_interval(new_state, system_idle, 0);
    k_work_reschedule(dwork, K_MSEC(interval));
}

// 初始化充电监控器
int charging_monitor_init(void)
{
    struct charging_monitor_data *data = get_data();
    int ret;
    
    if (data->initialized) {
        LOG_WRN("Charging monitor already initialized");
        return 0;
    }
    
    LOG_DBG("Initializing charging monitor with optimized design");
    
    // 获取GPIO设备 - 硬编码使用GPIO1
    data->gpio_dev = DEVICE_DT_GET(CHARGING_GPIO_PORT);
    if (!data->gpio_dev) {
        LOG_ERR("Failed to get GPIO1 device");
        return -ENODEV;
    }
    
    if (!device_is_ready(data->gpio_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return -ENODEV;
    }
    
    // 配置CHRG引脚为输入，上拉电阻
    ret = gpio_pin_configure(data->gpio_dev, CHARGING_GPIO_PIN, GPIO_INPUT | CHARGING_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    LOG_INF("Charging monitor configured: GPIO1 pin %d (P1.09), flags: 0x%x", 
            CHARGING_GPIO_PIN, CHARGING_GPIO_FLAGS);
    
    // 初始化工作队列
    k_work_init_delayable(&data->status_check_work, status_check_work_handler);
    k_work_init(&data->callback_work, callback_work_handler);
    
    // 设置初始活动时间
    record_activity();
    
    // 读取初始状态
    int initial_state = gpio_pin_get(data->gpio_dev, CHARGING_GPIO_PIN);
    if (initial_state >= 0) {
        data->current_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        data->last_state_change_time = k_uptime_get();
        
        LOG_INF("Initial charging state: %s", 
                (data->current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        data->current_state = CHARGING_STATE_ERROR;
    }
    
    // 立即执行一次状态检查（无延迟）
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
    
    data->initialized = true;
    LOG_INF("Charging monitor initialized successfully");
    
    return 0;
}

// 注册回调函数
int charging_monitor_register_callback(charging_state_changed_cb_t callback)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        LOG_ERR("Charging monitor not initialized");
        return -ENODEV;
    }
    
    if (!callback) {
        LOG_ERR("Callback function is NULL");
        return -EINVAL;
    }
    
    // 直接设置回调函数（不需要锁）
    data->callback = callback;
    
    LOG_DBG("Callback registered");
    
    // 立即触发一次回调（通过工作队列）
    k_work_submit(&data->callback_work);
    
    return 0;
}

// 获取当前充电状态
charging_state_t charging_monitor_get_state(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return CHARGING_STATE_ERROR;
    }
    
    return data->current_state;
}

// 获取充电状态字符串
const char* charging_monitor_get_state_str(void)
{
    charging_state_t state = charging_monitor_get_state();
    
    switch (state) {
    case CHARGING_STATE_CHARGING:
        return "CHARGING";
    case CHARGING_STATE_FULL:
        return "FULL";
    case CHARGING_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}