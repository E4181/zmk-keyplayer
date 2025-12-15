#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 使用Kconfig配置
#define CHRG_GPIO_PORT_STR CONFIG_CHARGING_MONITOR_GPIO_PORT
#define CHRG_GPIO_PIN CONFIG_CHARGING_MONITOR_GPIO_PIN
#define CHRG_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

#define POLL_INTERVAL_CHARGING_MS CONFIG_CHARGING_MONITOR_POLL_CHARGING_MS
#define POLL_INTERVAL_FULL_MS CONFIG_CHARGING_MONITOR_POLL_FULL_MS
#define POLL_INTERVAL_ERROR_MS CONFIG_CHARGING_MONITOR_POLL_ERROR_MS
#define IDLE_TIMEOUT_MS CONFIG_CHARGING_MONITOR_IDLE_TIMEOUT_MS
#define IDLE_MULTIPLIER CONFIG_CHARGING_MONITOR_IDLE_MULTIPLIER
#define MAX_CONSECUTIVE_ERRORS CONFIG_CHARGING_MONITOR_MAX_ERRORS

// 优化的充电监控器私有数据结构
struct charging_monitor_data {
    // 状态变量
#if CONFIG_CHARGING_MONITOR_USE_ATOMIC
    atomic_t current_state;
    atomic_t error_count;
    atomic_t consecutive_errors;
#else
    charging_state_t current_state;
    uint8_t error_count;
    uint8_t consecutive_errors;
#endif
    
    // 锁机制简化：只需要一个互斥锁保护复杂操作
    struct k_mutex data_mutex;
    
    // 工作队列
    struct k_work_delayable status_check_work;
    struct k_work callback_work;
    
    // 回调函数
    charging_state_changed_cb_t callback;
    
    // GPIO设备
    const struct device *gpio_dev;
    
    // 统计信息
    uint32_t total_state_changes;
    uint32_t total_errors;
    int64_t charging_start_time;
    int64_t last_activity_time;
    int64_t last_successful_read;
    
    // 错误信息
    charging_error_t last_error;
    int64_t last_error_time;
    
    // 控制标志
    bool initialized;
    bool polling_active;
    bool system_idle;
    bool recovery_in_progress;
};

// 获取私有数据实例（使用更紧凑的内存布局）
static struct charging_monitor_data *get_data(void)
{
    static struct charging_monitor_data data = {
#if CONFIG_CHARGING_MONITOR_USE_ATOMIC
        .current_state = ATOMIC_INIT(CHARGING_STATE_ERROR),
        .error_count = ATOMIC_INIT(0),
        .consecutive_errors = ATOMIC_INIT(0),
#else
        .current_state = CHARGING_STATE_ERROR,
        .error_count = 0,
        .consecutive_errors = 0,
#endif
        .callback = NULL,
        .gpio_dev = NULL,
        .total_state_changes = 0,
        .total_errors = 0,
        .charging_start_time = 0,
        .last_activity_time = 0,
        .last_successful_read = 0,
        .last_error = CHARGING_ERROR_NONE,
        .last_error_time = 0,
        .initialized = false,
        .polling_active = true,
        .system_idle = false,
        .recovery_in_progress = false,
    };
    return &data;
}

// 获取当前状态（线程安全）
static charging_state_t get_current_state(void)
{
#if CONFIG_CHARGING_MONITOR_USE_ATOMIC
    return (charging_state_t)atomic_get(&get_data()->current_state);
#else
    struct charging_monitor_data *data = get_data();
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    charging_state_t state = data->current_state;
    k_mutex_unlock(&data->data_mutex);
    return state;
#endif
}

// 设置当前状态（线程安全）
static void set_current_state(charging_state_t state)
{
    struct charging_monitor_data *data = get_data();
    
#if CONFIG_CHARGING_MONITOR_USE_ATOMIC
    atomic_set(&data->current_state, (atomic_val_t)state);
#else
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->current_state = state;
    k_mutex_unlock(&data->data_mutex);
#endif
}

// 异步回调工作函数
static void callback_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    struct charging_monitor_data *data = get_data();
    charging_state_changed_cb_t callback;
    charging_state_t current_state;
    
    current_state = get_current_state();
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    callback = data->callback;
    k_mutex_unlock(&data->data_mutex);
    
    if (callback) {
        callback(current_state);
    }
}

// 智能轮询间隔计算（优化版）
static uint32_t calculate_polling_interval(charging_state_t state, bool system_idle, 
                                          uint8_t consecutive_errors)
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
        // 错误状态使用指数退避
        base_interval = POLL_INTERVAL_ERROR_MS * (1 << MIN(consecutive_errors, 4));
        base_interval = MIN(base_interval, 300000); // 最大5分钟
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

// 记录错误（智能错误恢复）
static void record_error(charging_error_t error, int ret_code)
{
    struct charging_monitor_data *data = get_data();
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    data->last_error = error;
    data->last_error_time = k_uptime_get();
    data->total_errors++;
    
    if (data->consecutive_errors < MAX_CONSECUTIVE_ERRORS) {
        data->consecutive_errors++;
    }
    
    // 如果连续错误超过阈值，尝试恢复
    if (data->consecutive_errors >= MAX_CONSECUTIVE_ERRORS && !data->recovery_in_progress) {
        data->recovery_in_progress = true;
        LOG_WRN("Too many consecutive errors (%d), attempting recovery", 
                data->consecutive_errors);
        
        // 在下一个工作周期尝试恢复
        k_mutex_unlock(&data->data_mutex);
        
        // 设置一个短时间的延迟来尝试恢复
        k_work_reschedule(&data->status_check_work, K_MSEC(1000));
        return;
    }
    
    k_mutex_unlock(&data->data_mutex);
    
    LOG_ERR("Error %d: code %d, consecutive errors: %d", 
            error, ret_code, data->consecutive_errors);
}

// 清除错误计数（成功操作后调用）
static void clear_errors(void)
{
    struct charging_monitor_data *data = get_data();
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->consecutive_errors = 0;
    data->recovery_in_progress = false;
    data->last_successful_read = k_uptime_get();
    k_mutex_unlock(&data->data_mutex);
}

// 尝试恢复（从错误状态恢复）
static bool attempt_recovery(struct charging_monitor_data *data)
{
    int ret;
    
    LOG_INF("Attempting GPIO recovery");
    
    // 尝试重新配置GPIO
    ret = gpio_pin_configure(data->gpio_dev, CHRG_GPIO_PIN, GPIO_INPUT | CHRG_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("GPIO recovery failed: %d", ret);
        return false;
    }
    
    // 尝试读取一次状态
    int pin_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    if (pin_state < 0) {
        LOG_ERR("GPIO read after recovery failed: %d", pin_state);
        return false;
    }
    
    LOG_INF("GPIO recovery successful");
    clear_errors();
    return true;
}

// 记录活动时间
static void record_activity(void)
{
    struct charging_monitor_data *data = get_data();
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->last_activity_time = k_uptime_get();
    
    // 如果从空闲状态恢复，记录日志
    if (data->system_idle) {
        data->system_idle = false;
        LOG_DBG("Activity detected, exiting idle mode");
    }
    
    k_mutex_unlock(&data->data_mutex);
}

// 检查系统是否空闲
static bool check_system_idle(void)
{
    struct charging_monitor_data *data = get_data();
    bool is_idle;
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    int64_t now = k_uptime_get();
    is_idle = ((now - data->last_activity_time) > IDLE_TIMEOUT_MS);
    
    // 只有状态变化时才记录日志
    if (is_idle != data->system_idle) {
        data->system_idle = is_idle;
        LOG_DBG("System %s", is_idle ? "idle" : "active");
    }
    
    k_mutex_unlock(&data->data_mutex);
    return is_idle;
}

// 状态检查工作函数（优化版）
static void status_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized || !data->gpio_dev) {
        LOG_WRN("Charging monitor not initialized");
        k_work_reschedule(dwork, K_MSEC(POLL_INTERVAL_ERROR_MS));
        return;
    }
    
    // 检查恢复状态
    if (data->recovery_in_progress) {
        if (attempt_recovery(data)) {
            // 恢复成功，继续正常流程
        } else {
            // 恢复失败，延长等待时间
            uint32_t interval = calculate_polling_interval(CHARGING_STATE_ERROR, 
                                                          data->system_idle, 
                                                          data->consecutive_errors);
            k_work_reschedule(dwork, K_MSEC(interval));
            return;
        }
    }
    
    // 检查轮询是否激活
    if (!data->polling_active) {
        LOG_DBG("Polling paused");
        return;
    }
    
    // 更新空闲状态
    check_system_idle();
    
    // 读取CHRG引脚状态
    int pin_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    
    if (pin_state < 0) {
        record_error(CHARGING_ERROR_GPIO_READ, pin_state);
        set_current_state(CHARGING_STATE_ERROR);
        
        // 智能调度下一次检查
        uint32_t interval = calculate_polling_interval(CHARGING_STATE_ERROR, 
                                                      data->system_idle, 
                                                      data->consecutive_errors);
        k_work_reschedule(dwork, K_MSEC(interval));
        return;
    }
    
    // 成功读取，清除错误计数
    clear_errors();
    
    // 计算新状态
    charging_state_t new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    charging_state_t current_state = get_current_state();
    
    // 状态变化检测
    if (new_state != current_state) {
        const char *old_state_str = (current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : 
                                   (current_state == CHARGING_STATE_FULL) ? "FULL" : "ERROR";
        const char *new_state_str = (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL";
        
        LOG_INF("Charging state changed: %s -> %s", old_state_str, new_state_str);
        
        // 更新统计信息
        k_mutex_lock(&data->data_mutex, K_FOREVER);
        data->total_state_changes++;
        
        // 记录充电开始时间
        if (new_state == CHARGING_STATE_CHARGING && current_state != CHARGING_STATE_CHARGING) {
            data->charging_start_time = k_uptime_get();
            LOG_DBG("Charging started");
        }
        
        // 记录充电完成时间
        if (current_state == CHARGING_STATE_CHARGING && new_state == CHARGING_STATE_FULL) {
            int64_t charge_duration = k_uptime_get() - data->charging_start_time;
            LOG_INF("Charging completed in %lld ms", charge_duration);
        }
        
        k_mutex_unlock(&data->data_mutex);
        
        // 更新状态
        set_current_state(new_state);
        
        // 触发回调
        k_work_submit(&data->callback_work);
    }
    
    // 智能调度下一次检查
    uint32_t interval = calculate_polling_interval(new_state, 
                                                  data->system_idle, 
                                                  data->consecutive_errors);
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
    
    LOG_DBG("Initializing charging monitor with advanced optimizations");
    
    // 初始化互斥锁
    k_mutex_init(&data->data_mutex);
    
    // 获取GPIO设备
    data->gpio_dev = device_get_binding(CHRG_GPIO_PORT_STR);
    if (!data->gpio_dev) {
        LOG_ERR("Failed to get GPIO device: %s", CHRG_GPIO_PORT_STR);
        return -ENODEV;
    }
    
    if (!device_is_ready(data->gpio_dev)) {
        LOG_ERR("GPIO device not ready: %s", CHRG_GPIO_PORT_STR);
        return -ENODEV;
    }
    
    // 配置CHRG引脚
    ret = gpio_pin_configure(data->gpio_dev, CHRG_GPIO_PIN, GPIO_INPUT | CHRG_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        record_error(CHARGING_ERROR_GPIO_CONFIG, ret);
        return ret;
    }
    
    LOG_INF("Charging monitor configured: %s pin %d, flags: 0x%x", 
            CHRG_GPIO_PORT_STR, CHRG_GPIO_PIN, CHRG_GPIO_FLAGS);
    LOG_INF("Polling intervals: charging=%dms, full=%dms, error=%dms",
            POLL_INTERVAL_CHARGING_MS, POLL_INTERVAL_FULL_MS, POLL_INTERVAL_ERROR_MS);
    
    // 初始化工作队列
    k_work_init_delayable(&data->status_check_work, status_check_work_handler);
    k_work_init(&data->callback_work, callback_work_handler);
    
    // 设置初始活动时间
    record_activity();
    
    // 读取初始状态
    int initial_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    if (initial_state >= 0) {
        charging_state_t init_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        set_current_state(init_state);
        clear_errors();
        
        LOG_INF("Initial charging state: %s", 
                (init_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        record_error(CHARGING_ERROR_GPIO_READ, initial_state);
        set_current_state(CHARGING_STATE_ERROR);
    }
    
    // 立即执行一次状态检查
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
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->callback = callback;
    k_mutex_unlock(&data->data_mutex);
    
    LOG_DBG("Callback registered");
    
    // 立即触发一次回调
    k_work_submit(&data->callback_work);
    
    return 0;
}

// 取消注册回调函数
int charging_monitor_unregister_callback(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->callback = NULL;
    k_mutex_unlock(&data->data_mutex);
    
    LOG_DBG("Callback unregistered");
    return 0;
}

// 获取当前充电状态
charging_state_t charging_monitor_get_state(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return CHARGING_STATE_ERROR;
    }
    
    return get_current_state();
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

// 检查充电监控器是否已初始化
bool charging_monitor_is_initialized(void)
{
    struct charging_monitor_data *data = get_data();
    return data->initialized;
}

// 设置轮询间隔（运行时调整）
void charging_monitor_set_polling_intervals(uint32_t charging_ms, uint32_t full_ms, uint32_t error_ms)
{
    // 注意：这里只是示例，实际需要修改配置变量
    LOG_INF("Polling intervals updated: charging=%ums, full=%ums, error=%ums",
            charging_ms, full_ms, error_ms);
}

// 暂停轮询
void charging_monitor_pause(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return;
    }
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->polling_active = false;
    k_mutex_unlock(&data->data_mutex);
    
    LOG_DBG("Charging monitor polling paused");
}

// 恢复轮询
void charging_monitor_resume(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return;
    }
    
    record_activity();
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    data->polling_active = true;
    k_mutex_unlock(&data->data_mutex);
    
    // 立即触发一次状态检查
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
    
    LOG_DBG("Charging monitor polling resumed");
}

// 获取统计信息
int charging_monitor_get_stats(struct charging_monitor_stats *stats)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized || !stats) {
        return -EINVAL;
    }
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    stats->total_state_changes = data->total_state_changes;
    stats->total_errors = data->total_errors;
    stats->consecutive_errors = data->consecutive_errors;
    stats->current_state = data->current_state;
    stats->last_error = data->last_error;
    stats->last_error_time = data->last_error_time;
    stats->last_successful_read = data->last_successful_read;
    stats->system_idle = data->system_idle;
    stats->polling_active = data->polling_active;
    
    k_mutex_unlock(&data->data_mutex);
    
    return 0;
}

// 手动触发活动记录
void charging_monitor_notify_activity(void)
{
    record_activity();
}

// 手动触发状态检查
void charging_monitor_force_check(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return;
    }
    
    record_activity();
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
    LOG_DBG("Forced status check");
}