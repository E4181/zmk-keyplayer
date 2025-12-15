#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 硬编码GPIO配置 - ZMK常用方式
#define CHRG_GPIO_DEV DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define CHRG_GPIO_PIN 9  // P1.09
#define CHRG_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

// 智能轮询间隔配置（毫秒）
#define POLL_INTERVAL_CHARGING_MS   2000  // 充电中：2秒
#define POLL_INTERVAL_FULL_MS      10000  // 充满：10秒
#define POLL_INTERVAL_ERROR_MS     30000  // 错误：30秒
#define POLL_INTERVAL_DEFAULT_MS    5000  // 默认：5秒

// 充电监控器私有数据结构
struct charging_monitor_data {
    // 使用原子操作的状态变量，避免锁竞争
    atomic_int current_state_atomic;
    
    // 分离的锁机制
    struct k_mutex callback_mutex;   // 仅保护回调相关操作
    struct k_spinlock state_spinlock; // 轻量级自旋锁保护短暂操作
    
    // 智能轮询控制
    struct k_work_delayable status_check_work;
    atomic_int polling_interval_ms;
    atomic_bool polling_active;
    atomic_bool system_suspended;
    
    // 异步回调处理
    struct k_work callback_work;
    atomic_int pending_callback_state;
    
    // 回调函数
    charging_state_changed_cb_t callback;
    
    // GPIO设备
    const struct device *gpio_dev;
    
    // 初始化标志
    atomic_bool initialized;
};

// 获取私有数据实例
static struct charging_monitor_data *get_data(void)
{
    static struct charging_monitor_data data = {
        .current_state_atomic = ATOMIC_VAR_INIT(CHARGING_STATE_ERROR),
        .polling_interval_ms = ATOMIC_VAR_INIT(POLL_INTERVAL_DEFAULT_MS),
        .polling_active = ATOMIC_VAR_INIT(true),
        .system_suspended = ATOMIC_VAR_INIT(false),
        .pending_callback_state = ATOMIC_VAR_INIT(CHARGING_STATE_ERROR),
        .initialized = ATOMIC_VAR_INIT(false),
        .callback = NULL,
        .gpio_dev = NULL,
    };
    return &data;
}

// 异步回调工作函数 - 在系统工作队列中执行，避免在锁内执行用户代码
static void callback_work_handler(struct k_work *work)
{
    struct charging_monitor_data *data = get_data();
    charging_state_changed_cb_t callback;
    charging_state_t callback_state;
    
    // 原子获取回调状态
    callback_state = atomic_load(&data->pending_callback_state);
    
    // 安全获取回调函数指针
    k_mutex_lock(&data->callback_mutex, K_FOREVER);
    callback = data->callback;
    k_mutex_unlock(&data->callback_mutex);
    
    // 执行回调（无锁状态）
    if (callback) {
        callback(callback_state);
    }
}

// 智能轮询间隔计算
static uint32_t calculate_polling_interval(charging_state_t state)
{
    switch (state) {
    case CHARGING_STATE_CHARGING:
        return POLL_INTERVAL_CHARGING_MS;
    case CHARGING_STATE_FULL:
        return POLL_INTERVAL_FULL_MS;
    case CHARGING_STATE_ERROR:
        return POLL_INTERVAL_ERROR_MS;
    default:
        return POLL_INTERVAL_DEFAULT_MS;
    }
}

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = CONTAINER_OF(dwork, struct charging_monitor_data, status_check_work);
    
    // 检查是否已初始化
    if (!atomic_load(&data->initialized)) {
        LOG_WRN("Charging monitor not initialized");
        return;
    }
    
    // 检查系统是否挂起
    if (atomic_load(&data->system_suspended)) {
        LOG_DBG("System suspended, skipping poll");
        k_work_reschedule(dwork, K_MSEC(atomic_load(&data->polling_interval_ms)));
        return;
    }
    
    // 检查轮询是否激活
    if (!atomic_load(&data->polling_active)) {
        LOG_DBG("Polling paused");
        return;
    }
    
    // 读取CHRG引脚状态
    int pin_state;
    k_spinlock_key_t key = k_spin_lock(&data->state_spinlock);
    if (data->gpio_dev) {
        pin_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    } else {
        pin_state = -EIO;
    }
    k_spin_unlock(&data->state_spinlock, key);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        
        // 原子更新错误状态
        atomic_store(&data->current_state_atomic, CHARGING_STATE_ERROR);
        atomic_store(&data->polling_interval_ms, POLL_INTERVAL_ERROR_MS);
        
        // 调度下一次检查（错误状态间隔较长）
        k_work_reschedule(dwork, K_MSEC(POLL_INTERVAL_ERROR_MS));
        return;
    }
    
    // TP4056 CHRG引脚逻辑
    charging_state_t new_state;
    new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    
    // 原子获取当前状态进行比较
    charging_state_t current_state = atomic_load(&data->current_state_atomic);
    
    // 状态变化检测
    if (new_state != current_state) {
        const char *old_state_str = (current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : 
                                   (current_state == CHARGING_STATE_FULL) ? "FULL" : "ERROR";
        const char *new_state_str = (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL";
        
        LOG_INF("Charging state changed: %s -> %s", old_state_str, new_state_str);
        
        // 原子更新状态
        atomic_store(&data->current_state_atomic, new_state);
        
        // 设置待处理回调状态
        atomic_store(&data->pending_callback_state, new_state);
        
        // 提交异步回调工作（避免在锁内执行用户代码）
        k_work_submit(&data->callback_work);
        
        // 根据新状态更新轮询间隔
        uint32_t new_interval = calculate_polling_interval(new_state);
        atomic_store(&data->polling_interval_ms, new_interval);
    }
    
    // 智能调度下一次检查
    if (atomic_load(&data->polling_active)) {
        uint32_t interval = atomic_load(&data->polling_interval_ms);
        k_work_reschedule(dwork, K_MSEC(interval));
    }
}

// ZMK电源状态变化回调（可选）
#ifdef CONFIG_ZMK_PM_SOFT_OFF
static void on_pm_state_change(enum zmk_pm_state state)
{
    struct charging_monitor_data *data = get_data();
    
    switch (state) {
    case ZMK_PM_STATE_ACTIVE:
        atomic_store(&data->system_suspended, false);
        LOG_DBG("System resumed, resuming charging monitor");
        break;
    case ZMK_PM_STATE_SUSPENDED:
    case ZMK_PM_STATE_SOFT_OFF:
        atomic_store(&data->system_suspended, true);
        LOG_DBG("System suspended, pausing charging monitor");
        break;
    default:
        break;
    }
}
#endif

// 初始化充电监控器
int charging_monitor_init(void)
{
    struct charging_monitor_data *data = get_data();
    int ret;
    
    if (atomic_load(&data->initialized)) {
        LOG_WRN("Charging monitor already initialized");
        return 0;
    }
    
    LOG_DBG("Initializing charging monitor with thread safety optimizations");
    
    // 初始化互斥锁和自旋锁
    k_mutex_init(&data->callback_mutex);
    
    // 获取GPIO设备
    data->gpio_dev = CHRG_GPIO_DEV;
    if (!device_is_ready(data->gpio_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return -ENODEV;
    }
    
    // 配置CHRG引脚为输入，上拉电阻
    ret = gpio_pin_configure(data->gpio_dev, CHRG_GPIO_PIN, GPIO_INPUT | CHRG_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    LOG_INF("CHRG GPIO configured: GPIO1 pin %d, flags: 0x%x", CHRG_GPIO_PIN, CHRG_GPIO_FLAGS);
    
    // 初始化工作队列
    k_work_init_delayable(&data->status_check_work, status_check_work_handler);
    k_work_init(&data->callback_work, callback_work_handler);
    
    // 读取初始状态
    int initial_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    if (initial_state >= 0) {
        charging_state_t init_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        atomic_store(&data->current_state_atomic, init_state);
        atomic_store(&data->polling_interval_ms, calculate_polling_interval(init_state));
        
        LOG_INF("Initial charging state: %s", 
                (init_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        atomic_store(&data->current_state_atomic, CHARGING_STATE_ERROR);
        atomic_store(&data->polling_interval_ms, POLL_INTERVAL_ERROR_MS);
    }
    
    // 注册电源管理回调（如果可用）
    #ifdef CONFIG_ZMK_PM_SOFT_OFF
    static bool pm_callback_registered = false;
    if (!pm_callback_registered) {
        zmk_pm_register_callback(on_pm_state_change);
        pm_callback_registered = true;
        LOG_DBG("Registered power management callback");
    }
    #endif
    
    // 立即执行一次状态检查（无延迟）
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
    
    // 标记为已初始化
    atomic_store(&data->initialized, true);
    LOG_INF("Charging monitor initialized successfully with smart polling");
    
    return 0;
}

// 注册回调函数
int charging_monitor_register_callback(charging_state_changed_cb_t callback)
{
    struct charging_monitor_data *data = get_data();
    
    if (!atomic_load(&data->initialized)) {
        LOG_ERR("Charging monitor not initialized");
        return -ENODEV;
    }
    
    if (!callback) {
        LOG_ERR("Callback function is NULL");
        return -EINVAL;
    }
    
    k_mutex_lock(&data->callback_mutex, K_FOREVER);
    data->callback = callback;
    k_mutex_unlock(&data->callback_mutex);
    
    LOG_DBG("Callback registered");
    
    // 立即调用一次回调以设置初始状态（通过工作队列，避免直接调用）
    charging_state_t current_state = atomic_load(&data->current_state_atomic);
    atomic_store(&data->pending_callback_state, current_state);
    k_work_submit(&data->callback_work);
    
    return 0;
}

// 取消注册回调函数
int charging_monitor_unregister_callback(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!atomic_load(&data->initialized)) {
        return -ENODEV;
    }
    
    k_mutex_lock(&data->callback_mutex, K_FOREVER);
    data->callback = NULL;
    k_mutex_unlock(&data->callback_mutex);
    
    LOG_DBG("Callback unregistered");
    return 0;
}

// 获取当前充电状态
charging_state_t charging_monitor_get_state(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!atomic_load(&data->initialized)) {
        return CHARGING_STATE_ERROR;
    }
    
    // 原子读取状态，无需锁
    return atomic_load(&data->current_state_atomic);
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
    return atomic_load(&data->initialized);
}

// 设置轮询间隔
void charging_monitor_set_polling_interval(uint32_t interval_ms)
{
    struct charging_monitor_data *data = get_data();
    
    if (interval_ms < 100) {
        interval_ms = 100; // 最小100ms
    }
    
    atomic_store(&data->polling_interval_ms, interval_ms);
    LOG_DBG("Polling interval set to %u ms", interval_ms);
}

// 暂停轮询
void charging_monitor_pause(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!atomic_load(&data->initialized)) {
        return;
    }
    
    atomic_store(&data->polling_active, false);
    LOG_DBG("Charging monitor polling paused");
}

// 恢复轮询
void charging_monitor_resume(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!atomic_load(&data->initialized)) {
        return;
    }
    
    atomic_store(&data->polling_active, true);
    
    // 立即触发一次状态检查
    k_work_reschedule(&data->status_check_work, K_NO_WAIT);
    
    LOG_DBG("Charging monitor polling resumed");
}