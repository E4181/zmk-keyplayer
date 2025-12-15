#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 充电监控器实例
static struct {
    enum charging_state current_state;
    struct k_work_delayable status_check_work;
    charging_state_changed_cb_t callback;
    bool initialized;
    const struct device *gpio_dev;
    uint32_t chrg_pin;
    gpio_flags_t chrg_flags;
} monitor = {
    .current_state = CHARGING_STATE_ERROR,
    .initialized = false,
    .callback = NULL,
    .gpio_dev = NULL,
    .chrg_pin = 9,  // P1.09 默认值
    .chrg_flags = GPIO_ACTIVE_LOW
};

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (!monitor.initialized || !monitor.gpio_dev) {
        LOG_WRN("Charging monitor not initialized");
        return;
    }
    
    // 读取CHRG引脚状态
    int pin_state = gpio_pin_get(monitor.gpio_dev, monitor.chrg_pin);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        monitor.current_state = CHARGING_STATE_ERROR;
        
        // 继续调度下一次检查
        k_work_reschedule(&monitor.status_check_work, K_SECONDS(5));
        return;
    }
    
    // TP4056 CHRG引脚逻辑:
    // - 低电平(0): 正在充电
    // - 高电平(1): 已充满
    enum charging_state new_state;
    
    if (monitor.chrg_flags & GPIO_ACTIVE_LOW) {
        // 如果配置为低电平有效
        new_state = (pin_state == 0) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    } else {
        // 如果配置为高电平有效
        new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    }
    
    // 状态变化检测
    if (new_state != monitor.current_state) {
        LOG_INF("Charging state changed: %s -> %s", 
                charging_monitor_get_state_str(), 
                (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
        
        monitor.current_state = new_state;
        
        // 调用回调函数
        if (monitor.callback) {
            monitor.callback(new_state);
        }
    }
    
    // 调度下一次检查
    k_work_reschedule(&monitor.status_check_work, K_SECONDS(5));
}

// 初始化充电监控器
int charging_monitor_init(void)
{
    int ret;
    
    LOG_DBG("Initializing charging monitor");
    
    // 获取GPIO设备 - 使用设备树标签
    monitor.gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (!monitor.gpio_dev) {
        LOG_ERR("Failed to get GPIO1 device");
        return -ENODEV;
    }
    
    if (!device_is_ready(monitor.gpio_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return -ENODEV;
    }
    
    // 配置CHRG引脚为输入，上拉电阻（TP4056的CHRG引脚是开漏输出）
    ret = gpio_pin_configure(monitor.gpio_dev, monitor.chrg_pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    // 初始化工作队列
    k_work_init_delayable(&monitor.status_check_work, status_check_work_handler);
    
    // 读取初始状态
    int initial_state = gpio_pin_get(monitor.gpio_dev, monitor.chrg_pin);
    if (initial_state >= 0) {
        if (monitor.chrg_flags & GPIO_ACTIVE_LOW) {
            monitor.current_state = (initial_state == 0) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        } else {
            monitor.current_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        }
        LOG_INF("Initial charging state: %s", charging_monitor_get_state_str());
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        monitor.current_state = CHARGING_STATE_ERROR;
    }
    
    // 延迟启动状态监控
    k_work_reschedule(&monitor.status_check_work, K_SECONDS(3));
    
    monitor.initialized = true;
    LOG_INF("Charging monitor initialized successfully");
    
    return 0;
}

// 设置CHRG引脚配置（可选，如果不调用则使用默认值）
int charging_monitor_set_config(const struct device *gpio_dev, uint32_t pin, gpio_flags_t flags)
{
    if (monitor.initialized) {
        LOG_ERR("Cannot set config after initialization");
        return -EINVAL;
    }
    
    if (gpio_dev) {
        monitor.gpio_dev = gpio_dev;
    }
    
    monitor.chrg_pin = pin;
    monitor.chrg_flags = flags;
    
    return 0;
}

// 注册回调函数
int charging_monitor_register_callback(charging_state_changed_cb_t callback)
{
    if (!monitor.initialized) {
        LOG_ERR("Charging monitor not initialized");
        return -ENODEV;
    }
    
    if (!callback) {
        LOG_ERR("Callback function is NULL");
        return -EINVAL;
    }
    
    monitor.callback = callback;
    LOG_DBG("Callback registered");
    
    // 立即调用一次回调以设置初始状态
    callback(monitor.current_state);
    
    return 0;
}

// 获取当前充电状态
enum charging_state charging_monitor_get_state(void)
{
    if (!monitor.initialized) {
        return CHARGING_STATE_ERROR;
    }
    
    return monitor.current_state;
}

// 手动触发状态检查
void charging_monitor_check_status(void)
{
    if (monitor.initialized) {
        k_work_reschedule(&monitor.status_check_work, K_NO_WAIT);
    }
}

// 获取充电状态字符串
const char* charging_monitor_get_state_str(void)
{
    switch (monitor.current_state) {
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