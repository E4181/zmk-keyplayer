#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 充电监控器实例
static struct {
    enum charging_state current_state;
    struct k_work_delayable status_check_work;
    charging_state_changed_cb_t callbacks[5]; // 支持最多5个回调
    uint8_t callback_count;
    bool initialized;
    bool gpio_configured;
} monitor = {
    .current_state = CHARGING_STATE_ERROR,
    .initialized = false,
    .callback_count = 0,
    .gpio_configured = false
};

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (!monitor.initialized || !monitor.gpio_configured) {
        return;
    }

    // 读取CHRG引脚状态 (P1.09)
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    int pin_state = gpio_pin_get(gpio_dev, 9);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read charging pin: %d", pin_state);
        return;
    }
    
    enum charging_state new_state;
    
    // 根据TP4056的典型电平逻辑：
    // - 充电时：低电平
    // - 充满/未充电：高电平
    if (pin_state) {
        new_state = CHARGING_STATE_DISCHARGING;
    } else {
        new_state = CHARGING_STATE_CHARGING;
    }
    
    // 状态变化检测
    if (new_state != monitor.current_state) {
        LOG_INF("Charging state changed: %s -> %s", 
                charging_monitor_get_state_str(), 
                (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "NOT CHARGING");
        
        monitor.current_state = new_state;
        
        // 调用所有注册的回调函数
        for (int i = 0; i < monitor.callback_count; i++) {
            if (monitor.callbacks[i]) {
                monitor.callbacks[i](new_state);
            }
        }
    }
    
    // 每5秒检查一次状态，减少频率
    k_work_reschedule(&monitor.status_check_work, K_SECONDS(5));
}

// 初始化充电监控器
int charging_monitor_init(void)
{
    int ret;
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return -ENODEV;
    }
    
    // 配置P1.09为输入模式，上拉电阻
    ret = gpio_pin_configure(gpio_dev, 9, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin: %d", ret);
        return ret;
    }
    
    monitor.gpio_configured = true;
    LOG_INF("GPIO P1.09 configured for TP4056 status detection");
    
    // 初始化工作队列
    k_work_init_delayable(&monitor.status_check_work, status_check_work_handler);
    
    // 读取初始状态
    int initial_state = gpio_pin_get(gpio_dev, 9);
    if (initial_state >= 0) {
        monitor.current_state = (initial_state == 0) ? CHARGING_STATE_CHARGING : CHARGING_STATE_DISCHARGING;
        LOG_INF("Initial charging state: %s", charging_monitor_get_state_str());
    }
    
    // 延迟启动状态监控，确保键盘功能先初始化
    k_work_reschedule(&monitor.status_check_work, K_SECONDS(3));
    monitor.initialized = true;
    
    LOG_INF("Charging monitor initialized successfully");
    return 0;
}

// 注册回调函数
int charging_monitor_register_callback(charging_state_changed_cb_t callback)
{
    if (monitor.callback_count >= 5) {
        LOG_ERR("Too many callbacks registered");
        return -ENOMEM;
    }
    
    if (!monitor.initialized) {
        LOG_ERR("Charging monitor not initialized");
        return -ENODEV;
    }
    
    // 注册回调
    monitor.callbacks[monitor.callback_count++] = callback;
    LOG_INF("Callback registered, total callbacks: %d", monitor.callback_count);
    
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
    case CHARGING_STATE_DISCHARGING:
        return "DISCHARGING";
    case CHARGING_STATE_FULL:
        return "FULL";
    case CHARGING_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}