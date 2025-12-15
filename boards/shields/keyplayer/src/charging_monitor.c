#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 设备树匹配表
#define DT_DRV_COMPAT zmk_charging_monitor

// 设备配置数据结构
struct charging_monitor_config {
    struct gpio_dt_spec chrg_gpio;
    bool chrg_active_low;
};

// 设备私有数据
struct charging_monitor_data {
    enum charging_state current_state;
    struct k_work_delayable status_check_work;
    charging_state_changed_cb_t callback;
    bool initialized;
    struct k_mutex state_mutex;
};

// 获取设备配置
static const struct charging_monitor_config *get_config(void)
{
    static const struct charging_monitor_config config = {
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(0, chrg_gpios),
        .chrg_active_low = DT_INST_PROP(0, chrg_active_low),
    };
    return &config;
}

// 获取设备数据
static struct charging_monitor_data *get_data(void)
{
    static struct charging_monitor_data data = {
        .current_state = CHARGING_STATE_ERROR,
        .initialized = false,
    };
    return &data;
}

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = CONTAINER_OF(dwork, struct charging_monitor_data, status_check_work);
    const struct charging_monitor_config *config = get_config();
    
    if (!data->initialized) {
        LOG_WRN("Charging monitor not initialized");
        return;
    }
    
    if (!gpio_is_ready_dt(&config->chrg_gpio)) {
        LOG_ERR("CHRG GPIO device not ready");
        data->current_state = CHARGING_STATE_ERROR;
        k_work_reschedule(&data->status_check_work, K_SECONDS(10)); // 10秒后重试
        return;
    }
    
    // 读取CHRG引脚状态
    int pin_state = gpio_pin_get_dt(&config->chrg_gpio);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        data->current_state = CHARGING_STATE_ERROR;
        k_work_reschedule(&data->status_check_work, K_SECONDS(5));
        return;
    }
    
    // TP4056 CHRG引脚逻辑:
    // - 低电平(0): 正在充电
    // - 高电平(1): 已充满
    enum charging_state new_state;
    
    if (config->chrg_active_low) {
        // 低电平有效
        new_state = (pin_state == 0) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    } else {
        // 高电平有效
        new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    }
    
    // 状态变化检测
    if (new_state != data->current_state) {
        LOG_INF("Charging state changed: %s -> %s", 
                (data->current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : 
                (data->current_state == CHARGING_STATE_FULL) ? "FULL" : "ERROR",
                (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
        
        k_mutex_lock(&data->state_mutex, K_FOREVER);
        data->current_state = new_state;
        k_mutex_unlock(&data->state_mutex);
        
        // 调用回调函数
        if (data->callback) {
            data->callback(new_state);
        }
    }
    
    // 调度下一次检查
    k_work_reschedule(&data->status_check_work, K_SECONDS(5));
}

// 初始化充电监控器
int charging_monitor_init(void)
{
    struct charging_monitor_data *data = get_data();
    const struct charging_monitor_config *config = get_config();
    int ret;
    
    if (data->initialized) {
        LOG_WRN("Charging monitor already initialized");
        return 0;
    }
    
    LOG_DBG("Initializing charging monitor");
    
    // 检查设备树配置
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device not ready");
        return -ENODEV;
    }
    
    // 配置CHRG引脚为输入，上拉电阻
    ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    // 初始化互斥锁
    k_mutex_init(&data->state_mutex);
    
    // 初始化工作队列
    k_work_init_delayable(&data->status_check_work, status_check_work_handler);
    
    // 读取初始状态
    int initial_state = gpio_pin_get_dt(&config->chrg_gpio);
    if (initial_state >= 0) {
        if (config->chrg_active_low) {
            data->current_state = (initial_state == 0) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        } else {
            data->current_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        }
        LOG_INF("Initial charging state: %s", 
                (data->current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        data->current_state = CHARGING_STATE_ERROR;
    }
    
    // 延迟启动状态监控
    k_work_reschedule(&data->status_check_work, K_SECONDS(3));
    
    data->initialized = true;
    LOG_INF("Charging monitor initialized successfully (GPIO: %s, pin: %d, active: %s)",
            config->chrg_gpio.port->name, config->chrg_gpio.pin,
            config->chrg_active_low ? "low" : "high");
    
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
    
    // 设置回调函数
    data->callback = callback;
    LOG_DBG("Callback registered");
    
    // 立即调用一次回调以设置初始状态
    k_mutex_lock(&data->state_mutex, K_FOREVER);
    enum charging_state current_state = data->current_state;
    k_mutex_unlock(&data->state_mutex);
    
    callback(current_state);
    
    return 0;
}

// 获取当前充电状态
enum charging_state charging_monitor_get_state(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return CHARGING_STATE_ERROR;
    }
    
    enum charging_state state;
    k_mutex_lock(&data->state_mutex, K_FOREVER);
    state = data->current_state;
    k_mutex_unlock(&data->state_mutex);
    
    return state;
}

// 获取充电状态字符串
const char* charging_monitor_get_state_str(void)
{
    struct charging_monitor_data *data = get_data();
    
    switch (data->current_state) {
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