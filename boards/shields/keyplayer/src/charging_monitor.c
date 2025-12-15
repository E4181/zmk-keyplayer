#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charging_monitor, CONFIG_ZMK_LOG_LEVEL);

#include "charging_monitor.h"

// 硬编码GPIO配置 - ZMK常用方式
#define CHRG_GPIO_DEV DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define CHRG_GPIO_PIN 9  // P1.09
#define CHRG_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

// 充电监控器私有数据结构
struct charging_monitor_data {
    charging_state_t current_state;
    struct k_work_delayable status_check_work;
    charging_state_changed_cb_t callback;
    bool initialized;
    struct k_mutex state_mutex;
    const struct device *gpio_dev;
};

// 获取私有数据实例
static struct charging_monitor_data *get_data(void)
{
    static struct charging_monitor_data data = {
        .current_state = CHARGING_STATE_ERROR,
        .initialized = false,
        .callback = NULL,
        .gpio_dev = NULL,
    };
    return &data;
}

// 状态检查工作函数
static void status_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = CONTAINER_OF(dwork, struct charging_monitor_data, status_check_work);
    
    if (!data->initialized || !data->gpio_dev) {
        LOG_WRN("Charging monitor not initialized");
        return;
    }
    
    // 读取CHRG引脚状态
    int pin_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        data->current_state = CHARGING_STATE_ERROR;
        k_work_reschedule(&data->status_check_work, K_SECONDS(5));
        return;
    }
    
    // TP4056 CHRG引脚逻辑:
    // - 物理低电平 (0): 正在充电 (由于GPIO_ACTIVE_LOW，返回1)
    // - 物理高电平 (1): 已充满 (由于GPIO_ACTIVE_LOW，返回0)
    charging_state_t new_state;
    
    // GPIO_ACTIVE_LOW 表示低电平有效，所以：
    // pin_state == 1 表示引脚有效（正在充电）
    // pin_state == 0 表示引脚无效（已充满）
    new_state = (pin_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
    
    // 状态变化检测
    if (new_state != data->current_state) {
        const char *old_state_str = charging_monitor_get_state_str();
        const char *new_state_str = (new_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL";
        
        LOG_INF("Charging state changed: %s -> %s", old_state_str, new_state_str);
        
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
    int ret;
    
    if (data->initialized) {
        LOG_WRN("Charging monitor already initialized");
        return 0;
    }
    
    LOG_DBG("Initializing charging monitor");
    
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
    
    // 初始化互斥锁
    k_mutex_init(&data->state_mutex);
    
    // 初始化工作队列
    k_work_init_delayable(&data->status_check_work, status_check_work_handler);
    
    // 读取初始状态
    int initial_state = gpio_pin_get(data->gpio_dev, CHRG_GPIO_PIN);
    if (initial_state >= 0) {
        data->current_state = (initial_state == 1) ? CHARGING_STATE_CHARGING : CHARGING_STATE_FULL;
        LOG_INF("Initial charging state: %s", 
                (data->current_state == CHARGING_STATE_CHARGING) ? "CHARGING" : "FULL");
    } else {
        LOG_ERR("Failed to read initial CHRG pin state: %d", initial_state);
        data->current_state = CHARGING_STATE_ERROR;
    }
    
    // 延迟3秒启动状态监控
    k_work_reschedule(&data->status_check_work, K_SECONDS(3));
    
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
    
    data->callback = callback;
    LOG_DBG("Callback registered");
    
    // 立即调用一次回调以设置初始状态
    k_mutex_lock(&data->state_mutex, K_FOREVER);
    charging_state_t current_state = data->current_state;
    k_mutex_unlock(&data->state_mutex);
    
    callback(current_state);
    
    return 0;
}

// 获取当前充电状态
charging_state_t charging_monitor_get_state(void)
{
    struct charging_monitor_data *data = get_data();
    
    if (!data->initialized) {
        return CHARGING_STATE_ERROR;
    }
    
    charging_state_t state;
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