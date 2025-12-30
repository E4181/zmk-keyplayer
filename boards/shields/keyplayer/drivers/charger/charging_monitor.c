/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_charging_monitor

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <zmk/event_manager.h>
#include "charging_state_changed.h"
#include "charging_monitor.h"

LOG_MODULE_REGISTER(charging_monitor, CONFIG_CHARGING_MONITOR_LOG_LEVEL);

struct charging_monitor_callback {
    void (*function)(const struct device *dev, enum charging_status status, void *user_data);
    void *user_data;
};

struct charging_monitor_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback gpio_cb;
    enum charging_status current_status;
    enum charging_status last_status;
    struct k_timer debounce_timer;
    struct charging_monitor_callback user_callback;
    bool callback_registered;
};

struct charging_monitor_config {
    struct gpio_dt_spec chrg_gpio;
    uint32_t debounce_ms;
};

static void charging_monitor_work_handler(struct k_work *work) {
    struct charging_monitor_data *data =
        CONTAINER_OF(work, struct charging_monitor_data, work);
    const struct charging_monitor_config *config = data->dev->config;
    
    int pin_state = gpio_pin_get_dt(&config->chrg_gpio);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        return;
    }
    
    // 详细记录引脚状态
    LOG_DBG("CHRG pin read: %d (0=charging, 1=idle)", pin_state);
    
    // 确定充电状态：TP4056 CHRG引脚低电平表示正在充电
    enum charging_status new_status;
    
    if (pin_state == 0) {
        new_status = CHARGING_STATUS_CHARGING;
        LOG_INF("CHRG pin LOW (0) -> CHARGING");
    } else {
        new_status = CHARGING_STATUS_IDLE;
        LOG_INF("CHRG pin HIGH (1) -> IDLE");
    }
    
    // 更新状态
    if (new_status != data->current_status) {
        data->last_status = data->current_status;
        data->current_status = new_status;
        
        LOG_INF("Charging status changed: %s -> %s", 
                data->last_status == CHARGING_STATUS_CHARGING ? "CHARGING" : "IDLE",
                new_status == CHARGING_STATUS_CHARGING ? "CHARGING" : "IDLE");
        
        // 通知注册的回调
        if (data->callback_registered) {
            LOG_DBG("Notifying registered callback");
            data->user_callback.function(data->dev, new_status, 
                                        data->user_callback.user_data);
        }
        
        // 发布充电状态变化事件
        LOG_DBG("Raising charging state changed event");
        raise_charging_state_changed((struct charging_state_changed){
            .is_charging = (new_status == CHARGING_STATUS_CHARGING),
            .timestamp = k_uptime_get()
        });
    } else {
        LOG_DBG("Charging status unchanged: %s", 
                new_status == CHARGING_STATUS_CHARGING ? "CHARGING" : "IDLE");
    }
}

static void charging_monitor_debounce_timer_handler(struct k_timer *timer) {
    struct charging_monitor_data *data =
        CONTAINER_OF(timer, struct charging_monitor_data, debounce_timer);
    LOG_DBG("Debounce timer expired, submitting work");
    k_work_submit(&data->work);
}

static void charging_monitor_gpio_callback(const struct device *port,
                                          struct gpio_callback *cb,
                                          gpio_port_pins_t pins) {
    struct charging_monitor_data *data =
        CONTAINER_OF(cb, struct charging_monitor_data, gpio_cb);
    
    LOG_DBG("GPIO interrupt triggered on CHRG pin");
    
    // 启动去抖动定时器
    const struct charging_monitor_config *config = data->dev->config;
    k_timer_start(&data->debounce_timer, 
                  K_MSEC(config->debounce_ms), K_NO_WAIT);
}

static int charging_monitor_init(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    const struct charging_monitor_config *config = dev->config;
    int ret;
    
    data->dev = dev;
    data->current_status = CHARGING_STATUS_IDLE;
    data->last_status = CHARGING_STATUS_IDLE;
    data->callback_registered = false;
    
    LOG_INF("Initializing charging monitor...");
    
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device is not ready");
        LOG_ERR("Port: %s, Pin: %d", 
                config->chrg_gpio.port->name, config->chrg_gpio.pin);
        return -ENODEV;
    }
    
    LOG_INF("CHRG GPIO device ready: %s:%d", 
            config->chrg_gpio.port->name, config->chrg_gpio.pin);
    
    // 配置CHRG引脚为输入
    ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    LOG_INF("CHRG GPIO configured as input");
    
    // 立即读取一次引脚状态
    int init_state = gpio_pin_get_dt(&config->chrg_gpio);
    if (init_state < 0) {
        LOG_ERR("Failed to read initial CHRG pin state: %d", init_state);
    } else {
        LOG_INF("Initial CHRG pin state: %d (0=charging, 1=idle)", init_state);
    }
    
    // 配置中断
    ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, 
                                         GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO interrupt: %d", ret);
        return ret;
    }
    
    LOG_INF("CHRG GPIO interrupt configured (edge both)");
    
    // 初始化工作队列和定时器
    k_work_init(&data->work, charging_monitor_work_handler);
    k_timer_init(&data->debounce_timer, 
                 charging_monitor_debounce_timer_handler, NULL);
    
    LOG_INF("Work and timer initialized");
    
    // 设置GPIO回调
    gpio_init_callback(&data->gpio_cb, charging_monitor_gpio_callback,
                       BIT(config->chrg_gpio.pin));
    
    ret = gpio_add_callback(config->chrg_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO callback added successfully");
    
    // 执行初始状态读取
    LOG_INF("Submitting initial work...");
    k_work_submit(&data->work);
    
    LOG_INF("Charging monitor fully initialized");
    LOG_INF("  CHRG pin: %s:%d", config->chrg_gpio.port->name, config->chrg_gpio.pin);
    LOG_INF("  Debounce time: %d ms", config->debounce_ms);
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int charging_monitor_pm_action(const struct device *dev,
                                     enum pm_device_action action) {
    const struct charging_monitor_config *config = dev->config;
    struct charging_monitor_data *data = dev->data;
    int ret;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_INF("Suspending charging monitor");
        ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, 
                                             GPIO_INT_DISABLE);
        if (ret < 0) {
            LOG_WRN("Failed to disable interrupt on suspend: %d", ret);
        }
        return ret;
        
    case PM_DEVICE_ACTION_RESUME:
        LOG_INF("Resuming charging monitor");
        ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio,
                                             GPIO_INT_EDGE_BOTH);
        if (ret < 0) {
            LOG_WRN("Failed to enable interrupt on resume: %d", ret);
        }
        k_work_submit(&data->work);
        return ret;
        
    default:
        return -ENOTSUP;
    }
}

#endif /* CONFIG_PM_DEVICE */

// 驱动API函数实现
static enum charging_status charging_monitor_get_status_impl(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    LOG_DBG("get_status called, returning: %s", 
            data->current_status == CHARGING_STATUS_CHARGING ? "CHARGING" : "IDLE");
    return data->current_status;
}

static bool charging_monitor_is_charging_impl(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    bool is_charging = (data->current_status == CHARGING_STATUS_CHARGING);
    LOG_DBG("is_charging called, returning: %s", is_charging ? "true" : "false");
    return is_charging;
}

static int charging_monitor_register_callback_impl(const struct device *dev,
                                                   void (*callback)(const struct device *dev,
                                                                   enum charging_status status,
                                                                   void *user_data),
                                                   void *user_data) {
    struct charging_monitor_data *data = dev->data;
    
    if (callback == NULL) {
        LOG_ERR("Cannot register NULL callback");
        return -EINVAL;
    }
    
    data->user_callback.function = callback;
    data->user_callback.user_data = user_data;
    data->callback_registered = true;
    
    LOG_INF("Callback registered: %p", callback);
    
    return 0;
}

// 定义驱动API结构体
static const struct charging_monitor_driver_api charging_monitor_api = {
    .get_status = charging_monitor_get_status_impl,
    .is_charging = charging_monitor_is_charging_impl,
    .register_callback = charging_monitor_register_callback_impl,
};

#define CHARGING_MONITOR_INIT(n) \
    static struct charging_monitor_data charging_monitor_data_##n; \
    static const struct charging_monitor_config charging_monitor_config_##n = { \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(n, chrg_gpios), \
        .debounce_ms = DT_INST_PROP_OR(n, debounce_ms, 50), \
    }; \
    PM_DEVICE_DT_INST_DEFINE(n, charging_monitor_pm_action); \
    DEVICE_DT_INST_DEFINE(n, \
                         charging_monitor_init, \
                         PM_DEVICE_DT_INST_GET(n), \
                         &charging_monitor_data_##n, \
                         &charging_monitor_config_##n, \
                         POST_KERNEL, \
                         CONFIG_CHARGING_MONITOR_INIT_PRIORITY, \
                         &charging_monitor_api);

DT_INST_FOREACH_STATUS_OKAY(CHARGING_MONITOR_INIT)