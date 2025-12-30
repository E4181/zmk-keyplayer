/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_charging_led

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "charging_led.h"

LOG_MODULE_REGISTER(charging_led, CONFIG_CHARGING_LED_LOG_LEVEL);

struct charging_led_data {
    const struct device *dev;
    bool enabled;
    bool last_charging_state;
};

struct charging_led_config {
    struct gpio_dt_spec led_gpio;
    bool active_low;  // true: LED亮时输出低电平; false: LED亮时输出高电平
};

static void update_led_state(const struct device *dev, bool charging) {
    const struct charging_led_config *config = dev->config;
    struct charging_led_data *data = dev->data;
    
    if (!data->enabled || !device_is_ready(config->led_gpio.port)) {
        return;
    }
    
    // 根据充电状态和极性设置LED
    int led_value = charging ? 1 : 0;
    
    // 如果是active_low，需要反转逻辑
    if (config->active_low) {
        led_value = !led_value;
    }
    
    int ret = gpio_pin_set_dt(&config->led_gpio, led_value);
    if (ret < 0) {
        LOG_ERR("Failed to set LED pin: %d", ret);
    } else {
        data->last_charging_state = charging;
        LOG_DBG("LED %s (charging: %s)", 
                charging ? "ON" : "OFF",
                charging ? "yes" : "no");
    }
}

static void charging_led_set_charging_impl(const struct device *dev, bool charging) {
    struct charging_led_data *data = dev->data;
    
    if (!data->enabled) {
        LOG_WRN("Charging LED is disabled");
        return;
    }
    
    update_led_state(dev, charging);
}

static void charging_led_enable_impl(const struct device *dev) {
    struct charging_led_data *data = dev->data;
    
    if (data->enabled) {
        return;  // 已经启用
    }
    
    data->enabled = true;
    LOG_INF("Charging LED enabled");
    
    // 恢复到上次的状态
    update_led_state(dev, data->last_charging_state);
}

static void charging_led_disable_impl(const struct device *dev) {
    struct charging_led_data *data = dev->data;
    
    if (!data->enabled) {
        return;  // 已经禁用
    }
    
    data->enabled = false;
    
    // 关闭LED
    const struct charging_led_config *config = dev->config;
    if (device_is_ready(config->led_gpio.port)) {
        int led_value = config->active_low ? 1 : 0;  // 关闭状态
        gpio_pin_set_dt(&config->led_gpio, led_value);
    }
    
    LOG_INF("Charging LED disabled");
}

static int charging_led_init(const struct device *dev) {
    struct charging_led_data *data = dev->data;
    const struct charging_led_config *config = dev->config;
    int ret;
    
    data->dev = dev;
    data->enabled = true;  // 默认启用
    data->last_charging_state = false;  // 默认未充电
    
    if (!device_is_ready(config->led_gpio.port)) {
        LOG_ERR("LED GPIO device is not ready");
        return -ENODEV;
    }
    
    // 配置LED GPIO为输出
    ret = gpio_pin_configure_dt(&config->led_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO: %d", ret);
        return ret;
    }
    
    // 初始状态：关闭LED
    int init_value = config->active_low ? 1 : 0;
    gpio_pin_set_dt(&config->led_gpio, init_value);
    
    LOG_INF("Charging LED initialized on pin %s:%d (active_low: %s)", 
            config->led_gpio.port->name, config->led_gpio.pin,
            config->active_low ? "yes" : "no");
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int charging_led_pm_action(const struct device *dev,
                                 enum pm_device_action action) {
    struct charging_led_data *data = dev->data;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        // 挂起时关闭LED
        charging_led_disable_impl(dev);
        LOG_DBG("Charging LED suspended");
        return 0;
        
    case PM_DEVICE_ACTION_RESUME:
        // 恢复时启用LED
        charging_led_enable_impl(dev);
        LOG_DBG("Charging LED resumed");
        return 0;
        
    default:
        return -ENOTSUP;
    }
}

#endif /* CONFIG_PM_DEVICE */

// 定义驱动API结构体
static const struct charging_led_driver_api charging_led_api = {
    .set_charging = charging_led_set_charging_impl,
    .enable = charging_led_enable_impl,
    .disable = charging_led_disable_impl,
};

#define CHARGING_LED_INIT(n) \
    static struct charging_led_data charging_led_data_##n; \
    static const struct charging_led_config charging_led_config_##n = { \
        .led_gpio = GPIO_DT_SPEC_INST_GET(n, led_gpios), \
        .active_low = DT_INST_PROP(n, active_low), \
    }; \
    PM_DEVICE_DT_INST_DEFINE(n, charging_led_pm_action); \
    DEVICE_DT_INST_DEFINE(n, \
                         charging_led_init, \
                         PM_DEVICE_DT_INST_GET(n), \
                         &charging_led_data_##n, \
                         &charging_led_config_##n, \
                         POST_KERNEL, \
                         CONFIG_CHARGING_LED_INIT_PRIORITY, \
                         &charging_led_api);

DT_INST_FOREACH_STATUS_OKAY(CHARGING_LED_INIT)