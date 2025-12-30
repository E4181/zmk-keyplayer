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
    
    // Determine charging status based on TP4056 CHRG pin behavior
    enum charging_status new_status;
    
    if (pin_state == 0) {
        // CHRG pin is low (active low) - charging in progress
        new_status = CHARGING_STATUS_CHARGING;
    } else {
        // CHRG pin is high - not charging or fully charged
        new_status = CHARGING_STATUS_IDLE;
        
        // TODO: Could add fault detection logic here if needed
        // Some TP4056 variants pulse the CHRG pin on fault conditions
    }
    
    // Update status if changed
    if (new_status != data->current_status) {
        data->last_status = data->current_status;
        data->current_status = new_status;
        
        LOG_DBG("Charging status changed: %d -> %d", 
                data->last_status, data->current_status);
        
        // Notify registered callback
        if (data->callback_registered) {
            data->user_callback.function(data->dev, new_status, 
                                        data->user_callback.user_data);
        }
        
        // Raise ZMK event for system-wide notification
        raise_zmk_charging_state_changed((struct charging_state_changed){
            .is_charging = (new_status == CHARGING_STATUS_CHARGING),
            .timestamp = k_uptime_get()
        });
    }
}

static void charging_monitor_debounce_timer_handler(struct k_timer *timer) {
    struct charging_monitor_data *data =
        CONTAINER_OF(timer, struct charging_monitor_data, debounce_timer);
    k_work_submit(&data->work);
}

static void charging_monitor_gpio_callback(const struct device *port,
                                          struct gpio_callback *cb,
                                          gpio_port_pins_t pins) {
    struct charging_monitor_data *data =
        CONTAINER_OF(cb, struct charging_monitor_data, gpio_cb);
    
    // Start debounce timer to handle pin bouncing
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
    
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device is not ready");
        return -ENODEV;
    }
    
    // Configure CHRG pin as input with pull-up
    // TP4056 CHRG is open-drain output, needs pull-up
    // Use the GPIO flags from device tree (should include GPIO_ACTIVE_LOW)
    ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    // Configure interrupt on both edges (charging start/stop)
    ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, 
                                         GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO interrupt: %d", ret);
        return ret;
    }
    
    // Initialize work and timer
    k_work_init(&data->work, charging_monitor_work_handler);
    k_timer_init(&data->debounce_timer, 
                 charging_monitor_debounce_timer_handler, NULL);
    
    // Setup GPIO callback
    gpio_init_callback(&data->gpio_cb, charging_monitor_gpio_callback,
                       BIT(config->chrg_gpio.pin));
    
    ret = gpio_add_callback(config->chrg_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    // Perform initial read
    k_work_submit(&data->work);
    
    LOG_INF("Charging monitor initialized on pin %s:%d", 
            config->chrg_gpio.port->name, config->chrg_gpio.pin);
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int charging_monitor_pm_action(const struct device *dev,
                                     enum pm_device_action action) {
    const struct charging_monitor_config *config = dev->config;
    int ret;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        // Disable interrupt to save power
        ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, 
                                             GPIO_INT_DISABLE);
        if (ret < 0) {
            LOG_WRN("Failed to disable interrupt on suspend: %d", ret);
        }
        return 0;
        
    case PM_DEVICE_ACTION_RESUME:
        // Re-enable interrupt
        ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio,
                                             GPIO_INT_EDGE_BOTH);
        if (ret < 0) {
            LOG_WRN("Failed to enable interrupt on resume: %d", ret);
        }
        
        // Trigger a fresh read
        struct charging_monitor_data *data = dev->data;
        k_work_submit(&data->work);
        return 0;
        
    default:
        return -ENOTSUP;
    }
}

#endif /* CONFIG_PM_DEVICE */

static enum charging_status charging_monitor_get_status(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    return data->current_status;
}

static bool charging_monitor_is_charging(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    return (data->current_status == CHARGING_STATUS_CHARGING);
}

static int charging_monitor_register_callback(const struct device *dev,
                                             void (*callback)(const struct device *dev,
                                                             enum charging_status status,
                                                             void *user_data),
                                             void *user_data) {
    struct charging_monitor_data *data = dev->data;
    
    if (callback == NULL) {
        return -EINVAL;
    }
    
    data->user_callback.function = callback;
    data->user_callback.user_data = user_data;
    data->callback_registered = true;
    
    return 0;
}

static const struct charging_monitor_driver_api charging_monitor_api = {
    .get_status = charging_monitor_get_status,
    .is_charging = charging_monitor_is_charging,
    .register_callback = charging_monitor_register_callback,
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