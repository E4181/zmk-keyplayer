/*
 * Copyright (c) 2024 ZMK Project
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_tp4056_charger

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <drivers/charger/tp4056.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_REGISTER(tp4056, CONFIG_TP4056_LOG_LEVEL);

/* Forward declarations */
static int tp4056_pm_action(const struct device *dev, enum pm_device_action action);

/* Device configuration structure */
struct tp4056_config {
    struct gpio_dt_spec chrg_gpio;
    uint32_t poll_interval_ms;
    uint32_t debounce_ms;
    bool active_low;
};

/* Device data structure */
struct tp4056_data {
    const struct device *dev;
    
    /* GPIO interrupt callback */
    struct gpio_callback chrg_cb;
    
    /* Polling work */
    struct k_work_delayable poll_work;
    struct k_work_delayable debounce_work;
    
    /* State management */
    enum tp4056_charging_state state;
    enum tp4056_charging_state pending_state;
    bool initialized;
    
    /* Callback management */
    tp4056_callback_t callback;
    void *callback_user_data;
    
    /* Power management */
    bool pm_suspended;
    
#if CONFIG_TP4056_STATISTICS
    /* Statistics */
    uint32_t state_change_count;
    uint64_t total_charging_time;
    uint64_t last_state_change_time;
#endif
};

/* Helper function to get GPIO state */
static bool tp4056_get_gpio_state(const struct device *dev)
{
    const struct tp4056_config *config = dev->config;
    int ret = gpio_pin_get_dt(&config->chrg_gpio);
    
    if (ret < 0) {
        LOG_ERR("Failed to read GPIO state: %d", ret);
        return false;
    }
    
    /* CHRG pin is active low, so invert if needed */
    bool state = (ret != 0);
    return config->active_low ? !state : state;
}

/* Convert GPIO state to charging state */
static enum tp4056_charging_state tp4056_gpio_to_state(bool gpio_state)
{
    /* TP4056 CHRG pin: LOW = charging, HIGH = not charging (or full) */
    return gpio_state ? TP4056_NOT_CHARGING : TP4056_CHARGING;
}

/* Update charging state */
static int tp4056_update_state(const struct device *dev, bool from_isr)
{
    struct tp4056_data *data = dev->data;
    const struct tp4056_config *config = dev->config;
    
    if (!data->initialized) {
        return -EBUSY;
    }
    
    bool gpio_state = tp4056_get_gpio_state(dev);
    enum tp4056_charging_state new_state = tp4056_gpio_to_state(gpio_state);
    
    if (new_state == data->state) {
        return 0; /* No change */
    }
    
    LOG_INF("Charging state changed: %s -> %s",
            data->state == TP4056_CHARGING ? "CHARGING" :
            data->state == TP4056_NOT_CHARGING ? "NOT_CHARGING" : "UNKNOWN",
            new_state == TP4056_CHARGING ? "CHARGING" : "NOT_CHARGING");
    
#if CONFIG_TP4056_STATISTICS
    /* Update statistics */
    data->state_change_count++;
    uint64_t now = k_uptime_get();
    if (data->last_state_change_time > 0) {
        if (data->state == TP4056_CHARGING) {
            data->total_charging_time += (now - data->last_state_change_time);
        }
    }
    data->last_state_change_time = now;
#endif
    
    data->state = new_state;
    
    /* Notify callback if registered */
    if (data->callback != NULL) {
        struct tp4056_event event = {
            .type = TP4056_EVT_CHARGING_STATE_CHANGED,
            .data.charging_state = new_state,
        };
        
        if (from_isr) {
            /* Defer to work queue if called from ISR */
            data->pending_state = new_state;
        } else {
            data->callback(dev, &event, data->callback_user_data);
        }
    }
    
    return 0;
}

/* Debounce work handler */
static void tp4056_debounce_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tp4056_data *data = CONTAINER_OF(dwork, struct tp4056_data, debounce_work);
    const struct device *dev = data->dev;
    
    tp4056_update_state(dev, false);
}

/* GPIO interrupt callback */
static void tp4056_gpio_callback(const struct device *gpiob,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    struct tp4056_data *data = CONTAINER_OF(cb, struct tp4056_data, chrg_cb);
    const struct device *dev = data->dev;
    const struct tp4056_config *config = dev->config;
    
    /* Schedule debounce work */
    k_work_reschedule(&data->debounce_work,
                      K_MSEC(config->debounce_ms));
}

/* Polling work handler */
static void tp4056_poll_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tp4056_data *data = CONTAINER_OF(dwork, struct tp4056_data, poll_work);
    const struct device *dev = data->dev;
    const struct tp4056_config *config = dev->config;
    
    if (data->pm_suspended) {
        return;
    }
    
    tp4056_update_state(dev, false);
    
    /* Reschedule polling */
    k_work_reschedule(&data->poll_work,
                      K_MSEC(config->poll_interval_ms));
}

/* Initialize GPIO and interrupts */
static int tp4056_init_gpio(const struct device *dev)
{
    const struct tp4056_config *config = dev->config;
    struct tp4056_data *data = dev->data;
    int ret;
    
    /* Check if GPIO device is ready */
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    /* Configure GPIO as input with pull-up */
    ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO: %d", ret);
        return ret;
    }
    
#if !CONFIG_TP4056_POLLING_MODE
    /* Set up interrupt if not in polling mode */
    gpio_init_callback(&data->chrg_cb, tp4056_gpio_callback,
                       BIT(config->chrg_gpio.pin));
    
    ret = gpio_add_callback(config->chrg_gpio.port, &data->chrg_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    /* Enable interrupt on both edges */
    ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio,
                                          GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
#endif
    
    return 0;
}

/* Power management action */
static int tp4056_pm_action(const struct device *dev, enum pm_device_action action)
{
    struct tp4056_data *data = dev->data;
    const struct tp4056_config *config = dev->config;
    int ret = 0;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_DBG("Suspending TP4056 monitoring");
        data->pm_suspended = true;
        
#if !CONFIG_TP4056_POLLING_MODE
        /* Disable interrupts */
        gpio_pin_interrupt_configure_dt(&config->chrg_gpio, GPIO_INT_DISABLE);
#else
        /* Cancel polling work */
        k_work_cancel_delayable(&data->poll_work);
#endif
        break;
        
    case PM_DEVICE_ACTION_RESUME:
        LOG_DBG("Resuming TP4056 monitoring");
        data->pm_suspended = false;
        
        /* Update state immediately */
        tp4056_update_state(dev, false);
        
#if !CONFIG_TP4056_POLLING_MODE
        /* Re-enable interrupts */
        gpio_pin_interrupt_configure_dt(&config->chrg_gpio,
                                        GPIO_INT_EDGE_BOTH);
#else
        /* Restart polling */
        k_work_reschedule(&data->poll_work,
                          K_MSEC(config->poll_interval_ms));
#endif
        break;
        
    default:
        ret = -ENOTSUP;
        break;
    }
    
    return ret;
}

/* Initialize device */
static int tp4056_init(const struct device *dev)
{
    struct tp4056_data *data = dev->data;
    const struct tp4056_config *config = dev->config;
    int ret;
    
    data->dev = dev;
    data->state = TP4056_STATE_UNKNOWN;
    data->initialized = false;
    data->pm_suspended = false;
    
    /* Initialize works */
    k_work_init_delayable(&data->poll_work, tp4056_poll_work_handler);
    k_work_init_delayable(&data->debounce_work, tp4056_debounce_work_handler);
    
    /* Initialize GPIO */
    ret = tp4056_init_gpio(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize GPIO: %d", ret);
        return ret;
    }
    
    /* Read initial state */
    ret = tp4056_update_state(dev, false);
    if (ret < 0) {
        LOG_ERR("Failed to read initial state: %d", ret);
        return ret;
    }
    
    /* Start monitoring */
#if CONFIG_TP4056_POLLING_MODE
    k_work_reschedule(&data->poll_work,
                      K_MSEC(config->poll_interval_ms));
#endif
    
    data->initialized = true;
    
    LOG_INF("TP4056 initialized on GPIO %s, initial state: %s",
            config->chrg_gpio.port->name,
            data->state == TP4056_CHARGING ? "CHARGING" : "NOT_CHARGING");
    
    return 0;
}

/* API: Get charging state */
int tp4056_get_charging_state(const struct device *dev,
                             enum tp4056_charging_state *state)
{
    struct tp4056_data *data = dev->data;
    
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    if (state == NULL) {
        return -EINVAL;
    }
    
    *state = data->state;
    return 0;
}

/* API: Register callback */
int tp4056_register_callback(const struct device *dev,
                            tp4056_callback_t callback,
                            void *user_data)
{
    struct tp4056_data *data = dev->data;
    
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    if (callback == NULL) {
        return -EINVAL;
    }
    
    if (data->callback != NULL) {
        return -EALREADY; /* Already registered */
    }
    
    data->callback = callback;
    data->callback_user_data = user_data;
    
    return 0;
}

/* API: Unregister callback */
int tp4056_unregister_callback(const struct device *dev,
                              tp4056_callback_t callback)
{
    struct tp4056_data *data = dev->data;
    
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    if (data->callback != callback) {
        return -EINVAL; /* Not the registered callback */
    }
    
    data->callback = NULL;
    data->callback_user_data = NULL;
    
    return 0;
}

/* Device definitions */
#define TP4056_INIT(n) \
    static struct tp4056_data tp4056_data_##n; \
    static const struct tp4056_config tp4056_config_##n = { \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(n, chrg_gpios), \
        .poll_interval_ms = DT_INST_PROP_OR(n, poll_interval_ms, 1000), \
        .debounce_ms = DT_INST_PROP_OR(n, debounce_ms, 50), \
        .active_low = DT_INST_PROP(n, chrg_gpios) & GPIO_ACTIVE_LOW, \
    }; \
    PM_DEVICE_DT_INST_DEFINE(n, tp4056_pm_action); \
    DEVICE_DT_INST_DEFINE(n, tp4056_init, PM_DEVICE_DT_INST_GET(n), \
                         &tp4056_data_##n, &tp4056_config_##n, \
                         POST_KERNEL, CONFIG_TP4056_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(TP4056_INIT)