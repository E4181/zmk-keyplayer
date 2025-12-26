/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/pm/device.h>
#include <zephyr/init.h>

#include <zmk/charger_monitor.h>

LOG_MODULE_REGISTER(charger_monitor, CONFIG_ZMK_CHARGER_MONITOR_LOG_LEVEL);

/* Internal flags */
#define INTERRUPT_TRIGGERED_BIT 0
#define MONITOR_ENABLED_BIT 1
#define INITIALIZED_BIT 2

/* Maximum number of callbacks to limit memory usage */
#define MAX_CALLBACKS 2

/**
 * @brief Charger monitor device data structure
 */
struct zmk_charger_monitor_data {
    /* GPIO configuration */
    const struct gpio_dt_spec chrg_gpio;
    
    /* State management */
    atomic_t flags;
    enum zmk_charger_state current_state;
    enum zmk_charger_state last_state;
    int64_t last_state_change_time;
    int64_t last_interrupt_time;
    
    /* Callback management */
    struct {
        zmk_charger_state_changed_cb_t callback;
        void *user_data;
    } callbacks[MAX_CALLBACKS];
    uint8_t callback_count;
    
    /* Work items */
    struct k_work interrupt_work;
    struct k_work_delayable debounce_work;
    struct k_work_delayable idle_poll_work;
    struct k_work_delayable health_check_work;
    
    /* GPIO callback */
    struct gpio_callback gpio_callback;
    
    /* Mutex for thread safety */
    struct k_mutex mutex;
    
    /* Device pointer */
    const struct device *dev;
};

/**
 * @brief Read current charger state from GPIO pin
 * @param data Charger monitor data
 * @return Current charger state
 */
static inline enum zmk_charger_state read_charger_state(struct zmk_charger_monitor_data *data)
{
    int ret = gpio_pin_get_dt(&data->chrg_gpio);
    
    if (ret < 0) {
        LOG_DBG("Failed to read CHRG pin: %d", ret);
        return ZMK_CHARGER_STATE_UNKNOWN;
    }
    
    /* TP4056 CHRG pin is active-low: 0 = charging, 1 = not charging */
    return (ret == 0) ? ZMK_CHARGER_STATE_CHARGING : ZMK_CHARGER_STATE_NOT_CHARGING;
}

/**
 * @brief Process state change from interrupt or polling
 * @param work Work item
 */
static void process_state_change(struct k_work *work)
{
    struct zmk_charger_monitor_data *data = 
        CONTAINER_OF(work, struct zmk_charger_monitor_data, interrupt_work);
    
    /* Check if monitor is enabled */
    if (!atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT)) {
        return;
    }
    
    /* Try to lock mutex with timeout to avoid blocking */
    if (k_mutex_lock(&data->mutex, K_MSEC(10)) != 0) {
        LOG_DBG("Failed to acquire mutex, skipping state check");
        atomic_clear_bit(&data->flags, INTERRUPT_TRIGGERED_BIT);
        return;
    }
    
    /* Read current state */
    enum zmk_charger_state new_state = read_charger_state(data);
    
    /* Update state if changed */
    if (new_state != data->current_state && new_state != ZMK_CHARGER_STATE_UNKNOWN) {
        data->last_state = data->current_state;
        data->current_state = new_state;
        data->last_state_change_time = k_uptime_get();
        
        /* Log state change */
        LOG_INF("Charger state: %s -> %s",
                data->last_state == ZMK_CHARGER_STATE_CHARGING ? "CHARGING" : "NOT_CHARGING",
                data->current_state == ZMK_CHARGER_STATE_CHARGING ? "CHARGING" : "NOT_CHARGING");
        
        /* Notify all registered callbacks */
        for (int i = 0; i < data->callback_count; i++) {
            if (data->callbacks[i].callback != NULL) {
                data->callbacks[i].callback(data->current_state, data->callbacks[i].user_data);
            }
        }
    }
    
    k_mutex_unlock(&data->mutex);
    atomic_clear_bit(&data->flags, INTERRUPT_TRIGGERED_BIT);
}

/**
 * @brief Optimized GPIO interrupt callback
 * @note Marked with optimize for size to minimize stack usage
 */
__attribute__((optimize("-Os")))
static void gpio_interrupt_callback(const struct device *dev,
                                    struct gpio_callback *cb,
                                    uint32_t pins)
{
    struct zmk_charger_monitor_data *data = 
        CONTAINER_OF(cb, struct zmk_charger_monitor_data, gpio_callback);
    
    /* Check if monitor is enabled */
    if (!atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT)) {
        return;
    }
    
    /* Record interrupt time */
    data->last_interrupt_time = k_uptime_get();
    
    /* Set interrupt flag and submit work */
    atomic_set_bit(&data->flags, INTERRUPT_TRIGGERED_BIT);
    k_work_submit(&data->interrupt_work);
}

/**
 * @brief Idle polling work handler
 * @param work Delayed work item
 */
static void idle_poll_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zmk_charger_monitor_data *data = 
        CONTAINER_OF(dwork, struct zmk_charger_monitor_data, idle_poll_work);
    
    /* Check if monitor is enabled */
    if (!atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT)) {
        return;
    }
    
    /* Process state change */
    process_state_change(&data->interrupt_work);
    
    /* Reschedule if still in polling mode */
    int64_t now = k_uptime_get();
    int64_t time_since_change = now - data->last_state_change_time;
    
    if (time_since_change > CONFIG_ZMK_CHARGER_MONITOR_STATE_CHANGE_TIMEOUT_MS) {
        /* Still in polling mode, reschedule */
        k_work_reschedule(dwork, 
                         K_MSEC(CONFIG_ZMK_CHARGER_MONITOR_POLLING_INTERVAL_IDLE_MS));
        LOG_DBG("Continuing polling mode");
    } else {
        /* Recent state change, switch back to interrupt mode */
        int ret = gpio_pin_interrupt_configure_dt(&data->chrg_gpio, GPIO_INT_EDGE_BOTH);
        if (ret == 0) {
            LOG_DBG("Switched back to interrupt mode");
        }
    }
}

/**
 * @brief Health check work handler
 * @param work Delayed work item
 */
static void health_check_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zmk_charger_monitor_data *data = 
        CONTAINER_OF(dwork, struct zmk_charger_monitor_data, health_check_work);
    
    /* Check if GPIO device is still ready */
    if (!device_is_ready(data->chrg_gpio.port)) {
        LOG_WRN("CHRG GPIO device lost, disabling monitor");
        zmk_charger_monitor_disable(data->dev);
        return;
    }
    
    /* Reschedule health check */
    k_work_reschedule(dwork, K_MINUTES(5));
}

/**
 * @brief Enable charger monitor
 * @param data Charger monitor data
 * @return 0 on success, negative error code on failure
 */
static int enable_charger_monitor(struct zmk_charger_monitor_data *data)
{
    int ret;
    
    if (atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT)) {
        return 0; /* Already enabled */
    }
    
    /* Configure interrupt */
    ret = gpio_pin_interrupt_configure_dt(&data->chrg_gpio, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    atomic_set_bit(&data->flags, MONITOR_ENABLED_BIT);
    
    /* Schedule health check */
    k_work_reschedule(&data->health_check_work, K_MINUTES(5));
    
    LOG_DBG("Charger monitor enabled");
    
    return 0;
}

/**
 * @brief Disable charger monitor
 * @param data Charger monitor data
 */
static void disable_charger_monitor(struct zmk_charger_monitor_data *data)
{
    if (!atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT)) {
        return; /* Already disabled */
    }
    
    /* Disable interrupt */
    gpio_pin_interrupt_configure_dt(&data->chrg_gpio, GPIO_INT_DISABLE);
    
    /* Cancel all scheduled work */
    k_work_cancel(&data->interrupt_work);
    k_work_cancel_delayable(&data->idle_poll_work);
    k_work_cancel_delayable(&data->health_check_work);
    
    atomic_clear_bit(&data->flags, MONITOR_ENABLED_BIT);
    
    LOG_DBG("Charger monitor disabled");
}

/* API Implementation */

static enum zmk_charger_state get_state(const struct device *dev)
{
    struct zmk_charger_monitor_data *data = dev->data;
    
    if (!atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT)) {
        return ZMK_CHARGER_STATE_DISABLED;
    }
    
    k_mutex_lock(&data->mutex, K_FOREVER);
    enum zmk_charger_state state = data->current_state;
    k_mutex_unlock(&data->mutex);
    
    return state;
}

static int register_callback(const struct device *dev,
                             zmk_charger_state_changed_cb_t callback,
                             void *user_data)
{
    struct zmk_charger_monitor_data *data = dev->data;
    int ret = -ENOMEM;
    
    if (callback == NULL) {
        return -EINVAL;
    }
    
    k_mutex_lock(&data->mutex, K_FOREVER);
    
    /* Check for available slot */
    if (data->callback_count >= MAX_CALLBACKS) {
        ret = -ENOSPC;
        goto unlock;
    }
    
    /* Check if already registered */
    for (int i = 0; i < data->callback_count; i++) {
        if (data->callbacks[i].callback == callback && 
            data->callbacks[i].user_data == user_data) {
            ret = -EALREADY;
            goto unlock;
        }
    }
    
    /* Register callback */
    data->callbacks[data->callback_count].callback = callback;
    data->callbacks[data->callback_count].user_data = user_data;
    data->callback_count++;
    
    ret = 0;
    
unlock:
    k_mutex_unlock(&data->mutex);
    return ret;
}

static int unregister_callback(const struct device *dev,
                               zmk_charger_state_changed_cb_t callback)
{
    struct zmk_charger_monitor_data *data = dev->data;
    int ret = -ENOENT;
    
    k_mutex_lock(&data->mutex, K_FOREVER);
    
    for (int i = 0; i < data->callback_count; i++) {
        if (data->callbacks[i].callback == callback) {
            /* Shift remaining callbacks */
            for (int j = i; j < data->callback_count - 1; j++) {
                data->callbacks[j] = data->callbacks[j + 1];
            }
            data->callback_count--;
            ret = 0;
            break;
        }
    }
    
    k_mutex_unlock(&data->mutex);
    return ret;
}

static int enable(const struct device *dev)
{
    struct zmk_charger_monitor_data *data = dev->data;
    return enable_charger_monitor(data);
}

static int disable(const struct device *dev)
{
    struct zmk_charger_monitor_data *data = dev->data;
    disable_charger_monitor(data);
    return 0;
}

static bool is_enabled(const struct device *dev)
{
    struct zmk_charger_monitor_data *data = dev->data;
    return atomic_test_bit(&data->flags, MONITOR_ENABLED_BIT);
}

/**
 * @brief Charger monitor API structure
 */
static const struct zmk_charger_monitor_api charger_monitor_api = {
    .get_state = get_state,
    .register_callback = register_callback,
    .unregister_callback = unregister_callback,
    .enable = enable,
    .disable = disable,
    .is_enabled = is_enabled,
};

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Power management action handler
 */
static int charger_monitor_pm_action(const struct device *dev,
                                     enum pm_device_action action)
{
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        return enable(dev);
    default:
        return -ENOTSUP;
    }
}
#endif /* CONFIG_PM_DEVICE */

/**
 * @brief Delayed initialization function
 */
static void delayed_init_work_handler(struct k_work *work)
{
    struct zmk_charger_monitor_data *data = 
        CONTAINER_OF(work, struct zmk_charger_monitor_data, interrupt_work);
    const struct device *dev = data->dev;
    
    LOG_DBG("Starting delayed charger monitor initialization");
    
    /* Wait a bit to ensure keyboard matrix is initialized */
    k_sleep(K_MSEC(100));
    
    /* Check GPIO device */
    if (!device_is_ready(data->chrg_gpio.port)) {
        LOG_WRN("CHRG GPIO not ready, charger monitor will remain disabled");
        data->current_state = ZMK_CHARGER_STATE_DISABLED;
        atomic_set_bit(&data->flags, INITIALIZED_BIT);
        return;
    }
    
    /* Configure GPIO pin as input */
    int ret = gpio_pin_configure_dt(&data->chrg_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_WRN("Failed to configure CHRG pin (%d), charger monitor disabled", ret);
        data->current_state = ZMK_CHARGER_STATE_DISABLED;
        atomic_set_bit(&data->flags, INITIALIZED_BIT);
        return;
    }
    
    LOG_INF("CHRG pin: %s pin %d", 
            data->chrg_gpio.port->name, data->chrg_gpio.pin);
    
    /* Initialize work items */
    k_work_init(&data->interrupt_work, process_state_change);
    k_work_init_delayable(&data->idle_poll_work, idle_poll_work_handler);
    k_work_init_delayable(&data->health_check_work, health_check_work_handler);
    
    /* Initialize GPIO callback */
    gpio_init_callback(&data->gpio_callback, gpio_interrupt_callback,
                       BIT(data->chrg_gpio.pin));
    
    /* Add GPIO callback */
    ret = gpio_add_callback_dt(&data->chrg_gpio, &data->gpio_callback);
    if (ret < 0) {
        LOG_WRN("Failed to add GPIO callback (%d), charger monitor disabled", ret);
        data->current_state = ZMK_CHARGER_STATE_DISABLED;
        atomic_set_bit(&data->flags, INITIALIZED_BIT);
        return;
    }
    
    /* Read initial state */
    data->current_state = read_charger_state(data);
    data->last_state = data->current_state;
    data->last_state_change_time = k_uptime_get();
    data->last_interrupt_time = k_uptime_get();
    
    /* Enable monitor */
    ret = enable_charger_monitor(data);
    if (ret < 0) {
        LOG_WRN("Failed to enable charger monitor on init");
        data->current_state = ZMK_CHARGER_STATE_DISABLED;
    }
    
    atomic_set_bit(&data->flags, INITIALIZED_BIT);
    
    LOG_INF("Charger monitor ready (state: %s)",
            data->current_state == ZMK_CHARGER_STATE_CHARGING ? "CHARGING" :
            data->current_state == ZMK_CHARGER_STATE_NOT_CHARGING ? "NOT_CHARGING" : "UNKNOWN");
}

/**
 * @brief Main initialization function
 */
static int zmk_charger_monitor_init(const struct device *dev)
{
    struct zmk_charger_monitor_data *data = dev->data;
    
    LOG_DBG("Initializing charger monitor");
    
    /* Store device pointer */
    data->dev = dev;
    
    /* Initialize mutex */
    k_mutex_init(&data->mutex);
    
#ifdef CONFIG_ZMK_CHARGER_MONITOR_DELAYED_INIT
    /* Schedule delayed initialization */
    k_work_init(&data->interrupt_work, delayed_init_work_handler);
    k_work_submit(&data->interrupt_work);
#else
    /* Immediate initialization */
    delayed_init_work_handler(&data->interrupt_work);
#endif
    
    return 0;
}

/**
 * @brief Define charger monitor device instance
 */
#define ZMK_CHARGER_MONITOR_DEFINE(inst)                                         \
    static struct zmk_charger_monitor_data charger_monitor_data_##inst = {       \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(inst, chrg_gpios),                   \
        .current_state = ZMK_CHARGER_STATE_UNKNOWN,                             \
        .callback_count = 0,                                                    \
    };                                                                          \
    PM_DEVICE_DT_INST_DEFINE(inst, charger_monitor_pm_action);                  \
    DEVICE_DT_INST_DEFINE(inst,                                                 \
                          zmk_charger_monitor_init,                             \
                          PM_DEVICE_DT_INST_GET(inst),                          \
                          &charger_monitor_data_##inst,                         \
                          NULL,                                                 \
                          POST_KERNEL,                                          \
                          CONFIG_ZMK_CHARGER_MONITOR_INIT_PRIORITY,             \
                          &charger_monitor_api);

/* Instantiate all defined charger monitor devices */
DT_INST_FOREACH_STATUS_OKAY(ZMK_CHARGER_MONITOR_DEFINE)