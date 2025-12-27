/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/charger_tp4056.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* 设备树定义 - 使用DT_COMPAT方式 */
#define DT_DRV_COMPAT zmk_charger_tp4056

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* 静态变量 */
static enum charger_state current_state = CHARGER_STATE_UNKNOWN;

struct charger_tp4056_data {
    const struct device *dev;
    struct gpio_callback chrg_cb;
    struct k_work_delayable debounce_work;
    struct k_work status_work;
    struct k_work led_update_work;
    enum charger_state state;
};

struct charger_tp4056_config {
    struct gpio_dt_spec chrg_gpio;
    struct gpio_dt_spec led_gpio;
    uint16_t debounce_ms;
};

/* 事件实现 */
ZMK_EVENT_IMPL(zmk_charger_state_changed);

/* 私有函数声明 */
static void charger_status_work_handler(struct k_work *work);
static void charger_debounce_work_handler(struct k_work *work);
static void charger_gpio_callback(const struct device *dev,
                                  struct gpio_callback *cb,
                                  uint32_t pins);
static void led_update_work_handler(struct k_work *work);
static void update_led_state(const struct device *dev);

/* 状态工作处理器 */
static void charger_status_work_handler(struct k_work *work) {
    struct charger_tp4056_data *data = CONTAINER_OF(work, struct charger_tp4056_data, status_work);
    const struct device *dev = data->dev;
    const struct charger_tp4056_config *config = dev->config;
    
    int val = gpio_pin_get_dt(&config->chrg_gpio);
    
    if (val < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", val);
        return;
    }
    
    enum charger_state new_state = (val == 0) ? 
        CHARGER_STATE_CHARGING : CHARGER_STATE_NOT_CHARGING;
    
    if (new_state != data->state) {
        LOG_INF("Charger state changed: %s -> %s",
                data->state == CHARGER_STATE_CHARGING ? "CHARGING" : 
                data->state == CHARGER_STATE_NOT_CHARGING ? "NOT_CHARGING" : "UNKNOWN",
                new_state == CHARGER_STATE_CHARGING ? "CHARGING" : "NOT_CHARGING");
        
        data->state = new_state;
        current_state = new_state;
        
        // 发布充电状态变化事件
        raise_zmk_charger_state_changed((struct zmk_charger_state_changed){
            .state = new_state
        });
        
        // 更新LED状态
        k_work_submit(&data->led_update_work);
    }
}

/* 消抖工作处理器 */
static void charger_debounce_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charger_tp4056_data *data = CONTAINER_OF(dwork, struct charger_tp4056_data, debounce_work);
    
    k_work_submit(&data->status_work);
}

/* GPIO中断回调 */
static void charger_gpio_callback(const struct device *dev,
                                  struct gpio_callback *cb,
                                  uint32_t pins) {
    struct charger_tp4056_data *data = CONTAINER_OF(cb, struct charger_tp4056_data, chrg_cb);
    const struct charger_tp4056_config *config = dev->config;
    
    if (pins & BIT(config->chrg_gpio.pin)) {
        k_work_reschedule(&data->debounce_work, K_MSEC(config->debounce_ms));
    }
}

/* LED更新工作处理器 */
static void led_update_work_handler(struct k_work *work) {
    struct charger_tp4056_data *data = CONTAINER_OF(work, struct charger_tp4056_data, led_update_work);
    update_led_state(data->dev);
}

/* 更新LED状态 */
static void update_led_state(const struct device *dev) {
    const struct charger_tp4056_config *config = dev->config;
    struct charger_tp4056_data *data = dev->data;
    
    if (!device_is_ready(config->led_gpio.port)) {
        LOG_ERR("LED GPIO device not ready");
        return;
    }
    
    int ret;
    switch (data->state) {
        case CHARGER_STATE_CHARGING:
            ret = gpio_pin_set_dt(&config->led_gpio, 1);
            if (ret < 0) {
                LOG_ERR("Failed to turn on LED: %d", ret);
            }
            break;
            
        case CHARGER_STATE_NOT_CHARGING:
        case CHARGER_STATE_UNKNOWN:
            ret = gpio_pin_set_dt(&config->led_gpio, 0);
            if (ret < 0) {
                LOG_ERR("Failed to turn off LED: %d", ret);
            }
            break;
    }
}

/* 公共API函数 */
enum charger_state zmk_charger_get_state(void) {
    return current_state;
}

bool zmk_charger_is_charging(void) {
    return current_state == CHARGER_STATE_CHARGING;
}

/* 初始化函数 */
static int charger_tp4056_init(const struct device *dev) {
    struct charger_tp4056_data *data = dev->data;
    const struct charger_tp4056_config *config = dev->config;
    int ret;
    
    data->dev = dev;
    data->state = CHARGER_STATE_UNKNOWN;
    
    /* 初始化CHRG GPIO */
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    /* 初始化LED GPIO */
    if (!device_is_ready(config->led_gpio.port)) {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&config->led_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO: %d", ret);
        return ret;
    }
    
    /* 初始化GPIO中断 */
    ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    /* 初始化工作队列 */
    k_work_init(&data->status_work, charger_status_work_handler);
    k_work_init_delayable(&data->debounce_work, charger_debounce_work_handler);
    k_work_init(&data->led_update_work, led_update_work_handler);
    
    /* 设置GPIO回调 */
    gpio_init_callback(&data->chrg_cb, charger_gpio_callback, BIT(config->chrg_gpio.pin));
    gpio_add_callback(config->chrg_gpio.port, &data->chrg_cb);
    
    /* 读取初始状态 */
    k_work_submit(&data->status_work);
    
    LOG_INF("TP4056 charger driver initialized with LED on pin %d", config->led_gpio.pin);
    
    return 0;
}

/* 设备树实例化 */
#define CHARGER_TP4056_INIT(n) \
    static struct charger_tp4056_data charger_tp4056_data_##n = { \
        .state = CHARGER_STATE_UNKNOWN, \
    }; \
    \
    static const struct charger_tp4056_config charger_tp4056_config_##n = { \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(n, chrg_gpios), \
        .led_gpio = GPIO_DT_SPEC_INST_GET(n, led_gpios), \
        .debounce_ms = DT_INST_PROP(n, debounce_ms), \
    }; \
    \
    DEVICE_DT_INST_DEFINE(n, \
                         charger_tp4056_init, \
                         NULL, \
                         &charger_tp4056_data_##n, \
                         &charger_tp4056_config_##n, \
                         POST_KERNEL, \
                         CONFIG_APPLICATION_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(CHARGER_TP4056_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */