/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_tp4056_charger

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct tp4056_data {
    struct gpio_callback gpio_cb;
    struct sensor_value charging_state;
    struct k_work work;
    atomic_t debounce_flag;
    struct k_timer debounce_timer;
};

struct tp4056_config {
    struct gpio_dt_spec chrg_gpio;
    struct gpio_dt_spec led_gpio;
    uint32_t debounce_ms;
};

static void debounce_timer_handler(struct k_timer *timer)
{
    struct tp4056_data *data = CONTAINER_OF(timer, struct tp4056_data, debounce_timer);
    atomic_set(&data->debounce_flag, 0);
}

static void tp4056_work_handler(struct k_work *work)
{
    struct tp4056_data *data = CONTAINER_OF(work, struct tp4056_data, work);
    const struct device *dev = data->dev;
    const struct tp4056_config *cfg = dev->config;
    
    int val = gpio_pin_get_dt(&cfg->chrg_gpio);
    bool is_charging = (val == 0); // CHRG低电平表示充电
    
    // 更新传感器值
    data->charging_state.val1 = is_charging ? 1 : 0;
    data->charging_state.val2 = 0;
    
    // 控制LED：充电时点亮
    int ret = gpio_pin_set_dt(&cfg->led_gpio, is_charging ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to set LED: %d", ret);
    }
    
    LOG_DBG("TP4056: %s, LED: %s",
            is_charging ? "CHARGING" : "NOT CHARGING",
            is_charging ? "ON" : "OFF");
}

static void tp4056_gpio_callback(const struct device *dev,
                                 struct gpio_callback *cb,
                                 uint32_t pins)
{
    struct tp4056_data *data = CONTAINER_OF(cb, struct tp4056_data, gpio_cb);
    const struct tp4056_config *cfg = data->dev->config;
    
    // 防抖处理
    if (atomic_cas(&data->debounce_flag, 0, 1)) {
        k_timer_start(&data->debounce_timer, K_MSEC(cfg->debounce_ms), K_NO_WAIT);
        k_work_submit(&data->work);
    }
}

static int tp4056_sample_fetch(const struct device *dev,
                               enum sensor_channel chan)
{
    struct tp4056_data *data = dev->data;
    const struct tp4056_config *cfg = dev->config;
    
    if (chan != SENSOR_CHAN_CUSTOM_0 && chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }
    
    int val = gpio_pin_get_dt(&cfg->chrg_gpio);
    data->charging_state.val1 = (val == 0) ? 1 : 0;
    data->charging_state.val2 = 0;
    
    return 0;
}

static int tp4056_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct tp4056_data *data = dev->data;
    
    if (chan == SENSOR_CHAN_CUSTOM_0) {
        val->val1 = data->charging_state.val1;
        val->val2 = data->charging_state.val2;
        return 0;
    }
    
    return -ENOTSUP;
}

static const struct sensor_driver_api tp4056_api = {
    .sample_fetch = tp4056_sample_fetch,
    .channel_get = tp4056_channel_get,
};

static int tp4056_init(const struct device *dev)
{
    struct tp4056_data *data = dev->data;
    const struct tp4056_config *cfg = dev->config;
    
    data->dev = dev;
    atomic_set(&data->debounce_flag, 0);
    k_work_init(&data->work, tp4056_work_handler);
    k_timer_init(&data->debounce_timer, debounce_timer_handler, NULL);
    
    // 初始化CHRG GPIO
    if (!device_is_ready(cfg->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device not ready");
        return -ENODEV;
    }
    
    int ret = gpio_pin_configure_dt(&cfg->chrg_gpio, 
                                   GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    // 配置CHRG引脚中断（双边沿触发）
    ret = gpio_pin_interrupt_configure_dt(&cfg->chrg_gpio,
                                         GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    gpio_init_callback(&data->gpio_cb, tp4056_gpio_callback,
                      BIT(cfg->chrg_gpio.pin));
    gpio_add_callback(cfg->chrg_gpio.port, &data->gpio_cb);
    
    // 初始化LED GPIO
    if (!device_is_ready(cfg->led_gpio.port)) {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&cfg->led_gpio, 
                               GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO: %d", ret);
        return ret;
    }
    
    // 读取初始状态并设置LED
    tp4056_sample_fetch(dev, SENSOR_CHAN_CUSTOM_0);
    k_work_submit(&data->work);
    
    LOG_INF("TP4056 charger sensor initialized");
    return 0;
}

#define TP4056_DEFINE(index)                                         \
    static struct tp4056_data tp4056_data_##index;                   \
    static const struct tp4056_config tp4056_config_##index = {      \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(index, chrg_gpios),       \
        .led_gpio = GPIO_DT_SPEC_INST_GET(index, led_gpios),         \
        .debounce_ms = DT_INST_PROP_OR(index, debounce_ms, 50),      \
    };                                                               \
    DEVICE_DT_INST_DEFINE(index,                                     \
                          tp4056_init,                               \
                          NULL,                                      \
                          &tp4056_data_##index,                      \
                          &tp4056_config_##index,                    \
                          POST_KERNEL,                               \
                          CONFIG_SENSOR_INIT_PRIORITY,               \
                          &tp4056_api);

DT_INST_FOREACH_STATUS_OKAY(TP4056_DEFINE)