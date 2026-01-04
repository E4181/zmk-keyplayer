/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT charging_status

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "charging_status.h"

LOG_MODULE_REGISTER(charging_status, CONFIG_SENSOR_LOG_LEVEL);

/* 设备配置结构 */
struct charging_status_config {
    struct gpio_dt_spec status_gpio;
};

/* 设备数据结构 */
struct charging_status_data {
#ifdef CONFIG_CHARGING_STATUS_TRIGGER
    struct gpio_callback gpio_cb;
    struct sensor_trigger trig;
    sensor_trigger_handler_t handler;
#endif
    int last_state;
    void *user_data;
};

#ifdef CONFIG_CHARGING_STATUS_TRIGGER
static void charging_status_gpio_callback(const struct device *dev,
                                         struct gpio_callback *cb,
                                         uint32_t pins)
{
    struct charging_status_data *data = CONTAINER_OF(cb, struct charging_status_data, gpio_cb);
    const struct device *sensor_dev = dev->parent;
    
    if (data->handler) {
        data->trig.type = SENSOR_TRIG_THRESHOLD;
        data->handler(sensor_dev, &data->trig);
    }
}
#endif

static int charging_status_sample_fetch(const struct device *dev,
                                       enum sensor_channel chan)
{
    struct charging_status_data *data = dev->data;
    const struct charging_status_config *config = dev->config;
    
    /* 只支持状态通道 */
    if (chan != SENSOR_CHAN_ALL && chan != CHARGING_STATUS_CHAN_CHARGING) {
        return -ENOTSUP;
    }
    
    /* 读取 GPIO 状态 */
    int state = gpio_pin_get_dt(&config->status_gpio);
    
    /* 注意：CHRG 是低电平有效 */
    /* 如果 GPIO 配置为 ACTIVE_LOW，则硬件已经反转，否则需要手动反转 */
    if (!(config->status_gpio.dt_flags & GPIO_ACTIVE_LOW)) {
        state = !state;
    }
    
    data->last_state = state;
    
    return 0;
}

static int charging_status_channel_get(const struct device *dev,
                                      enum sensor_channel chan,
                                      struct sensor_value *val)
{
    struct charging_status_data *data = dev->data;
    
    switch (chan) {
    case CHARGING_STATUS_CHAN_CHARGING:
        val->val1 = data->last_state;
        val->val2 = 0;
        break;
        
    default:
        return -ENOTSUP;
    }
    
    return 0;
}

#ifdef CONFIG_CHARGING_STATUS_TRIGGER
static int charging_status_trigger_set(const struct device *dev,
                                      const struct sensor_trigger *trig,
                                      sensor_trigger_handler_t handler)
{
    struct charging_status_data *data = dev->data;
    const struct charging_status_config *config = dev->config;
    
    if (trig->type != SENSOR_TRIG_THRESHOLD) {
        return -ENOTSUP;
    }
    
    data->handler = handler;
    
    if (handler) {
        /* 启用 GPIO 中断 */
        return gpio_pin_interrupt_configure_dt(&config->status_gpio,
                                              GPIO_INT_EDGE_BOTH);
    } else {
        /* 禁用 GPIO 中断 */
        return gpio_pin_interrupt_configure_dt(&config->status_gpio,
                                              GPIO_INT_DISABLE);
    }
}
#endif

/* 传感器驱动 API */
static const struct sensor_driver_api charging_status_api = {
    .sample_fetch = charging_status_sample_fetch,
    .channel_get = charging_status_channel_get,
#ifdef CONFIG_CHARGING_STATUS_TRIGGER
    .trigger_set = charging_status_trigger_set,
#endif
};

/* 公共 API 实现 */
int charging_status_get_state(const struct device *dev)
{
    struct charging_status_data *data;
    
    if (!dev) {
        return -EINVAL;
    }
    
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    data = dev->data;
    return data->last_state;
}

int charging_status_register_callback(const struct device *dev,
                                     sensor_trigger_handler_t callback,
                                     void *user_data)
{
#ifdef CONFIG_CHARGING_STATUS_TRIGGER
    struct charging_status_data *data = dev->data;
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_THRESHOLD,
        .chan = CHARGING_STATUS_CHAN_CHANGE_EVENT,
    };
    
    data->user_data = user_data;
    return sensor_trigger_set(dev, &trig, callback);
#else
    return -ENOTSUP;
#endif
}

/* 设备初始化函数 */
static int charging_status_init(const struct device *dev)
{
    struct charging_status_data *data = dev->data;
    const struct charging_status_config *config = dev->config;
    int ret;
    
    /* 检查 GPIO 设备是否就绪 */
    if (!device_is_ready(config->status_gpio.port)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    /* 配置 GPIO 为输入 */
    ret = gpio_pin_configure_dt(&config->status_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO: %d", ret);
        return ret;
    }
    
    /* 初始化数据结构 */
    data->last_state = -1;
    
#ifdef CONFIG_CHARGING_STATUS_TRIGGER
    /* 设置 GPIO 回调 */
    gpio_init_callback(&data->gpio_cb, charging_status_gpio_callback,
                       BIT(config->status_gpio.pin));
    
    ret = gpio_add_callback(config->status_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    /* 初始化触发器 */
    data->trig.type = SENSOR_TRIG_THRESHOLD;
    data->trig.chan = CHARGING_STATUS_CHAN_CHANGE_EVENT;
    data->handler = NULL;
#endif
    
    /* 获取初始状态 */
    charging_status_sample_fetch(dev, SENSOR_CHAN_ALL);
    
    LOG_INF("Charging status driver initialized");
    
    return 0;
}

/* 设备定义宏 */
#define CHARGING_STATUS_INIT(n) \
    static struct charging_status_data charging_status_data_##n; \
    \
    static const struct charging_status_config charging_status_config_##n = { \
        .status_gpio = GPIO_DT_SPEC_INST_GET(n, status_gpios), \
    }; \
    \
    DEVICE_DT_INST_DEFINE(n, \
                         charging_status_init, \
                         NULL, \
                         &charging_status_data_##n, \
                         &charging_status_config_##n, \
                         POST_KERNEL, \
                         CONFIG_SENSOR_INIT_PRIORITY, \
                         &charging_status_api);

/* 实例化所有匹配的设备 */
DT_INST_FOREACH_STATUS_OKAY(CHARGING_STATUS_INIT)