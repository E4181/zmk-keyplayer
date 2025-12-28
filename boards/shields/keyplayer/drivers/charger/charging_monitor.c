/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_charging_monitor

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct charging_monitor_config {
    struct gpio_dt_spec chrg_gpio;   // TP4056 CHRG引脚
    struct gpio_dt_spec led_gpio;    // 指示灯LED引脚
    int poll_interval_ms;            // 轮询间隔
};

struct charging_monitor_data {
    const struct device *dev;
    struct k_work_delayable work;
    bool is_charging;
    struct sensor_trigger trigger;
    sensor_trigger_handler_t trigger_handler;
};

static int charging_monitor_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    // 此函数用于获取传感器数据
    // 对于充电监控，我们不需要复杂的采样，状态变化时直接更新
    return 0;
}

static int charging_monitor_channel_get(const struct device *dev,
                                       enum sensor_channel chan,
                                       struct sensor_value *val) {
    struct charging_monitor_data *data = dev->data;
    
    if (chan != SENSOR_CHAN_GAUGE_CHARGING_STATUS) {
        return -ENOTSUP;
    }
    
    // 返回充电状态：1=充电中，0=未充电
    val->val1 = data->is_charging ? 1 : 0;
    val->val2 = 0;
    
    return 0;
}

static int charging_monitor_trigger_set(const struct device *dev,
                                       const struct sensor_trigger *trig,
                                       sensor_trigger_handler_t handler) {
    struct charging_monitor_data *data = dev->data;
    
    if (trig->type != SENSOR_TRIG_CHARGING_STATUS_CHANGE) {
        return -ENOTSUP;
    }
    
    data->trigger = *trig;
    data->trigger_handler = handler;
    
    return 0;
}

static void update_led_state(const struct device *dev, bool charging) {
    const struct charging_monitor_config *config = dev->config;
    
    if (!device_is_ready(config->led_gpio.port)) {
        LOG_ERR("LED GPIO device not ready");
        return;
    }
    
    int ret = gpio_pin_set_dt(&config->led_gpio, charging ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to set LED state: %d", ret);
    }
}

static void check_charging_status(const struct device *dev) {
    const struct charging_monitor_config *config = dev->config;
    struct charging_monitor_data *data = dev->data;
    
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device not ready");
        return;
    }
    
    // 读取CHRG引脚状态
    int state = gpio_pin_get_dt(&config->chrg_gpio);
    if (state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", state);
        return;
    }
    
    // TP4056: 充电时CHRG为低电平，不充电时为高电平/高阻态
    // 假设电路中有上拉电阻，所以高电平=不充电，低电平=充电
    bool new_charging_state = (state == 0); // 低电平表示充电中
    
    if (data->is_charging != new_charging_state) {
        data->is_charging = new_charging_state;
        
        LOG_INF("Charging status changed: %s", 
                new_charging_state ? "Charging" : "Not charging");
        
        // 更新LED状态
        update_led_state(dev, new_charging_state);
        
        // 触发事件（如果有注册的handler）
        if (data->trigger_handler) {
            sensor_trigger_handler_t handler = data->trigger_handler;
            handler(dev, &data->trigger);
        }
    }
}

static void charging_monitor_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = CONTAINER_OF(dwork, struct charging_monitor_data, work);
    
    check_charging_status(data->dev);
    
    // 重新调度工作
    const struct charging_monitor_config *config = data->dev->config;
    k_work_reschedule(&data->work, K_MSEC(config->poll_interval_ms));
}

static int charging_monitor_init(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    const struct charging_monitor_config *config = dev->config;
    
    data->dev = dev;
    data->is_charging = false;
    
    // 初始化CHRG引脚
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device not ready");
        return -ENODEV;
    }
    
    int ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG pin: %d", ret);
        return ret;
    }
    
    // 初始化LED引脚
    if (!device_is_ready(config->led_gpio.port)) {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&config->led_gpio, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED pin: %d", ret);
        return ret;
    }
    
    // 初始关闭LED
    gpio_pin_set_dt(&config->led_gpio, 0);
    
    // 初始化工作队列
    k_work_init_delayable(&data->work, charging_monitor_work_handler);
    
    // 立即检查一次状态
    check_charging_status(dev);
    
    // 开始定期检查
    k_work_reschedule(&data->work, K_MSEC(config->poll_interval_ms));
    
    LOG_INF("Charging monitor initialized");
    
    return 0;
}

static const struct sensor_driver_api charging_monitor_api = {
    .sample_fetch = charging_monitor_sample_fetch,
    .channel_get = charging_monitor_channel_get,
    .trigger_set = charging_monitor_trigger_set,
};

#define CHARGING_MONITOR_INIT(n) \
    static struct charging_monitor_data charging_monitor_data_##n; \
    static const struct charging_monitor_config charging_monitor_config_##n = { \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(n, chrg_gpios), \
        .led_gpio = GPIO_DT_SPEC_INST_GET(n, led_gpios), \
        .poll_interval_ms = DT_INST_PROP(n, poll_interval_ms), \
    }; \
    DEVICE_DT_INST_DEFINE(n, \
                         charging_monitor_init, \
                         NULL, \
                         &charging_monitor_data_##n, \
                         &charging_monitor_config_##n, \
                         POST_KERNEL, \
                         CONFIG_SENSOR_INIT_PRIORITY, \
                         &charging_monitor_api);

DT_INST_FOREACH_STATUS_OKAY(CHARGING_MONITOR_INIT)