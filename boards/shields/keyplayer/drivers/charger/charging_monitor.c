/*
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* 设备树匹配 */
#define CHARGING_MONITOR_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zmk_charging_monitor)

struct charging_monitor_config {
    struct gpio_dt_spec chrg_gpio;
    struct gpio_dt_spec led_gpio;
    const char *label;
};

/* 设备数据 */
struct charging_monitor_data {
    struct gpio_callback chrg_cb_data;
    struct k_work_delayable monitor_work;
    bool is_charging;
};

/* 工作队列函数 */
static void charging_monitor_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data = CONTAINER_OF(dwork, struct charging_monitor_data, monitor_work);
    const struct device *dev = data->dev;
    const struct charging_monitor_config *config = dev->config;
    
    int chrg_state = gpio_pin_get_dt(&config->chrg_gpio);
    
    if (chrg_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", chrg_state);
        return;
    }
    
    bool new_charging_state = (chrg_state == 0); // CHRG低电平表示正在充电
    
    if (data->is_charging != new_charging_state) {
        data->is_charging = new_charging_state;
        
        /* 控制LED */
        int ret = gpio_pin_set_dt(&config->led_gpio, new_charging_state ? 1 : 0);
        if (ret < 0) {
            LOG_ERR("Failed to set LED: %d", ret);
        }
        
        LOG_INF("Charging state changed: %s", new_charging_state ? "CHARGING" : "NOT CHARGING");
    }
    
    /* 继续监控（每500ms检查一次） */
    k_work_reschedule(&data->monitor_work, K_MSEC(500));
}

/* 充电状态中断回调 */
static void charging_state_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct charging_monitor_data *data = CONTAINER_OF(cb, struct charging_monitor_data, chrg_cb_data);
    
    /* 立即触发工作队列处理 */
    k_work_reschedule(&data->monitor_work, K_NO_WAIT);
}

/* 初始化函数 */
static int charging_monitor_init(const struct device *dev) {
    const struct charging_monitor_config *config = dev->config;
    struct charging_monitor_data *data = dev->data;
    
    data->dev = dev;
    data->is_charging = false;
    
    LOG_DBG("Initializing charging monitor on %s", config->label);
    
    /* 初始化CHRG引脚为输入 */
    if (!gpio_is_ready_dt(&config->chrg_gpio)) {
        LOG_ERR("CHRG GPIO not ready");
        return -ENODEV;
    }
    
    int ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG pin: %d", ret);
        return ret;
    }
    
    /* 初始化LED引脚为输出 */
    if (!gpio_is_ready_dt(&config->led_gpio)) {
        LOG_ERR("LED GPIO not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&config->led_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED pin: %d", ret);
        return ret;
    }
    
    /* 设置中断回调 */
    ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    gpio_init_callback(&data->chrg_cb_data, charging_state_changed, BIT(config->chrg_gpio.pin));
    gpio_add_callback(config->chrg_gpio.port, &data->chrg_cb_data);
    
    /* 初始化工作队列 */
    k_work_init_delayable(&data->monitor_work, charging_monitor_work_handler);
    
    /* 启动第一次监控 */
    k_work_reschedule(&data->monitor_work, K_NO_WAIT);
    
    LOG_INF("Charging monitor initialized");
    return 0;
}

/* 设备定义 */
static struct charging_monitor_data charging_monitor_data;
static const struct charging_monitor_config charging_monitor_config = {
    .chrg_gpio = GPIO_DT_SPEC_GET(CHARGING_MONITOR_NODE, chrg_gpio),
    .led_gpio = GPIO_DT_SPEC_GET(CHARGING_MONITOR_NODE, led_gpio),
    .label = DT_LABEL(CHARGING_MONITOR_NODE),
};

DEVICE_DT_DEFINE(CHARGING_MONITOR_NODE,
                 charging_monitor_init,
                 NULL,
                 &charging_monitor_data,
                 &charging_monitor_config,
                 POST_KERNEL,
                 CONFIG_APPLICATION_INIT_PRIORITY,
                 NULL);