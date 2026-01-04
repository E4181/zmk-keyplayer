/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "charging_status.h"

LOG_MODULE_REGISTER(charging_status, CONFIG_CHARGING_STATUS_LOG_LEVEL);

/* 硬编码 GPIO 配置 - P1.09 (nRF52840) */
#define CHARGING_STATUS_PORT DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define CHARGING_STATUS_PIN 9
#define CHARGING_STATUS_FLAGS (GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW)

/* 回调函数结构 */
struct charging_status_callback {
    void (*func)(bool is_charging, void *user_data);
    void *user_data;
    sys_snode_t node;
};

/* 驱动数据结构 */
static struct {
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;
    struct k_mutex mutex;
    sys_slist_t callbacks;
    bool last_state;
    bool initialized;
} charging_status_data;

/* GPIO 中断回调函数 */
static void charging_status_gpio_callback(const struct device *dev,
                                         struct gpio_callback *cb,
                                         uint32_t pins)
{
    bool current_state;
    
    if (!charging_status_data.initialized) {
        return;
    }
    
    /* 获取当前状态 */
    int ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get GPIO state: %d", ret);
        return;
    }
    
    current_state = (ret == 0); /* 注意：ACTIVE_LOW，所以0表示正在充电 */
    
    /* 如果状态改变，通知所有回调 */
    if (current_state != charging_status_data.last_state) {
        charging_status_data.last_state = current_state;
        
        /* 调用所有注册的回调 */
        struct charging_status_callback *cb_node;
        SYS_SLIST_FOR_EACH_CONTAINER(&charging_status_data.callbacks, cb_node, node) {
            if (cb_node->func) {
                cb_node->func(current_state, cb_node->user_data);
            }
        }
        
        LOG_DBG("Charging state changed: %s", 
                current_state ? "Charging" : "Not charging");
    }
}

int charging_status_init(void)
{
    int ret;
    
    /* 检查是否已经初始化 */
    if (charging_status_data.initialized) {
        return 0;
    }
    
    /* 初始化数据结构 */
    k_mutex_init(&charging_status_data.mutex);
    sys_slist_init(&charging_status_data.callbacks);
    charging_status_data.last_state = false;
    charging_status_data.initialized = false;
    
    /* 获取 GPIO 设备 */
    charging_status_data.gpio_dev = CHARGING_STATUS_PORT;
    if (!device_is_ready(charging_status_data.gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    /* 配置 GPIO 引脚 */
    ret = gpio_pin_configure(charging_status_data.gpio_dev, CHARGING_STATUS_PIN,
                            CHARGING_STATUS_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO: %d", ret);
        return ret;
    }
    
    /* 初始化 GPIO 回调 */
    gpio_init_callback(&charging_status_data.gpio_cb,
                      charging_status_gpio_callback,
                      BIT(CHARGING_STATUS_PIN));
    
    /* 添加回调 */
    ret = gpio_add_callback(charging_status_data.gpio_dev, &charging_status_data.gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    /* 启用 GPIO 中断 */
    ret = gpio_pin_interrupt_configure(charging_status_data.gpio_dev,
                                      CHARGING_STATUS_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
        return ret;
    }
    
    /* 获取初始状态 */
    ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get initial GPIO state: %d", ret);
        return ret;
    }
    
    charging_status_data.last_state = (ret == 0); /* ACTIVE_LOW */
    charging_status_data.initialized = true;
    
    LOG_INF("Charging status driver initialized on P1.%02d, initial state: %s",
            CHARGING_STATUS_PIN,
            charging_status_data.last_state ? "Charging" : "Not charging");
    
    return 0;
}

bool charging_status_is_charging(void)
{
    int ret;
    
    if (!charging_status_data.initialized) {
        LOG_WRN("Driver not initialized");
        return false;
    }
    
    /* 获取当前状态 */
    ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get GPIO state: %d", ret);
        return false;
    }
    
    return (ret == 0); /* ACTIVE_LOW，所以0表示正在充电 */
}

int charging_status_register_callback(void (*callback)(bool is_charging, void *user_data),
                                     void *user_data)
{
    struct charging_status_callback *cb;
    
    if (!callback) {
        return -EINVAL;
    }
    
    /* 分配回调结构 */
    cb = k_malloc(sizeof(struct charging_status_callback));
    if (!cb) {
        return -ENOMEM;
    }
    
    /* 填充回调数据 */
    cb->func = callback;
    cb->user_data = user_data;
    
    /* 添加到链表 */
    k_mutex_lock(&charging_status_data.mutex, K_FOREVER);
    sys_slist_append(&charging_status_data.callbacks, &cb->node);
    k_mutex_unlock(&charging_status_data.mutex);
    
    LOG_DBG("Callback registered");
    
    return 0;
}