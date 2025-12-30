/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "charging_monitor.h"
#include "charging_led.h"

LOG_MODULE_REGISTER(charging_controller, LOG_LEVEL_INF);

// 充电状态变化回调函数
static void charging_status_changed_callback(const struct device *charger_dev,
                                            enum charging_status status,
                                            void *user_data) {
    const struct device *led_dev = (const struct device *)user_data;
    
    LOG_INF("charging_status_changed_callback: status=%d", status);
    
    if (led_dev == NULL) {
        LOG_ERR("LED device is NULL in callback");
        return;
    }
    
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device not ready in callback");
        return;
    }
    
    bool is_charging = (status == CHARGING_STATUS_CHARGING);
    
    LOG_INF("Charging status: %s -> Setting LED: %s",
            is_charging ? "CHARGING" : "IDLE",
            is_charging ? "ON" : "OFF");
    
    charging_led_set_charging(led_dev, is_charging);
}

// 初始化充电控制器
static int charging_controller_init(const struct device *dev) {
    (void)dev;  // 未使用
    
    LOG_INF("Initializing charging controller...");
    
    const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(charging_monitor));
    const struct device *led = DEVICE_DT_GET(DT_NODELABEL(charging_led));
    
    LOG_INF("Charger device: %p", charger);
    LOG_INF("LED device: %p", led);
    
    if (!device_is_ready(charger)) {
        LOG_ERR("Charging monitor device not ready");
        return -ENODEV;
    }
    
    LOG_INF("Charging monitor device is ready");
    
    if (!device_is_ready(led)) {
        LOG_ERR("Charging LED device not ready");
        // LED不可用，但监控功能仍可工作
        return 0;
    }
    
    LOG_INF("Charging LED device is ready");
    
    // 注册回调，将充电状态传递给LED
    int ret = charging_monitor_register_callback(charger, 
                                                charging_status_changed_callback,
                                                (void *)led);
    if (ret < 0) {
        LOG_ERR("Failed to register charging callback: %d", ret);
    } else {
        LOG_INF("Charging callback registered successfully");
        
        // 设置初始LED状态
        bool is_charging = charging_monitor_is_charging(charger);
        LOG_INF("Initial charging state: %s", 
                is_charging ? "CHARGING" : "IDLE");
        
        charging_led_set_charging(led, is_charging);
    }
    
    LOG_INF("Charging controller initialized successfully");
    
    return 0;
}

// 在应用层初始化充电控制器
SYS_INIT(charging_controller_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);