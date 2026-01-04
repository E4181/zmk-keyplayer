/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#include "charging_status.h"

LOG_MODULE_REGISTER(charging_status, CONFIG_CHARGING_STATUS_LOG_LEVEL);

/* 硬编码 GPIO 配置 - P1.09 (nRF52840) */
#define CHARGING_STATUS_PORT DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define CHARGING_STATUS_PIN 9
#define CHARGING_STATUS_FLAGS (GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW)

/* 驱动数据结构 */
static struct {
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;
    bool last_state;
    bool initialized;
} charging_status_data;

/* GPIO 中断回调函数 */
static void charging_status_gpio_callback(const struct device *dev,
                                         struct gpio_callback *cb,
                                         uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    
    if (!charging_status_data.initialized) {
        return;
    }
    
    /* 获取当前状态 */
    int ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get GPIO state: %d", ret);
        return;
    }
    
    bool current_state = (ret == 0); /* 注意：ACTIVE_LOW，所以0表示正在充电 */
    
    /* 如果状态改变，记录日志 */
    if (current_state != charging_status_data.last_state) {
        charging_status_data.last_state = current_state;
        
        if (current_state) {
            LOG_INF("🔌 CHARGING STARTED - CHRG pin is LOW (GPIO P1.%02d)", CHARGING_STATUS_PIN);
        } else {
            LOG_INF("🔋 CHARGING STOPPED - CHRG pin is HIGH (GPIO P1.%02d)", CHARGING_STATUS_PIN);
        }
    }
}

int charging_status_init(void)
{
    int ret;
    
    LOG_INF("Initializing charging status driver...");
    
    /* 检查是否已经初始化 */
    if (charging_status_data.initialized) {
        LOG_INF("Charging status driver already initialized");
        return 0;
    }
    
    /* 初始化数据结构 */
    charging_status_data.last_state = false;
    charging_status_data.initialized = false;
    
    /* 获取 GPIO 设备 */
    charging_status_data.gpio_dev = CHARGING_STATUS_PORT;
    if (!device_is_ready(charging_status_data.gpio_dev)) {
        LOG_ERR("GPIO device not ready for P1.%02d", CHARGING_STATUS_PIN);
        return -ENODEV;
    }
    
    LOG_INF("GPIO device ready for P1.%02d", CHARGING_STATUS_PIN);
    
    /* 配置 GPIO 引脚 */
    ret = gpio_pin_configure(charging_status_data.gpio_dev, CHARGING_STATUS_PIN,
                            CHARGING_STATUS_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO P1.%02d: %d", CHARGING_STATUS_PIN, ret);
        return ret;
    }
    
    LOG_INF("GPIO P1.%02d configured as INPUT with PULL_UP and ACTIVE_LOW", CHARGING_STATUS_PIN);
    
    /* 初始化 GPIO 回调 */
    gpio_init_callback(&charging_status_data.gpio_cb,
                      charging_status_gpio_callback,
                      BIT(CHARGING_STATUS_PIN));
    
    /* 添加回调 */
    ret = gpio_add_callback(charging_status_data.gpio_dev, &charging_status_data.gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback for P1.%02d: %d", CHARGING_STATUS_PIN, ret);
        return ret;
    }
    
    /* 启用 GPIO 中断 */
    ret = gpio_pin_interrupt_configure(charging_status_data.gpio_dev,
                                      CHARGING_STATUS_PIN,
                                      GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO interrupt for P1.%02d: %d", CHARGING_STATUS_PIN, ret);
        return ret;
    }
    
    LOG_INF("GPIO interrupt enabled for P1.%02d (EDGE_BOTH)", CHARGING_STATUS_PIN);
    
    /* 获取初始状态 */
    ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get initial GPIO state for P1.%02d: %d", CHARGING_STATUS_PIN, ret);
        return ret;
    }
    
    charging_status_data.last_state = (ret == 0); /* ACTIVE_LOW */
    charging_status_data.initialized = true;
    
    LOG_INF("Charging status driver initialized successfully");
    LOG_INF("Initial CHRG pin state: %s (GPIO P1.%02d = %d)", 
            charging_status_data.last_state ? "CHARGING" : "NOT CHARGING",
            CHARGING_STATUS_PIN,
            ret);
    
    return 0;
}

bool charging_status_is_charging(void)
{
    int ret;
    bool is_charging;
    
    if (!charging_status_data.initialized) {
        LOG_WRN("Charging status driver not initialized");
        return false;
    }
    
    /* 获取当前状态 */
    ret = gpio_pin_get(charging_status_data.gpio_dev, CHARGING_STATUS_PIN);
    if (ret < 0) {
        LOG_ERR("Failed to get GPIO state: %d", ret);
        return false;
    }
    
    is_charging = (ret == 0); /* ACTIVE_LOW，所以0表示正在充电 */
    
    return is_charging;
}

void charging_status_log_detailed(void)
{
    if (!charging_status_data.initialized) {
        LOG_WRN("Driver not initialized");
        return;
    }
    
    int raw_level = charging_status_is_charging() ? 0 : 1;
    bool is_charging = charging_status_is_charging();
    
    LOG_INF("Charging status details:");
    LOG_INF("  - GPIO Port: GPIO1");
    LOG_INF("  - GPIO Pin: %d", CHARGING_STATUS_PIN);
    LOG_INF("  - Raw level: %d", raw_level);
    LOG_INF("  - Interpreted: %s", is_charging ? "CHARGING" : "NOT CHARGING");
    LOG_INF("  - GPIO config: INPUT | PULL_UP | ACTIVE_LOW");
    LOG_INF("  - Last state: %s", charging_status_data.last_state ? "CHARGING" : "NOT CHARGING");
    LOG_INF("  - Driver initialized: %s", charging_status_data.initialized ? "YES" : "NO");
}

/* 使用 SYS_INIT 确保驱动在系统启动时自动初始化 */
static int charging_status_sys_init(void)
{
    int ret;
    
    LOG_INF("SYS_INIT: Starting charging status driver initialization");
    
    ret = charging_status_init();
    if (ret < 0) {
        LOG_ERR("SYS_INIT: Failed to initialize charging status driver: %d", ret);
        return ret;
    }
    
    LOG_INF("SYS_INIT: Charging status driver initialization complete");
    
    return 0;
}

/* 使用 POST_KERNEL 初始化级别，优先级设置为 90（在 GPIO 驱动之后） */
SYS_INIT(charging_status_sys_init, POST_KERNEL, 90);