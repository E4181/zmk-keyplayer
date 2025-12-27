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

/* 设备树配置 */
#define CHRG_NODE DT_NODELABEL(charger)
#define CHRG_GPIO_LABEL DT_GPIO_LABEL(CHRG_NODE, chrg_gpios)
#define CHRG_GPIO_PIN DT_GPIO_PIN(CHRG_NODE, chrg_gpios)
#define CHRG_GPIO_FLAGS DT_GPIO_FLAGS(CHRG_NODE, chrg_gpios)

#define LED_NODE DT_NODELABEL(charger_led)
#define LED_GPIO_LABEL DT_GPIO_LABEL(LED_NODE, led_gpios)
#define LED_GPIO_PIN DT_GPIO_PIN(LED_NODE, led_gpios)
#define LED_GPIO_FLAGS DT_GPIO_FLAGS(LED_NODE, led_gpios)

#define DEBOUNCE_MS DT_PROP(CHRG_NODE, debounce_ms)

/* 静态变量 */
static enum charger_state current_state = CHARGER_STATE_UNKNOWN;
static const struct device *chrg_gpio;
static const struct device *led_gpio;
static struct gpio_callback chrg_cb;
static struct k_work_delayable debounce_work;
static struct k_work status_work;
static struct k_work led_update_work;

/* 事件实现 */
ZMK_EVENT_IMPL(zmk_charger_state_changed);

/* 私有函数声明 */
static void charger_status_work_handler(struct k_work *work);
static void charger_debounce_work_handler(struct k_work *work);
static void charger_gpio_callback(const struct device *dev,
                                  struct gpio_callback *cb,
                                  uint32_t pins);
static void led_update_work_handler(struct k_work *work);
static void update_led_state(enum charger_state state);

/* 状态工作处理器 */
static void charger_status_work_handler(struct k_work *work) {
    int val = gpio_pin_get(chrg_gpio, CHRG_GPIO_PIN);
    
    if (val < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", val);
        return;
    }
    
    enum charger_state new_state = (val == 0) ? 
        CHARGER_STATE_CHARGING : CHARGER_STATE_NOT_CHARGING;
    
    if (new_state != current_state) {
        LOG_INF("Charger state changed: %s -> %s",
                current_state == CHARGER_STATE_CHARGING ? "CHARGING" : 
                current_state == CHARGER_STATE_NOT_CHARGING ? "NOT_CHARGING" : "UNKNOWN",
                new_state == CHARGER_STATE_CHARGING ? "CHARGING" : "NOT_CHARGING");
        
        current_state = new_state;
        
        // 发布充电状态变化事件
        raise_zmk_charger_state_changed((struct zmk_charger_state_changed){
            .state = new_state
        });
        
        // 更新LED状态
        k_work_submit(&led_update_work);
    }
}

/* 消抖工作处理器 */
static void charger_debounce_work_handler(struct k_work *work) {
    k_work_submit(&status_work);
}

/* GPIO中断回调 */
static void charger_gpio_callback(const struct device *dev,
                                  struct gpio_callback *cb,
                                  uint32_t pins) {
    if (pins & BIT(CHRG_GPIO_PIN)) {
        k_work_reschedule(&debounce_work, K_MSEC(DEBOUNCE_MS));
    }
}

/* LED更新工作处理器 */
static void led_update_work_handler(struct k_work *work) {
    update_led_state(current_state);
}

/* 更新LED状态 */
static void update_led_state(enum charger_state state) {
    if (!device_is_ready(led_gpio)) {
        LOG_ERR("LED GPIO device not ready");
        return;
    }
    
    int ret;
    switch (state) {
        case CHARGER_STATE_CHARGING:
            ret = gpio_pin_set(led_gpio, LED_GPIO_PIN, 1);
            if (ret < 0) {
                LOG_ERR("Failed to turn on LED: %d", ret);
            }
            break;
            
        case CHARGER_STATE_NOT_CHARGING:
        case CHARGER_STATE_UNKNOWN:
            ret = gpio_pin_set(led_gpio, LED_GPIO_PIN, 0);
            if (ret < 0) {
                LOG_ERR("Failed to turn off LED: %d", ret);
            }
            break;
    }
}

/* 初始化函数 */
static int charger_tp4056_init(void) {
    int ret;
    
    /* 初始化CHRG GPIO */
    chrg_gpio = device_get_binding(CHRG_GPIO_LABEL);
    if (!device_is_ready(chrg_gpio)) {
        LOG_ERR("CHRG GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure(chrg_gpio, CHRG_GPIO_PIN,
                             GPIO_INPUT | GPIO_PULL_UP | CHRG_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    /* 初始化LED GPIO */
    led_gpio = device_get_binding(LED_GPIO_LABEL);
    if (!device_is_ready(led_gpio)) {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure(led_gpio, LED_GPIO_PIN,
                             GPIO_OUTPUT_INACTIVE | LED_GPIO_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO: %d", ret);
        return ret;
    }
    
    /* 初始化GPIO中断 */
    ret = gpio_pin_interrupt_configure(chrg_gpio, CHRG_GPIO_PIN,
                                       GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt: %d", ret);
        return ret;
    }
    
    /* 初始化工作队列 */
    k_work_init(&status_work, charger_status_work_handler);
    k_work_init_delayable(&debounce_work, charger_debounce_work_handler);
    k_work_init(&led_update_work, led_update_work_handler);
    
    /* 设置GPIO回调 */
    gpio_init_callback(&chrg_cb, charger_gpio_callback, BIT(CHRG_GPIO_PIN));
    gpio_add_callback(chrg_gpio, &chrg_cb);
    
    /* 读取初始状态 */
    k_work_submit(&status_work);
    
    LOG_INF("TP4056 charger driver initialized with LED on GPIO %s pin %d",
            LED_GPIO_LABEL, LED_GPIO_PIN);
    
    return 0;
}

/* 公共API函数 */
enum charger_state zmk_charger_get_state(void) {
    return current_state;
}

bool zmk_charger_is_charging(void) {
    return current_state == CHARGER_STATE_CHARGING;
}

/* 初始化调用 */
SYS_INIT(charger_tp4056_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);