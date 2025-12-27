/*
 * Copyright (c) 2024 ZMK LED Controller
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "led_controller.h"

LOG_MODULE_REGISTER(led_controller, CONFIG_LAYER_STATE_LOG_LEVEL);

// LED状态机
typedef enum {
    LED_STATE_OFF,
    LED_STATE_ON,
    LED_STATE_BLINKING,
} led_state_t;

// LED控制结构
struct led_controller {
    const struct device *gpio_dev;
    gpio_pin_t pin;
    gpio_flags_t flags;
    
    led_state_t state;
    struct k_work_delayable blink_work;
    uint8_t blink_count;
    uint8_t blink_remaining;
    uint32_t blink_interval_ms;
    uint32_t blink_duration_ms;
    bool blink_on_phase;
};

static struct led_controller led_ctrl;

/**
 * @brief Initialize GPIO pin for LED control.
 * 
 * @return int 0 on success, negative error code on failure.
 */
static int led_gpio_init(void) {
    LOG_INF("Initializing LED GPIO for P1.06 (pin 38)...");
    
    // 尝试查找GPIO设备
    const char* gpio_names[] = {
        "gpio1",     // 首选：GPIO1对应P1端口
        "GPIO_1",    // 备用1
        "GPIO1",     // 备用2
        "gpio0",     // 备用3
        "GPIO_0",    // 备用4
        "GPIO0",     // 备用5
        NULL
    };
    
    for (int i = 0; gpio_names[i] != NULL; i++) {
        LOG_INF("Trying GPIO device: %s", gpio_names[i]);
        led_ctrl.gpio_dev = device_get_binding(gpio_names[i]);
        if (led_ctrl.gpio_dev != NULL) {
            LOG_INF("Found GPIO device: %s", gpio_names[i]);
            break;
        }
    }
    
    if (led_ctrl.gpio_dev == NULL) {
        LOG_ERR("No GPIO device found!");
        return -ENODEV;
    }
    
    // 配置引脚为输出
    led_ctrl.pin = CONFIG_LAYER_LED_GPIO_PIN;  // GPIO 38
    led_ctrl.flags = GPIO_OUTPUT;
    
    if (CONFIG_LAYER_LED_ACTIVE_HIGH) {
        led_ctrl.flags |= GPIO_OUTPUT_INIT_LOW;  // 初始低电平
        LOG_INF("LED active high (1=ON, 0=OFF)");
    } else {
        led_ctrl.flags |= GPIO_OUTPUT_INIT_HIGH;  // 初始高电平
        LOG_INF("LED active low (0=ON, 1=OFF)");
    }
    
    int ret = gpio_pin_configure(led_ctrl.gpio_dev, led_ctrl.pin, led_ctrl.flags);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin %d: %d", led_ctrl.pin, ret);
        return ret;
    }
    
    LOG_INF("LED GPIO initialized successfully");
    LOG_INF("  - Device: %s", device_get_name(led_ctrl.gpio_dev));
    LOG_INF("  - Pin: %d (P1.06)", led_ctrl.pin);
    LOG_INF("  - Active: %s", CONFIG_LAYER_LED_ACTIVE_HIGH ? "high" : "low");
    
    return 0;
}

/**
 * @brief Set LED state.
 * 
 * @param on True to turn LED on, false to turn off.
 * @return int 0 on success, negative error code on failure.
 */
static int led_set(bool on) {
    if (led_ctrl.gpio_dev == NULL) {
        LOG_ERR("GPIO device not initialized");
        return -ENODEV;
    }
    
    int value = on ? 1 : 0;
    if (!CONFIG_LAYER_LED_ACTIVE_HIGH) {
        value = !value;  // 反转电平
    }
    
    LOG_DBG("Setting LED pin %d to %d (LED %s)", led_ctrl.pin, value, on ? "ON" : "OFF");
    
    int ret = gpio_pin_set(led_ctrl.gpio_dev, led_ctrl.pin, value);
    if (ret < 0) {
        LOG_ERR("Failed to set LED pin %d: %d", led_ctrl.pin, ret);
        return ret;
    }
    
    led_ctrl.state = on ? LED_STATE_ON : LED_STATE_OFF;
    
    return 0;
}

/**
 * @brief Blink work handler.
 * 
 * @param work Work item.
 */
static void led_blink_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    
    if (led_ctrl.blink_remaining == 0) {
        // 闪烁完成，关闭LED
        LOG_INF("LED blinking completed");
        led_off();
        return;
    }
    
    if (led_ctrl.blink_on_phase) {
        // 闪烁亮阶段结束，准备进入灭阶段
        LOG_DBG("Blink ON phase finished, turning OFF");
        led_off();
        led_ctrl.blink_on_phase = false;
        
        // 如果还有更多闪烁，安排下一次
        if (led_ctrl.blink_remaining > 0) {
            uint32_t off_time = led_ctrl.blink_interval_ms - led_ctrl.blink_duration_ms;
            LOG_DBG("Waiting %d ms before next blink", off_time);
            k_work_schedule(dwork, K_MSEC(off_time));
        }
    } else {
        // 闪烁灭阶段结束，开始下一个闪烁
        if (led_ctrl.blink_remaining > 0) {
            LOG_DBG("Starting blink %d/%d", 
                   CONFIG_LAYER_LED_BLINK_COUNT - led_ctrl.blink_remaining + 1,
                   CONFIG_LAYER_LED_BLINK_COUNT);
            
            led_on();
            led_ctrl.blink_on_phase = true;
            led_ctrl.blink_remaining--;
            
            // 安排关闭LED
            LOG_DBG("LED will turn off in %d ms", led_ctrl.blink_duration_ms);
            k_work_schedule(dwork, K_MSEC(led_ctrl.blink_duration_ms));
        }
    }
}

/**
 * @brief Initialize the LED controller.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_controller_init(void) {
    LOG_INF("=== LED CONTROLLER INITIALIZATION ===");
    LOG_INF("Hardware: P1.06 (GPIO pin 38), Active High");
    
    // 初始化结构
    memset(&led_ctrl, 0, sizeof(led_ctrl));
    led_ctrl.state = LED_STATE_OFF;
    
    // 初始化GPIO
    int ret = led_gpio_init();
    if (ret < 0) {
        LOG_ERR("LED GPIO initialization FAILED: %d", ret);
        return ret;
    }
    
    // 初始化工作队列
    k_work_init_delayable(&led_ctrl.blink_work, led_blink_work_handler);
    
    // 确保LED初始状态为关闭
    led_off();
    
    // 测试LED功能
    LOG_INF("Testing LED functionality...");
    for (int i = 0; i < 2; i++) {
        led_on();
        k_msleep(100);
        led_off();
        k_msleep(100);
    }
    
    LOG_INF("LED controller initialized SUCCESSFULLY");
    return 0;
}

/**
 * @brief Turn LED on.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_on(void) {
    LOG_DBG("Turning LED ON");
    // 停止任何正在进行的闪烁
    led_stop_blinking();
    return led_set(true);
}

/**
 * @brief Turn LED off.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_off(void) {
    LOG_DBG("Turning LED OFF");
    // 停止任何正在进行的闪烁
    led_stop_blinking();
    return led_set(false);
}

/**
 * @brief Toggle LED state.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_toggle(void) {
    if (led_ctrl.gpio_dev == NULL) {
        LOG_ERR("GPIO device not initialized");
        return -ENODEV;
    }
    
    LOG_DBG("Toggling LED");
    // 停止任何正在进行的闪烁
    led_stop_blinking();
    
    // 读取当前状态
    int value = gpio_pin_get(led_ctrl.gpio_dev, led_ctrl.pin);
    if (value < 0) {
        LOG_ERR("Failed to get LED pin state: %d", value);
        return value;
    }
    
    bool current_on = (value != 0);
    if (!CONFIG_LAYER_LED_ACTIVE_HIGH) {
        current_on = !current_on;
    }
    
    return led_set(!current_on);
}

/**
 * @brief Start LED blinking pattern.
 * 
 * @param count Number of blinks.
 * @param interval_ms Interval between blinks in milliseconds.
 * @param duration_ms Duration of each blink in milliseconds.
 * @return int 0 on success, negative error code on failure.
 */
int led_blink(uint8_t count, uint32_t interval_ms, uint32_t duration_ms) {
    LOG_INF("Starting LED blink: count=%d, interval=%dms, duration=%dms",
           count, interval_ms, duration_ms);
    
    if (led_ctrl.gpio_dev == NULL) {
        LOG_ERR("GPIO device not initialized!");
        return -ENODEV;
    }
    
    if (count == 0 || interval_ms == 0 || duration_ms == 0 || duration_ms >= interval_ms) {
        LOG_ERR("Invalid blink parameters");
        return -EINVAL;
    }
    
    // 停止任何正在进行的闪烁
    led_stop_blinking();
    
    // 设置闪烁参数
    led_ctrl.blink_count = count;
    led_ctrl.blink_remaining = count;
    led_ctrl.blink_interval_ms = interval_ms;
    led_ctrl.blink_duration_ms = duration_ms;
    led_ctrl.blink_on_phase = false;
    led_ctrl.state = LED_STATE_BLINKING;
    
    LOG_INF("LED blink configured, starting pattern...");
    
    // 立即开始第一次闪烁
    k_work_schedule(&led_ctrl.blink_work, K_NO_WAIT);
    
    return 0;
}

/**
 * @brief Stop any ongoing LED blinking.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_stop_blinking(void) {
    if (led_ctrl.state == LED_STATE_BLINKING) {
        LOG_DBG("Stopping LED blinking");
        int ret = k_work_cancel_delayable(&led_ctrl.blink_work);
        if (ret < 0 && ret != -EINPROGRESS) {
            LOG_WRN("Failed to cancel blink work: %d", ret);
        }
        
        led_ctrl.state = LED_STATE_OFF;
        led_ctrl.blink_remaining = 0;
        led_ctrl.blink_on_phase = false;
    }
    
    return 0;
}

/**
 * @brief Check if LED is currently blinking.
 * 
 * @return true LED is blinking.
 * @return false LED is not blinking.
 */
bool led_is_blinking(void) {
    return led_ctrl.state == LED_STATE_BLINKING;
}

/**
 * @brief Get the current LED state.
 * 
 * @return led_state_t Current LED state.
 */
led_state_t led_get_state(void) {
    return led_ctrl.state;
}