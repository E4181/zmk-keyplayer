/*
 * Copyright (c) 2024 ZMK LED Controller
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// LED状态枚举
enum led_state {
    LED_STATE_OFF,
    LED_STATE_ON,
    LED_STATE_BLINKING,
};

/**
 * @brief Initialize the LED controller.
 * 
 * @param dev Device instance.
 * @return int 0 on success, negative error code on failure.
 */
int led_controller_init(const struct device *dev);

/**
 * @brief Turn LED on.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_on(void);

/**
 * @brief Turn LED off.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_off(void);

/**
 * @brief Toggle LED state.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_toggle(void);

/**
 * @brief Start LED blinking pattern.
 * 
 * @param count Number of blinks.
 * @param interval_ms Interval between blinks in milliseconds.
 * @param duration_ms Duration of each blink in milliseconds.
 * @return int 0 on success, negative error code on failure.
 */
int led_blink(uint8_t count, uint32_t interval_ms, uint32_t duration_ms);

/**
 * @brief Stop any ongoing LED blinking.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int led_stop_blinking(void);

/**
 * @brief Check if LED is currently blinking.
 * 
 * @return true LED is blinking.
 * @return false LED is not blinking.
 */
bool led_is_blinking(void);

/**
 * @brief Get the current LED state.
 * 
 * @return enum led_state Current LED state.
 */
enum led_state led_get_state(void);

#ifdef __cplusplus
}
#endif