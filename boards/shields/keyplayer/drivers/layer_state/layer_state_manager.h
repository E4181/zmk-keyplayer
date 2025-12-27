/*
 * Copyright (c) 2024 ZMK Layer State Manager
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Layer state callback function type.
 * 
 * @param layer The layer number that changed.
 * @param state True if layer was activated, false if deactivated.
 * @param user_data User-provided data passed during registration.
 */
typedef void (*layer_state_callback_t)(uint8_t layer, bool state, void *user_data);

/**
 * @brief LED indicator configuration from device tree.
 */
struct layer_led_config {
    const struct gpio_dt_spec led_gpio;  /**< LED GPIO specification */
    uint8_t trigger_layer;               /**< Layer number that triggers blinking */
    uint32_t blink_duration_ms;          /**< Duration of each blink in milliseconds */
    uint8_t blink_count;                 /**< Number of blinks */
    const char *label;                   /**< LED label */
};

/**
 * @brief LED indicator instance.
 */
struct led_indicator {
    struct gpio_dt_spec led;            /**< LED GPIO */
    uint8_t target_layer;               /**< Target layer for blinking */
    uint32_t blink_duration_ms;         /**< Blink duration in ms */
    uint8_t blink_count;                /**< Number of blinks */
    uint8_t current_blink;              /**< Current blink count */
    bool is_blinking;                   /**< Whether LED is currently blinking */
    struct k_timer blink_timer;         /**< Timer for blinking */
    struct k_work blink_work;           /**< Work queue item for blinking */
};

/**
 * @brief Initialize the layer state manager.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_manager_init(void);

/**
 * @brief Register a callback for layer state changes.
 * 
 * @param callback Function to call when layer state changes.
 * @param user_data User data to pass to the callback.
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_register_callback(layer_state_callback_t callback, void *user_data);

/**
 * @brief Unregister a previously registered callback.
 * 
 * @param callback Callback function to unregister.
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_unregister_callback(layer_state_callback_t callback);

/**
 * @brief Get the current active layers as a bitmask.
 * 
 * @return zmk_keymap_layers_state_t Bitmask of active layers.
 */
zmk_keymap_layers_state_t layer_state_get_active(void);

/**
 * @brief Check if a specific layer is currently active.
 * 
 * @param layer Layer number to check.
 * @return true Layer is active.
 * @return false Layer is not active.
 */
bool layer_state_is_active(uint8_t layer);

/**
 * @brief Get the highest currently active layer index.
 * 
 * @return uint8_t Highest active layer index.
 */
uint8_t layer_state_get_highest_active(void);

/**
 * @brief Print current layer state to logs.
 */
void layer_state_print_current(void);

/**
 * @brief Initialize LED indicator from device tree.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_led_indicator_init(void);

/**
 * @brief Start LED blinking for layer indication.
 * 
 * @param layer The layer that triggered the blinking.
 */
void layer_state_led_start_blinking(uint8_t layer);

/**
 * @brief Stop LED blinking.
 */
void layer_state_led_stop_blinking(void);

/**
 * @brief Check if LED indicator is available.
 * 
 * @return true if LED indicator is available and configured.
 */
bool layer_state_led_is_available(void);

#ifdef __cplusplus
}
#endif