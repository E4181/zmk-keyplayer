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

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Layer state callback function type.
 * 
 * @param layer The layer number that changed.
 * @param state True if layer was activated, false if deactivated.
 * @param locked True if layer is locked, false otherwise.
 * @param user_data User-provided data passed during registration.
 */
typedef void (*layer_state_callback_t)(uint8_t layer, bool state, bool locked, void *user_data);

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
 * @brief Get the locked layers as a bitmask.
 * 
 * @return zmk_keymap_layers_state_t Bitmask of locked layers.
 */
zmk_keymap_layers_state_t layer_state_get_locked(void);

/**
 * @brief Check if a specific layer is currently active.
 * 
 * @param layer Layer number to check.
 * @return true Layer is active.
 * @return false Layer is not active.
 */
bool layer_state_is_active(uint8_t layer);

/**
 * @brief Check if a specific layer is locked.
 * 
 * @param layer Layer number to check.
 * @return true Layer is locked.
 * @return false Layer is not locked.
 */
bool layer_state_is_locked(uint8_t layer);

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

#ifdef __cplusplus
}
#endif