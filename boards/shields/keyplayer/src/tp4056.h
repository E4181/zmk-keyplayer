/*
 * Copyright (c) 2024 ZMK Project
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TP4056 charging states
 */
enum tp4056_charging_state {
    /** Charging in progress */
    TP4056_CHARGING,
    /** Not charging (full, disconnected, or error) */
    TP4056_NOT_CHARGING,
    /** Unknown state (initializing or error) */
    TP4056_STATE_UNKNOWN
};

/**
 * @brief TP4056 event types
 */
enum tp4056_event_type {
    /** Charging state changed */
    TP4056_EVT_CHARGING_STATE_CHANGED,
    /** Error occurred */
    TP4056_EVT_ERROR,
};

/**
 * @brief TP4056 event data structure
 */
struct tp4056_event {
    /** Event type */
    enum tp4056_event_type type;
    /** Event data */
    union {
        /** Charging state for state change events */
        enum tp4056_charging_state charging_state;
        /** Error code for error events */
        int error_code;
    } data;
};

/**
 * @brief Callback function type for TP4056 events
 *
 * @param dev TP4056 device
 * @param event Event data
 * @param user_data User data passed when registering callback
 */
typedef void (*tp4056_callback_t)(const struct device *dev,
                                 const struct tp4056_event *event,
                                 void *user_data);

/**
 * @brief Get current charging state
 *
 * @param dev TP4056 device
 * @param state Pointer to store the charging state
 * @return 0 on success, negative error code on failure
 */
int tp4056_get_charging_state(const struct device *dev,
                             enum tp4056_charging_state *state);

/**
 * @brief Register a callback for TP4056 events
 *
 * @param dev TP4056 device
 * @param callback Callback function
 * @param user_data User data to pass to callback
 * @return 0 on success, negative error code on failure
 */
int tp4056_register_callback(const struct device *dev,
                            tp4056_callback_t callback,
                            void *user_data);

/**
 * @brief Unregister a callback for TP4056 events
 *
 * @param dev TP4056 device
 * @param callback Callback function to unregister
 * @return 0 on success, negative error code on failure
 */
int tp4056_unregister_callback(const struct device *dev,
                              tp4056_callback_t callback);

#ifdef __cplusplus
}
#endif