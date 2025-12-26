/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Charger state enumeration
 */
enum zmk_charger_state {
    /** Charger is actively charging */
    ZMK_CHARGER_STATE_CHARGING,
    /** Charger is not charging (idle or disconnected) */
    ZMK_CHARGER_STATE_NOT_CHARGING,
    /** Charger state is unknown or error */
    ZMK_CHARGER_STATE_UNKNOWN,
    /** Charger monitor is disabled */
    ZMK_CHARGER_STATE_DISABLED
};

/**
 * @brief Charger monitor callback function type
 */
typedef void (*zmk_charger_state_changed_cb_t)(enum zmk_charger_state state, void *user_data);

/**
 * @brief Charger monitor API structure
 */
struct zmk_charger_monitor_api {
    /**
     * @brief Get current charger state
     * @param dev Charger monitor device
     * @return Current charger state
     */
    enum zmk_charger_state (*get_state)(const struct device *dev);
    
    /**
     * @brief Register state change callback
     * @param dev Charger monitor device
     * @param callback Callback function
     * @param user_data User data passed to callback
     * @return 0 on success, negative error code on failure
     */
    int (*register_callback)(const struct device *dev,
                             zmk_charger_state_changed_cb_t callback,
                             void *user_data);
    
    /**
     * @brief Unregister state change callback
     * @param dev Charger monitor device
     * @param callback Callback function to unregister
     * @return 0 on success, negative error code on failure
     */
    int (*unregister_callback)(const struct device *dev,
                               zmk_charger_state_changed_cb_t callback);
    
    /**
     * @brief Enable charger monitor
     * @param dev Charger monitor device
     * @return 0 on success, negative error code on failure
     */
    int (*enable)(const struct device *dev);
    
    /**
     * @brief Disable charger monitor
     * @param dev Charger monitor device
     * @return 0 on success, negative error code on failure
     */
    int (*disable)(const struct device *dev);
    
    /**
     * @brief Check if monitor is enabled
     * @param dev Charger monitor device
     * @return true if enabled, false otherwise
     */
    bool (*is_enabled)(const struct device *dev);
};

/**
 * @brief Get current charger state
 * @param dev Charger monitor device
 * @return Current charger state
 */
static inline enum zmk_charger_state zmk_charger_monitor_get_state(const struct device *dev)
{
    const struct zmk_charger_monitor_api *api = 
        (const struct zmk_charger_monitor_api *)dev->api;
    
    return api->get_state(dev);
}

/**
 * @brief Register state change callback
 * @param dev Charger monitor device
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return 0 on success, negative error code on failure
 */
static inline int zmk_charger_monitor_register_callback(const struct device *dev,
                                                        zmk_charger_state_changed_cb_t callback,
                                                        void *user_data)
{
    const struct zmk_charger_monitor_api *api = 
        (const struct zmk_charger_monitor_api *)dev->api;
    
    if (api->register_callback == NULL) {
        return -ENOTSUP;
    }
    
    return api->register_callback(dev, callback, user_data);
}

/**
 * @brief Unregister state change callback
 * @param dev Charger monitor device
 * @param callback Callback function to unregister
 * @return 0 on success, negative error code on failure
 */
static inline int zmk_charger_monitor_unregister_callback(const struct device *dev,
                                                          zmk_charger_state_changed_cb_t callback)
{
    const struct zmk_charger_monitor_api *api = 
        (const struct zmk_charger_monitor_api *)dev->api;
    
    if (api->unregister_callback == NULL) {
        return -ENOTSUP;
    }
    
    return api->unregister_callback(dev, callback);
}

/**
 * @brief Enable charger monitor
 * @param dev Charger monitor device
 * @return 0 on success, negative error code on failure
 */
static inline int zmk_charger_monitor_enable(const struct device *dev)
{
    const struct zmk_charger_monitor_api *api = 
        (const struct zmk_charger_monitor_api *)dev->api;
    
    if (api->enable == NULL) {
        return -ENOTSUP;
    }
    
    return api->enable(dev);
}

/**
 * @brief Disable charger monitor
 * @param dev Charger monitor device
 * @return 0 on success, negative error code on failure
 */
static inline int zmk_charger_monitor_disable(const struct device *dev)
{
    const struct zmk_charger_monitor_api *api = 
        (const struct zmk_charger_monitor_api *)dev->api;
    
    if (api->disable == NULL) {
        return -ENOTSUP;
    }
    
    return api->disable(dev);
}

/**
 * @brief Check if monitor is enabled
 * @param dev Charger monitor device
 * @return true if enabled, false otherwise
 */
static inline bool zmk_charger_monitor_is_enabled(const struct device *dev)
{
    const struct zmk_charger_monitor_api *api = 
        (const struct zmk_charger_monitor_api *)dev->api;
    
    if (api->is_enabled == NULL) {
        return false;
    }
    
    return api->is_enabled(dev);
}

/**
 * @brief Get charger monitor device from device tree
 * @param node_id Device tree node identifier
 * @return Pointer to charger monitor device
 */
#define ZMK_CHARGER_MONITOR_DEVICE(node_id) DEVICE_DT_GET(node_id)

#ifdef __cplusplus
}
#endif