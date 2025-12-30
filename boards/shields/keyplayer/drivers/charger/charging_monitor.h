/*
 * Copyright (c) 2024 The ZMK Contributors
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
 * @brief Charging status enumeration
 */
enum charging_status {
    /** Not charging or fully charged */
    CHARGING_STATUS_IDLE = 0,
    /** Actively charging */
    CHARGING_STATUS_CHARGING,
    /** Charging fault or error */
    CHARGING_STATUS_FAULT,
};

/**
 * @brief Charging monitor driver API structure
 */
__subsystem struct charging_monitor_driver_api {
    enum charging_status (*get_status)(const struct device *dev);
    bool (*is_charging)(const struct device *dev);
    int (*register_callback)(const struct device *dev,
                            void (*callback)(const struct device *dev,
                                            enum charging_status status,
                                            void *user_data),
                            void *user_data);
};

/**
 * @brief Get the current charging status
 *
 * @param dev Charging monitor device instance
 * @return enum charging_status Current charging status
 */
static inline enum charging_status charging_monitor_get_status(const struct device *dev)
{
    const struct charging_monitor_driver_api *api = dev->api;

    return api->get_status(dev);
}

/**
 * @brief Check if currently charging
 *
 * @param dev Charging monitor device instance
 * @return true if charging, false otherwise
 */
static inline bool charging_monitor_is_charging(const struct device *dev)
{
    const struct charging_monitor_driver_api *api = dev->api;

    return api->is_charging(dev);
}

/**
 * @brief Register a callback for charging status changes
 *
 * @param dev Charging monitor device instance
 * @param callback Function to call on status change
 * @param user_data User data passed to callback
 * @return int 0 on success, negative error code on failure
 */
static inline int charging_monitor_register_callback(const struct device *dev,
                                                   void (*callback)(const struct device *dev,
                                                                   enum charging_status status,
                                                                   void *user_data),
                                                   void *user_data)
{
    const struct charging_monitor_driver_api *api = dev->api;

    if (api->register_callback == NULL) {
        return -ENOSYS;
    }

    return api->register_callback(dev, callback, user_data);
}

#ifdef __cplusplus
}
#endif