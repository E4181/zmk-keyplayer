/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/slist.h>

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
 * @brief Charging fault type enumeration
 */
enum charging_fault_type {
    CHARGING_FAULT_NONE = 0,
    CHARGING_FAULT_TIMEOUT,      // 充电超时（长时间充电但电压不上升）
    CHARGING_FAULT_BAD_BATTERY,  // 电池异常
    CHARGING_FAULT_OVER_TEMP,    // 过热保护
    CHARGING_FAULT_SHORT_CIRCUIT // 短路保护
};

/**
 * @brief Charging monitor callback structure
 * 
 * Users should allocate an instance of this structure and pass it to
 * charging_monitor_register_callback().
 */
struct charging_monitor_callback {
    sys_snode_t node; /**< Linked list node */
    void (*function)(const struct device *dev, 
                     enum charging_status status,
                     enum charging_fault_type fault_type,
                     void *user_data);
    void *user_data; /**< User data passed to callback */
};

/**
 * @brief Charging monitor driver API structure
 */
__subsystem struct charging_monitor_driver_api {
    enum charging_status (*get_status)(const struct device *dev);
    bool (*is_charging)(const struct device *dev);
    enum charging_fault_type (*get_fault_type)(const struct device *dev);
    int (*register_callback)(const struct device *dev,
                            struct charging_monitor_callback *callback);
    int (*unregister_callback)(const struct device *dev,
                              struct charging_monitor_callback *callback);
    int (*clear_fault)(const struct device *dev);  // 手动清除故障状态
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
 * @brief Get the current fault type
 *
 * @param dev Charging monitor device instance
 * @return enum charging_fault_type Current fault type
 */
static inline enum charging_fault_type charging_monitor_get_fault_type(const struct device *dev)
{
    const struct charging_monitor_driver_api *api = dev->api;
    
    if (api->get_fault_type == NULL) {
        return CHARGING_FAULT_NONE;
    }
    
    return api->get_fault_type(dev);
}

/**
 * @brief Register a callback for charging status changes
 *
 * @param dev Charging monitor device instance
 * @param callback Callback structure to register
 * @return int 0 on success, negative error code on failure
 */
static inline int charging_monitor_register_callback(const struct device *dev,
                                                   struct charging_monitor_callback *callback)
{
    const struct charging_monitor_driver_api *api = dev->api;

    if (api->register_callback == NULL) {
        return -ENOSYS;
    }

    return api->register_callback(dev, callback);
}

/**
 * @brief Unregister a previously registered callback
 *
 * @param dev Charging monitor device instance
 * @param callback Callback structure to unregister
 * @return int 0 on success, negative error code on failure
 */
static inline int charging_monitor_unregister_callback(const struct device *dev,
                                                     struct charging_monitor_callback *callback)
{
    const struct charging_monitor_driver_api *api = dev->api;

    if (api->unregister_callback == NULL) {
        return -ENOSYS;
    }

    return api->unregister_callback(dev, callback);
}

/**
 * @brief Clear fault status
 *
 * @param dev Charging monitor device instance
 * @return int 0 on success, negative error code on failure
 */
static inline int charging_monitor_clear_fault(const struct device *dev)
{
    const struct charging_monitor_driver_api *api = dev->api;

    if (api->clear_fault == NULL) {
        return -ENOSYS;
    }

    return api->clear_fault(dev);
}

#ifdef __cplusplus
}
#endif