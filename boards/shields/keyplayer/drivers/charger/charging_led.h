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
 * @brief Charging LED driver API structure
 */
__subsystem struct charging_led_driver_api {
    void (*set_charging)(const struct device *dev, bool charging);
    void (*enable)(const struct device *dev);
    void (*disable)(const struct device *dev);
};

/**
 * @brief Set charging LED state
 *
 * @param dev Charging LED device instance
 * @param charging true for charging (LED on), false for not charging (LED off)
 */
static inline void charging_led_set_charging(const struct device *dev, bool charging)
{
    const struct charging_led_driver_api *api = dev->api;
    
    if (api->set_charging != NULL) {
        api->set_charging(dev, charging);
    }
}

/**
 * @brief Enable charging LED
 *
 * @param dev Charging LED device instance
 */
static inline void charging_led_enable(const struct device *dev)
{
    const struct charging_led_driver_api *api = dev->api;
    
    if (api->enable != NULL) {
        api->enable(dev);
    }
}

/**
 * @brief Disable charging LED
 *
 * @param dev Charging LED device instance
 */
static inline void charging_led_disable(const struct device *dev)
{
    const struct charging_led_driver_api *api = dev->api;
    
    if (api->disable != NULL) {
        api->disable(dev);
    }
}

#ifdef __cplusplus
}
#endif