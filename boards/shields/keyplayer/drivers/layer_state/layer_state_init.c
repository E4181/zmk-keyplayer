/*
 * Copyright (c) 2024 ZMK Layer State Manager
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include "layer_state_manager.h"

#if IS_ENABLED(CONFIG_LAYER_STATE_LED_CONTROL)
#include "led_controller.h"
#endif

LOG_MODULE_REGISTER(layer_state_init, CONFIG_LAYER_STATE_LOG_LEVEL);

/**
 * @brief Example callback function for demonstration.
 * 
 * This function will log every layer state change.
 * You can remove this if you don't need it.
 */
static void example_layer_callback(uint8_t layer, bool state, void *user_data) {
    const char *user = (const char *)user_data;
    LOG_INF("[%s] Layer %d %s", 
           user ? user : "unknown",
           layer,
           state ? "activated" : "deactivated");
}

/**
 * @brief Initialize the layer state manager at application startup.
 * 
 * @return int 0 on success.
 */
static int layer_state_init(void) {
    LOG_INF("Starting layer state manager initialization");
    
    int ret;
    
#if IS_ENABLED(CONFIG_LAYER_STATE_LED_CONTROL)
    // Initialize LED controller
    LOG_INF("Initializing LED controller from device tree...");
    ret = led_controller_init(NULL);
    if (ret < 0) {
        LOG_ERR("LED controller initialization FAILED: %d", ret);
        LOG_WRN("LED functionality will be unavailable, but layer state manager will continue working");
    } else {
        LOG_INF("LED controller initialized SUCCESSFULLY");
    }
#endif
    
    // Initialize the layer state manager
    ret = layer_state_manager_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize layer state manager: %d", ret);
        return ret;
    }
    
    // Register example callback (optional - can be removed)
    ret = layer_state_register_callback(example_layer_callback, "Example");
    if (ret < 0 && ret != -EALREADY) {
        LOG_WRN("Failed to register example callback: %d", ret);
    }
    
    // Print initial state
    layer_state_print_current();
    
    LOG_INF("Layer state manager initialized successfully");
    
#if IS_ENABLED(CONFIG_LAYER_STATE_LED_CONTROL)
    LOG_INF("=== LED Configuration Summary ===");
    LOG_INF("GPIO: From device tree (P1.06)");
    LOG_INF("Target layer: %d", LAYER_LED_TARGET_LAYER);
    LOG_INF("Blink pattern: %d times, %dms on, %dms off", 
           LAYER_LED_BLINK_COUNT,
           LAYER_LED_BLINK_DURATION_MS,
           LAYER_LED_BLINK_INTERVAL_MS - LAYER_LED_BLINK_DURATION_MS);
#endif
    
    return 0;
}

// Register the initialization function
SYS_INIT(layer_state_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);