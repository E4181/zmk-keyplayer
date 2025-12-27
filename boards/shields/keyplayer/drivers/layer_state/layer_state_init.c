/*
 * Copyright (c) 2024 ZMK Layer State Manager
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zmk/features/layer_state_manager.h>

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
    
    int ret;  // 将变量定义移到函数开头
    
#if IS_ENABLED(CONFIG_LAYER_STATE_LED_CONTROL)
    // Initialize LED controller
    ret = led_controller_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize LED controller: %d", ret);
        // Continue anyway, layer state manager can still work
    } else {
        LOG_INF("LED controller initialized");
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
    LOG_INF("LED control enabled for layer 2:");
    LOG_INF("  - Blink count: %d", CONFIG_LAYER_LED_BLINK_COUNT);
    LOG_INF("  - Blink interval: %d ms", CONFIG_LAYER_LED_BLINK_INTERVAL_MS);
    LOG_INF("  - Blink duration: %d ms", CONFIG_LAYER_LED_BLINK_DURATION_MS);
    LOG_INF("  - GPIO: %s pin %d", CONFIG_LAYER_LED_GPIO_PORT, CONFIG_LAYER_LED_GPIO_PIN);
#endif
    
    return 0;
}

// Register the initialization function
SYS_INIT(layer_state_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);