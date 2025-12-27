/*
 * Copyright (c) 2024 ZMK Layer State Manager
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include "layer_state_manager.h"

LOG_MODULE_REGISTER(layer_state_manager, CONFIG_LAYER_STATE_LOG_LEVEL);

/**
 * @brief Layer state manager instance.
 */
static struct layer_state_manager {
    /** Current active layers bitmask */
    zmk_keymap_layers_state_t current_state;
    /** Default layer ID */
    zmk_keymap_layer_id_t default_layer;
    
    /** Registered callbacks */
    struct {
        layer_state_callback_t callback;
        void *user_data;
    } callbacks[CONFIG_LAYER_STATE_MAX_CALLBACKS];
    
    /** Number of registered callbacks */
    uint8_t callback_count;
} manager;

/**
 * @brief LED indicator instance.
 */
static struct led_indicator led_indicator;

/**
 * @brief Check if LED indicator is configured in device tree.
 */
#if DT_NODE_EXISTS(DT_PATH(led_indicator))
#define LED_INDICATOR_AVAILABLE 1
#else
#define LED_INDICATOR_AVAILABLE 0
#endif

/**
 * @brief Work handler for LED blinking.
 * 
 * @param work Work queue item.
 */
static void led_blink_work_handler(struct k_work *work) {
    bool led_state = !gpio_pin_get_dt(&led_indicator.led);
    int ret = gpio_pin_set_dt(&led_indicator.led, led_state);
    
    if (ret < 0) {
        LOG_ERR("Failed to set LED state: %d", ret);
        return;
    }
    
    // If LED was turned off, increment blink count
    if (!led_state) {
        led_indicator.current_blink++;
        
        if (led_indicator.current_blink >= led_indicator.blink_count * 2) {
            // Blinking complete
            led_indicator.is_blinking = false;
            led_indicator.current_blink = 0;
            k_timer_stop(&led_indicator.blink_timer);
            LOG_DBG("LED blinking complete");
        }
    }
}

/**
 * @brief Timer callback for LED blinking.
 * 
 * @param timer The timer that expired.
 */
static void led_blink_timer_handler(struct k_timer *timer) {
    k_work_submit(&led_indicator.blink_work);
}

/**
 * @brief Initialize the LED indicator from device tree.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_led_indicator_init(void) {
#if LED_INDICATOR_AVAILABLE
    // Initialize LED GPIO from device tree
    led_indicator.led = GPIO_DT_SPEC_GET(DT_PATH(led_indicator), led_gpios);
    led_indicator.target_layer = DT_PROP(DT_PATH(led_indicator), trigger_layer);
    led_indicator.blink_duration_ms = DT_PROP(DT_PATH(led_indicator), blink_duration_ms);
    led_indicator.blink_count = DT_PROP(DT_PATH(led_indicator), blink_count);
    
    LOG_INF("LED indicator configured:");
    LOG_INF("  GPIO: %s pin %d", led_indicator.led.port->name, led_indicator.led.pin);
    LOG_INF("  Trigger layer: %d", led_indicator.target_layer);
    LOG_INF("  Blink duration: %d ms", led_indicator.blink_duration_ms);
    LOG_INF("  Blink count: %d", led_indicator.blink_count);
    
    // Configure GPIO
    int ret = gpio_pin_configure_dt(&led_indicator.led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO: %d", ret);
        return ret;
    }
    
    // Initialize timer and work queue
    k_timer_init(&led_indicator.blink_timer, led_blink_timer_handler, NULL);
    k_work_init(&led_indicator.blink_work, led_blink_work_handler);
    
    led_indicator.is_blinking = false;
    led_indicator.current_blink = 0;
    
    LOG_INF("LED indicator initialized successfully");
    return 0;
#else
    LOG_WRN("LED indicator not configured in device tree");
    return -ENODEV;
#endif
}

/**
 * @brief Start LED blinking for layer indication.
 * 
 * @param layer The layer that triggered the blinking.
 */
void layer_state_led_start_blinking(uint8_t layer) {
#if LED_INDICATOR_AVAILABLE
    if (!layer_state_led_is_available()) {
        return;
    }
    
    if (layer == led_indicator.target_layer) {
        // Stop any existing blinking
        layer_state_led_stop_blinking();
        
        // Reset state
        led_indicator.current_blink = 0;
        led_indicator.is_blinking = true;
        
        // Start blinking timer (twice the rate for on/off cycles)
        k_timer_start(&led_indicator.blink_timer,
                     K_MSEC(led_indicator.blink_duration_ms),
                     K_MSEC(led_indicator.blink_duration_ms));
        
        LOG_INF("LED blinking started for layer %d", layer);
    }
#endif
}

/**
 * @brief Stop LED blinking.
 */
void layer_state_led_stop_blinking(void) {
#if LED_INDICATOR_AVAILABLE
    if (led_indicator.is_blinking) {
        k_timer_stop(&led_indicator.blink_timer);
        led_indicator.is_blinking = false;
        led_indicator.current_blink = 0;
        
        // Ensure LED is off
        gpio_pin_set_dt(&led_indicator.led, 0);
        
        LOG_DBG("LED blinking stopped");
    }
#endif
}

/**
 * @brief Check if LED indicator is available.
 * 
 * @return true if LED indicator is available and configured.
 */
bool layer_state_led_is_available(void) {
#if LED_INDICATOR_AVAILABLE
    return gpio_is_ready_dt(&led_indicator.led);
#else
    return false;
#endif
}

/**
 * @brief LED layer callback function.
 * 
 * This function is called when layer state changes and handles LED blinking.
 */
static void led_layer_callback(uint8_t layer, bool state, void *user_data) {
    if (state) {
        layer_state_led_start_blinking(layer);
    }
}

/**
 * @brief Initialize the layer state manager.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_manager_init(void) {
    LOG_INF("Initializing layer state manager");
    
    // Initialize manager structure
    memset(&manager, 0, sizeof(manager));
    
    // Get initial state from ZMK
    manager.current_state = zmk_keymap_layer_state();
    manager.default_layer = zmk_keymap_layer_default();
    
    LOG_INF("Initial layer state: 0x%08X", manager.current_state);
    LOG_INF("Default layer: %d", manager.default_layer);
    
    // Print initial layer states
    for (int i = 0; i < 32; i++) {
        if (zmk_keymap_layer_active(i)) {
            LOG_INF("Initial active layer: %d", i);
        }
    }
    
    // Initialize LED indicator
#if CONFIG_LAYER_STATE_LED_INDICATOR
    int ret = layer_state_led_indicator_init();
    if (ret < 0) {
        LOG_WRN("Failed to initialize LED indicator: %d", ret);
    } else {
        // Register LED callback
        ret = layer_state_register_callback(led_layer_callback, "LED Indicator");
        if (ret < 0 && ret != -EALREADY) {
            LOG_WRN("Failed to register LED callback: %d", ret);
        }
    }
#endif
    
    return 0;
}

// ... (其余函数保持不变，与原始代码相同)
// Register callback, unregister callback, get_active, is_active, get_highest_active, print_current
// layer_state_update, layer_state_notify_callbacks, on_layer_state_changed

/**
 * @brief Layer state changed event handler.
 * 
 * @param eh Event header.
 * @return int Event handling result.
 */
static int on_layer_state_changed(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    
    if (ev != NULL) {
        // Log the event
        LOG_INF("Layer state change: layer=%d, state=%s, timestamp=%lld",
               ev->layer,
               ev->state ? "active" : "inactive",
               ev->timestamp);
        
        // Update internal state
        layer_state_update(ev);
        
        // Notify callbacks
        layer_state_notify_callbacks(ev);
        
        // Print updated state for debugging
        if (CONFIG_LAYER_STATE_LOG_LEVEL >= LOG_LEVEL_DBG) {
            layer_state_print_current();
        }
    }
    
    // Allow other listeners to process the event
    return ZMK_EV_EVENT_BUBBLE;
}

// Define the listener
ZMK_LISTENER(layer_state_listener, on_layer_state_changed);
ZMK_SUBSCRIPTION(layer_state_listener, zmk_layer_state_changed);