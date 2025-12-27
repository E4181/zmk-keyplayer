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
static struct led_indicator led_indicator = {
    .initialized = false,
    .is_blinking = false,
    .current_blink = 0
};

/**
 * @brief Work handler for LED blinking.
 * 
 * @param work Work queue item.
 */
static void led_blink_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    
    if (!led_indicator.initialized || !led_indicator.is_blinking) {
        return;
    }
    
    // Get current LED state and toggle it
    int current_state = gpio_pin_get_dt(&led_indicator.led);
    if (current_state < 0) {
        LOG_ERR("Failed to get LED state: %d", current_state);
        return;
    }
    
    int new_state = current_state ? 0 : 1;
    int ret = gpio_pin_set_dt(&led_indicator.led, new_state);
    
    if (ret < 0) {
        LOG_ERR("Failed to set LED state: %d", ret);
        return;
    }
    
    // If LED was turned off, increment blink count
    if (new_state == 0) {
        led_indicator.current_blink++;
        
        if (led_indicator.current_blink >= led_indicator.blink_count) {
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
    ARG_UNUSED(timer);
    k_work_submit(&led_indicator.blink_work);
}

/**
 * @brief Initialize the LED indicator.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_led_indicator_init(void) {
    int ret = 0;
    
#if CONFIG_LAYER_STATE_LED_INDICATOR
    // 使用硬编码配置：nRF52840 P1.06 (GPIO1 pin 6)
    led_indicator.led.port = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    led_indicator.led.pin = 6;
    led_indicator.led.dt_flags = GPIO_ACTIVE_HIGH;
    
    // 使用配置的默认值
    led_indicator.target_layer = CONFIG_LAYER_STATE_DEFAULT_TRIGGER_LAYER;
    led_indicator.blink_duration_ms = CONFIG_LAYER_STATE_LED_BLINK_DURATION_MS;
    led_indicator.blink_count = CONFIG_LAYER_STATE_LED_BLINK_COUNT;
    
    LOG_INF("Layer LED configured for nRF52840 P1.06");
    LOG_INF("  Target layer: %d", led_indicator.target_layer);
    LOG_INF("  Blink duration: %d ms", led_indicator.blink_duration_ms);
    LOG_INF("  Blink count: %d", led_indicator.blink_count);
    
    // 检查GPIO端口
    if (!device_is_ready(led_indicator.led.port)) {
        LOG_ERR("GPIO1 port not ready");
        return -ENODEV;
    }
    
    // 配置GPIO引脚
    ret = gpio_pin_configure_dt(&led_indicator.led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO: %d", ret);
        return ret;
    }
    
    // 初始化定时器和工作队列
    k_timer_init(&led_indicator.blink_timer, led_blink_timer_handler, NULL);
    k_work_init(&led_indicator.blink_work, led_blink_work_handler);
    
    led_indicator.initialized = true;
    LOG_INF("Layer LED initialized successfully");
    
    return 0;
#else
    LOG_INF("Layer LED indicator disabled in configuration");
    return -ENOTSUP;
#endif
}

/**
 * @brief Start LED blinking for layer indication.
 * 
 * @param layer The layer that triggered the blinking.
 */
void layer_state_led_start_blinking(uint8_t layer) {
    if (!led_indicator.initialized) {
        return;
    }
    
    if (layer == led_indicator.target_layer) {
        // Stop any existing blinking
        if (led_indicator.is_blinking) {
            k_timer_stop(&led_indicator.blink_timer);
        }
        
        // Reset state
        led_indicator.current_blink = 0;
        led_indicator.is_blinking = true;
        
        // Start blinking timer
        k_timer_start(&led_indicator.blink_timer,
                     K_MSEC(led_indicator.blink_duration_ms),
                     K_MSEC(led_indicator.blink_duration_ms));
        
        LOG_INF("LED blinking started for layer %d", layer);
    }
}

/**
 * @brief Stop LED blinking.
 */
void layer_state_led_stop_blinking(void) {
    if (led_indicator.initialized && led_indicator.is_blinking) {
        k_timer_stop(&led_indicator.blink_timer);
        led_indicator.is_blinking = false;
        led_indicator.current_blink = 0;
        
        // Ensure LED is off
        gpio_pin_set_dt(&led_indicator.led, 0);
        
        LOG_DBG("LED blinking stopped");
    }
}

/**
 * @brief Check if LED indicator is available.
 * 
 * @return true if LED indicator is available and configured.
 */
bool layer_state_led_is_available(void) {
    return led_indicator.initialized;
}

/**
 * @brief LED layer callback function.
 * 
 * This function is called when layer state changes and handles LED blinking.
 */
static void led_layer_callback(uint8_t layer, bool state, void *user_data) {
    ARG_UNUSED(user_data);
    
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
        ret = layer_state_register_callback(led_layer_callback, NULL);
        if (ret < 0 && ret != -EALREADY) {
            LOG_WRN("Failed to register LED callback: %d", ret);
        }
    }
#endif
    
    return 0;
}

/**
 * @brief Register a callback for layer state changes.
 * 
 * @param callback Function to call when layer state changes.
 * @param user_data User data to pass to the callback.
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_register_callback(layer_state_callback_t callback, void *user_data) {
    if (callback == NULL) {
        LOG_ERR("Cannot register NULL callback");
        return -EINVAL;
    }
    
    if (manager.callback_count >= CONFIG_LAYER_STATE_MAX_CALLBACKS) {
        LOG_ERR("Maximum number of callbacks (%d) reached", CONFIG_LAYER_STATE_MAX_CALLBACKS);
        return -ENOMEM;
    }
    
    // Check if callback is already registered
    for (int i = 0; i < manager.callback_count; i++) {
        if (manager.callbacks[i].callback == callback) {
            LOG_WRN("Callback already registered");
            return -EALREADY;
        }
    }
    
    // Register the callback
    manager.callbacks[manager.callback_count].callback = callback;
    manager.callbacks[manager.callback_count].user_data = user_data;
    manager.callback_count++;
    
    LOG_DBG("Callback registered, total callbacks: %d", manager.callback_count);
    
    return 0;
}

/**
 * @brief Unregister a previously registered callback.
 * 
 * @param callback Callback function to unregister.
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_unregister_callback(layer_state_callback_t callback) {
    if (callback == NULL) {
        return -EINVAL;
    }
    
    for (int i = 0; i < manager.callback_count; i++) {
        if (manager.callbacks[i].callback == callback) {
            // Move remaining callbacks down
            for (int j = i; j < manager.callback_count - 1; j++) {
                manager.callbacks[j] = manager.callbacks[j + 1];
            }
            manager.callback_count--;
            
            LOG_DBG("Callback unregistered, remaining callbacks: %d", manager.callback_count);
            return 0;
        }
    }
    
    LOG_WRN("Callback not found for unregistration");
    return -ENOENT;
}

/**
 * @brief Get the current active layers as a bitmask.
 * 
 * @return zmk_keymap_layers_state_t Bitmask of active layers.
 */
zmk_keymap_layers_state_t layer_state_get_active(void) {
    return manager.current_state;
}

/**
 * @brief Check if a specific layer is currently active.
 * 
 * @param layer Layer number to check.
 * @return true Layer is active.
 * @return false Layer is not active.
 */
bool layer_state_is_active(uint8_t layer) {
    if (layer >= 32) {
        return false;
    }
    return (manager.current_state & BIT(layer)) != 0 || layer == manager.default_layer;
}

/**
 * @brief Get the highest currently active layer index.
 * 
 * @return uint8_t Highest active layer index.
 */
uint8_t layer_state_get_highest_active(void) {
    for (int i = 31; i >= 0; i--) {
        if (layer_state_is_active(i)) {
            return i;
        }
    }
    return manager.default_layer;
}

/**
 * @brief Print current layer state to logs.
 */
void layer_state_print_current(void) {
    LOG_INF("=== Current Layer State ===");
    LOG_INF("Active layers bitmask: 0x%08X", manager.current_state);
    LOG_INF("Default layer: %d", manager.default_layer);
    LOG_INF("Highest active layer: %d", layer_state_get_highest_active());
    
    LOG_INF("Active layers:");
    for (int i = 0; i < 32; i++) {
        if (layer_state_is_active(i)) {
            LOG_INF("  Layer %d: %s", 
                   i,
                   (i == manager.default_layer) ? "(default)" : "");
        }
    }
}

/**
 * @brief Internal function to update manager state.
 * 
 * @param ev Layer state changed event.
 */
static void layer_state_update(const struct zmk_layer_state_changed *ev) {
    if (ev->layer >= 32) {
        LOG_WRN("Invalid layer number: %d", ev->layer);
        return;
    }
    
    // Update state
    if (ev->state) {
        manager.current_state |= BIT(ev->layer);
    } else {
        manager.current_state &= ~BIT(ev->layer);
    }
    
    // Log the change
#if CONFIG_LAYER_STATE_DEBUG_LOG
    LOG_DBG("Layer %d %s (total active: 0x%08X)", 
           ev->layer,
           ev->state ? "ACTIVATED" : "DEACTIVATED",
           manager.current_state);
#endif
}

/**
 * @brief Internal function to notify all callbacks.
 * 
 * @param ev Layer state changed event.
 */
static void layer_state_notify_callbacks(const struct zmk_layer_state_changed *ev) {
    for (int i = 0; i < manager.callback_count; i++) {
        if (manager.callbacks[i].callback) {
            manager.callbacks[i].callback(ev->layer, ev->state, 
                                          manager.callbacks[i].user_data);
        }
    }
}

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