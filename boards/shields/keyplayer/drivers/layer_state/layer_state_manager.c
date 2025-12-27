/*
 * Copyright (c) 2024 ZMK Layer State Manager
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include "layer_state_manager.h"

#if IS_ENABLED(CONFIG_LAYER_STATE_LED_CONTROL)
#include "led_controller.h"
#endif

LOG_MODULE_REGISTER(layer_state_manager, CONFIG_LAYER_STATE_LOG_LEVEL);

// 从设备树获取层LED配置
#define LAYER_STATE_NODE DT_PATH(layer_state_manager)

#if DT_NODE_EXISTS(LAYER_STATE_NODE)
    #define LAYER_LED_TARGET_LAYER DT_PROP(LAYER_STATE_NODE, layer)
    #define LAYER_LED_BLINK_COUNT DT_PROP(LAYER_STATE_NODE, count)
    #define LAYER_LED_BLINK_INTERVAL_MS DT_PROP(LAYER_STATE_NODE, interval_ms)
    #define LAYER_LED_BLINK_DURATION_MS DT_PROP(LAYER_STATE_NODE, duration_ms)
#else
    // 默认配置
    #define LAYER_LED_TARGET_LAYER 2
    #define LAYER_LED_BLINK_COUNT 5
    #define LAYER_LED_BLINK_INTERVAL_MS 500
    #define LAYER_LED_BLINK_DURATION_MS 200
#endif

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
    
    /** Last triggered layer (for LED control) */
    uint8_t last_triggered_layer;
    /** Timestamp of last layer change */
    int64_t last_change_timestamp;
} manager;

/**
 * @brief Initialize the layer state manager.
 * 
 * @return int 0 on success, negative error code on failure.
 */
int layer_state_manager_init(void) {
    LOG_INF("=== LAYER STATE MANAGER INIT ===");
    
#if DT_NODE_EXISTS(LAYER_STATE_NODE)
    LOG_INF("Configuration from device tree:");
    LOG_INF("  - Target layer: %d", LAYER_LED_TARGET_LAYER);
    LOG_INF("  - Blink count: %d", LAYER_LED_BLINK_COUNT);
    LOG_INF("  - Blink interval: %d ms", LAYER_LED_BLINK_INTERVAL_MS);
    LOG_INF("  - Blink duration: %d ms", LAYER_LED_BLINK_DURATION_MS);
#else
    LOG_INF("Using default configuration (device tree node not found)");
    LOG_INF("  - Target layer: %d", LAYER_LED_TARGET_LAYER);
    LOG_INF("  - Blink count: %d", LAYER_LED_BLINK_COUNT);
    LOG_INF("  - Blink interval: %d ms", LAYER_LED_BLINK_INTERVAL_MS);
    LOG_INF("  - Blink duration: %d ms", LAYER_LED_BLINK_DURATION_MS);
#endif
    
    // Initialize manager structure
    memset(&manager, 0, sizeof(manager));
    
    // Get initial state from ZMK
    manager.current_state = zmk_keymap_layer_state();
    manager.default_layer = zmk_keymap_layer_default();
    manager.last_triggered_layer = 0xFF; // Invalid value
    manager.last_change_timestamp = k_uptime_get();
    
    LOG_INF("Initial layer state: 0x%08X", manager.current_state);
    LOG_INF("Default layer: %d", manager.default_layer);
    
    // Print initial layer states
    for (int i = 0; i < 32; i++) {
        if (zmk_keymap_layer_active(i)) {
            LOG_INF("Initial active layer: %d", i);
        }
    }
    
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
 * @brief Internal function to control LED based on layer state.
 * 
 * @param layer Layer that changed.
 * @param state New state of the layer.
 */
static void layer_state_control_led(uint8_t layer, bool state) {
#if IS_ENABLED(CONFIG_LAYER_STATE_LED_CONTROL)
    LOG_INF("LED Control: layer=%d, state=%s", layer, state ? "active" : "inactive");
    
    // 检查是否是目标层
    if (layer == LAYER_LED_TARGET_LAYER && state) {
        LOG_INF(">>> Layer %d ACTIVATED - triggering LED blink <<<", layer);
        
        // 启动LED闪烁
        int ret = led_blink(LAYER_LED_BLINK_COUNT,
                           LAYER_LED_BLINK_INTERVAL_MS,
                           LAYER_LED_BLINK_DURATION_MS);
        
        if (ret < 0) {
            LOG_ERR("Failed to start LED blink: %d", ret);
        } else {
            manager.last_triggered_layer = layer;
            manager.last_change_timestamp = k_uptime_get();
            LOG_INF("LED blink started successfully");
        }
    } else if (layer == LAYER_LED_TARGET_LAYER && !state) {
        LOG_INF("Layer %d deactivated", layer);
    }
#endif // CONFIG_LAYER_STATE_LED_CONTROL
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
    
    // 记录状态变化前
    zmk_keymap_layers_state_t old_state = manager.current_state;
    
    // Update state
    if (ev->state) {
        manager.current_state |= BIT(ev->layer);
    } else {
        manager.current_state &= ~BIT(ev->layer);
    }
    
    LOG_INF("State updated: layer=%d, new_state=%s, old_bitmask=0x%08X, new_bitmask=0x%08X",
           ev->layer,
           ev->state ? "active" : "inactive",
           old_state,
           manager.current_state);
}

/**
 * @brief Internal function to notify all callbacks.
 * 
 * @param ev Layer state changed event.
 */
static void layer_state_notify_callbacks(const struct zmk_layer_state_changed *ev) {
    LOG_INF("Notifying %d callbacks", manager.callback_count);
    for (int i = 0; i < manager.callback_count; i++) {
        if (manager.callbacks[i].callback) {
            LOG_INF("  Calling callback %d", i);
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
    LOG_INF("=== LAYER STATE CHANGED EVENT RECEIVED ===");
    
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    
    if (ev != NULL) {
        // Log the event
        LOG_INF("Layer state change EVENT: layer=%d, state=%s, locked=%d, timestamp=%lld",
               ev->layer,
               ev->state ? "active" : "inactive",
               ev->locked,
               ev->timestamp);
        
        // Update internal state
        layer_state_update(ev);
        
        // Control LED based on layer state
        layer_state_control_led(ev->layer, ev->state);
        
        // Notify callbacks
        layer_state_notify_callbacks(ev);
        
        // Print updated state for debugging
        LOG_INF("Post-event state:");
        layer_state_print_current();
    } else {
        LOG_ERR("Failed to cast layer state changed event!");
    }
    
    // Allow other listeners to process the event
    LOG_INF("Event processing complete, returning ZMK_EV_EVENT_BUBBLE");
    return ZMK_EV_EVENT_BUBBLE;
}

// Define the listener
ZMK_LISTENER(layer_state_listener, on_layer_state_changed);
ZMK_SUBSCRIPTION(layer_state_listener, zmk_layer_state_changed);