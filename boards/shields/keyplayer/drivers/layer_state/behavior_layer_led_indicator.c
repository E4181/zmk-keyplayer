/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_layer_led_indicator

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

// 配置结构体
struct layer_led_config {
    struct gpio_dt_spec led_gpio;
    uint8_t target_layer;
    uint8_t blink_count;
    uint32_t blink_period_ms;
    uint32_t on_duration_ms;
};

// 数据状态结构体
struct layer_led_data {
    bool is_blinking;
    uint8_t blink_counter;
    bool led_state;
    struct k_timer blink_timer;
    struct k_work_delayable blink_work;
};

// 全局设备实例指针数组
#define GET_DEV(inst) DEVICE_DT_INST_GET(inst),
static const struct device *devs[] = {DT_INST_FOREACH_STATUS_OKAY(GET_DEV)};

// 工作队列处理函数 - 控制LED状态
static void blink_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct layer_led_data *data = CONTAINER_OF(dwork, struct layer_led_data, blink_work);
    
    if (!data->is_blinking) {
        return;
    }
    
    // 切换LED状态
    for (int i = 0; i < ARRAY_SIZE(devs); i++) {
        const struct device *dev = devs[i];
        if (dev->data == data) {
            const struct layer_led_config *config = dev->config;
            
            data->led_state = !data->led_state;
            gpio_pin_set_dt(&config->led_gpio, data->led_state ? 1 : 0);
            
            // 如果LED刚关闭，增加计数器
            if (!data->led_state) {
                data->blink_counter++;
                LOG_DBG("Blink counter: %d/%d", data->blink_counter, config->blink_count);
                
                if (data->blink_counter >= config->blink_count) {
                    // 闪烁完成，停止定时器
                    data->is_blinking = false;
                    k_timer_stop(&data->blink_timer);
                    LOG_DBG("Blinking completed");
                    return;
                }
            }
            
            // 设置下一次状态切换的时间
            uint32_t delay = data->led_state ? 
                config->on_duration_ms : 
                (config->blink_period_ms - config->on_duration_ms);
            
            k_work_schedule(dwork, K_MSEC(delay));
            break;
        }
    }
}

// 定时器回调函数
static void blink_timer_handler(struct k_timer *timer) {
    struct layer_led_data *data = CONTAINER_OF(timer, struct layer_led_data, blink_timer);
    k_work_schedule(&data->blink_work, K_NO_WAIT);
}

// 开始闪烁序列
static void start_blinking(const struct device *dev) {
    struct layer_led_data *data = dev->data;
    const struct layer_led_config *config = dev->config;
    
    if (data->is_blinking) {
        // 如果已经在闪烁，先停止
        k_timer_stop(&data->blink_timer);
        k_work_cancel_delayable(&data->blink_work);
    }
    
    data->is_blinking = true;
    data->blink_counter = 0;
    data->led_state = true; // 从亮开始
    
    // 确保LED初始状态为关闭
    gpio_pin_set_dt(&config->led_gpio, 0);
    
    // 启动第一次闪烁
    k_work_schedule(&data->blink_work, K_NO_WAIT);
    
    LOG_DBG("Started blinking for layer %d", config->target_layer);
}

// 事件监听器 - 监听层级变化
static int layer_led_indicator_layer_state_changed_listener(const zmk_event_t *eh);

ZMK_LISTENER(layer_led_indicator, layer_led_indicator_layer_state_changed_listener);
ZMK_SUBSCRIPTION(layer_led_indicator, zmk_layer_state_changed);

static int layer_led_indicator_layer_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    LOG_DBG("Layer change event: layer=%d, state=%d", ev->layer, ev->state);
    
    // 只处理激活状态
    if (ev->state) {
        for (int i = 0; i < ARRAY_SIZE(devs); i++) {
            const struct device *dev = devs[i];
            const struct layer_led_config *config = dev->config;
            
            if (ev->layer == config->target_layer) {
                LOG_DBG("Target layer %d activated, triggering LED", config->target_layer);
                start_blinking(dev);
                break;
            }
        }
    }
    
    return ZMK_EV_EVENT_BUBBLE;
}

// 行为接口函数
static int on_layer_led_binding_pressed(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    // 这个行为是自动触发的，不需要按键操作
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_layer_led_binding_released(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_layer_led_driver_api = {
    .binding_pressed = on_layer_led_binding_pressed,
    .binding_released = on_layer_led_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = NULL, // 暂时不需要复杂参数
#endif
};

// 设备实例定义宏
#define LAYER_LED_INDICATOR_INST(inst)                                         \
    static struct layer_led_data layer_led_data_##inst = {                     \
        .is_blinking = false,                                                  \
        .blink_counter = 0,                                                    \
        .led_state = false,                                                    \
    };                                                                         \
    static const struct layer_led_config layer_led_config_##inst = {           \
        .led_gpio = GPIO_DT_SPEC_INST_GET(inst, led_gpios),                    \
        .target_layer = DT_INST_PROP(inst, target_layer),                      \
        .blink_count = DT_INST_PROP_OR(inst, blink_count, 5),                  \
        .blink_period_ms = DT_INST_PROP_OR(inst, blink_period_ms, 1000),       \
        .on_duration_ms = DT_INST_PROP_OR(inst, on_duration_ms, 100),          \
    };                                                                         \
    static int layer_led_init_##inst(const struct device *dev) {               \
        struct layer_led_data *data = dev->data;                               \
        const struct layer_led_config *config = dev->config;                   \
                                                                               \
        /* 初始化GPIO */                                                       \
        if (!device_is_ready(config->led_gpio.port)) {                         \
            LOG_ERR("LED GPIO device not ready");                              \
            return -ENODEV;                                                    \
        }                                                                      \
                                                                               \
        int ret = gpio_pin_configure_dt(&config->led_gpio, GPIO_OUTPUT_INACTIVE); \
        if (ret < 0) {                                                         \
            LOG_ERR("Failed to configure LED GPIO (err %d)", ret);             \
            return ret;                                                        \
        }                                                                      \
                                                                               \
        /* 初始状态关闭 */                                                     \
        gpio_pin_set_dt(&config->led_gpio, 0);                                 \
                                                                               \
        /* 初始化定时器和工作队列 */                                           \
        k_timer_init(&data->blink_timer, blink_timer_handler, NULL);           \
        k_work_init_delayable(&data->blink_work, blink_work_handler);          \
                                                                               \
        LOG_DBG("Layer LED indicator initialized for layer %d",                \
                config->target_layer);                                         \
        return 0;                                                              \
    }                                                                          \
    BEHAVIOR_DT_INST_DEFINE(inst, layer_led_init_##inst, NULL,                 \
                          &layer_led_data_##inst, &layer_led_config_##inst,    \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                          &behavior_layer_led_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LAYER_LED_INDICATOR_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */