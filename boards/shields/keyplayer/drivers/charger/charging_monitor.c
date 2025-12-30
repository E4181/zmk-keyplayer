/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_charging_monitor

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>

#include <zmk/event_manager.h>
#include "charging_state_changed.h"
#include "charging_monitor.h"

LOG_MODULE_REGISTER(charging_monitor, CONFIG_CHARGING_MONITOR_LOG_LEVEL);

struct charging_monitor_data {
    const struct device *dev;
    struct k_work work;
    struct k_work_delayable fault_check_work;
    struct gpio_callback gpio_cb;
    enum charging_status current_status;
    enum charging_status last_status;
    enum charging_fault_type current_fault;
    struct k_timer debounce_timer;
    sys_slist_t callbacks;  // 回调链表
    struct k_spinlock callbacks_lock;  // 回调链表自旋锁
    
    // 故障检测相关
    uint32_t chrg_pulse_count;  // CHRG引脚脉冲计数
    uint32_t chrg_state_changes;  // 状态变化次数
    int64_t charging_start_time;  // 充电开始时间
    bool fault_detected;  // 故障检测标志
    uint32_t max_charging_minutes;  // 最大充电时间（分钟）
    uint32_t max_pulse_count;  // 最大允许脉冲计数
};

struct charging_monitor_config {
    struct gpio_dt_spec chrg_gpio;
    uint32_t debounce_ms;
    uint32_t fault_check_interval_ms;  // 故障检查间隔
    uint32_t max_charging_minutes;  // 最大充电时间（默认8小时）
    uint32_t max_pulse_per_minute;  // 每分钟最大脉冲数（用于故障检测）
};

// 内部状态变化检测函数
static void detect_fault_conditions(struct charging_monitor_data *data, 
                                   const struct charging_monitor_config *config,
                                   int pin_state, int64_t now);

static void notify_callbacks(struct charging_monitor_data *data,
                            enum charging_status status,
                            enum charging_fault_type fault_type);

static void charging_monitor_work_handler(struct k_work *work) {
    struct charging_monitor_data *data =
        CONTAINER_OF(work, struct charging_monitor_data, work);
    const struct device *dev = data->dev;
    const struct charging_monitor_config *config = dev->config;
    
    int pin_state = gpio_pin_get_dt(&config->chrg_gpio);
    
    if (pin_state < 0) {
        LOG_ERR("Failed to read CHRG pin: %d", pin_state);
        return;
    }
    
    int64_t now = k_uptime_get();
    
    // 故障检测
    detect_fault_conditions(data, config, pin_state, now);
    
    // 如果检测到故障，直接使用故障状态
    enum charging_status new_status;
    if (data->current_fault != CHARGING_FAULT_NONE) {
        new_status = CHARGING_STATUS_FAULT;
    } else {
        // 正常状态判断
        new_status = (pin_state == 0) ? CHARGING_STATUS_CHARGING : CHARGING_STATUS_IDLE;
    }
    
    // 状态变化处理
    if (new_status != data->current_status) {
        data->last_status = data->current_status;
        data->current_status = new_status;
        
        LOG_DBG("Charging status changed: %d -> %d (fault: %d)", 
                data->last_status, data->current_status, data->current_fault);
        
        // 通知所有注册的回调
        notify_callbacks(data, new_status, data->current_fault);
        
        // 记录充电开始时间
        if (new_status == CHARGING_STATUS_CHARGING) {
            data->charging_start_time = now;
            data->chrg_state_changes = 0;
            LOG_DBG("Charging started at %lld", now);
        } else if (new_status == CHARGING_STATUS_IDLE) {
            data->charging_start_time = 0;
            // 重置故障检测计数器
            atomic_set(&data->chrg_pulse_count, 0);
            LOG_DBG("Charging stopped or completed");
        }
        
        // 发布ZMK事件
        raise_charging_state_changed((struct charging_state_changed){
            .is_charging = (new_status == CHARGING_STATUS_CHARGING),
            .timestamp = now
        });
    }
}

// 故障检查工作处理函数
static void fault_check_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct charging_monitor_data *data =
        CONTAINER_OF(dwork, struct charging_monitor_data, fault_check_work);
    const struct device *dev = data->dev;
    const struct charging_monitor_config *config = dev->config;
    
    // 检查充电超时
    if (data->current_status == CHARGING_STATUS_CHARGING && 
        data->charging_start_time > 0) {
        int64_t now = k_uptime_get();
        int64_t charging_duration = now - data->charging_start_time;
        
        // 检查是否超过最大充电时间
        if (charging_duration > (data->max_charging_minutes * 60 * 1000)) {
            LOG_WRN("Charging timeout detected! Duration: %lld ms", charging_duration);
            data->current_fault = CHARGING_FAULT_TIMEOUT;
            k_work_submit(&data->work);
        }
    }
    
    // 重置每分钟脉冲计数
    atomic_set(&data->chrg_pulse_count, 0);
    
    // 重新调度故障检查
    k_work_reschedule(dwork, K_MSEC(config->fault_check_interval_ms));
}

// 检测故障条件
static void detect_fault_conditions(struct charging_monitor_data *data, 
                                   const struct charging_monitor_config *config,
                                   int pin_state, int64_t now) {
    
    // 统计状态变化次数（用于故障检测）
    static int last_pin_state = -1;
    if (last_pin_state != pin_state) {
        data->chrg_state_changes++;
        last_pin_state = pin_state;
        
        // TP4056故障模式检测：如果CHRG引脚快速闪烁（1Hz方波），表示故障
        // 增加脉冲计数
        atomic_inc(&data->chrg_pulse_count);
        
        // 如果脉冲计数超过阈值，检测为故障
        uint32_t pulse_count = atomic_get(&data->chrg_pulse_count);
        if (pulse_count > data->max_pulse_count) {
            LOG_WRN("Excessive CHRG pin pulses detected: %u (max: %u)", 
                   pulse_count, data->max_pulse_count);
            data->current_fault = CHARGING_FAULT_BAD_BATTERY;  // 电池异常或短路
            data->fault_detected = true;
        }
    }
    
    // 检查充电超时（仅在充电状态下）
    if (data->current_status == CHARGING_STATUS_CHARGING && 
        data->charging_start_time > 0) {
        int64_t charging_duration = now - data->charging_start_time;
        
        // 如果充电时间超过8小时（默认），认为是超时故障
        if (charging_duration > (data->max_charging_minutes * 60 * 1000)) {
            data->current_fault = CHARGING_FAULT_TIMEOUT;
            data->fault_detected = true;
        }
    }
}

// 通知所有注册的回调
static void notify_callbacks(struct charging_monitor_data *data,
                            enum charging_status status,
                            enum charging_fault_type fault_type) {
    struct charging_monitor_callback *cb;
    k_spinlock_key_t key;
    
    key = k_spin_lock(&data->callbacks_lock);
    
    // 遍历链表，调用所有回调
    SYS_SLIST_FOR_EACH_CONTAINER(&data->callbacks, cb, node) {
        if (cb->function != NULL) {
            cb->function(data->dev, status, fault_type, cb->user_data);
        }
    }
    
    k_spin_unlock(&data->callbacks_lock, key);
}

static void charging_monitor_debounce_timer_handler(struct k_timer *timer) {
    struct charging_monitor_data *data =
        CONTAINER_OF(timer, struct charging_monitor_data, debounce_timer);
    k_work_submit(&data->work);
}

static void charging_monitor_gpio_callback(const struct device *port,
                                          struct gpio_callback *cb,
                                          gpio_port_pins_t pins) {
    struct charging_monitor_data *data =
        CONTAINER_OF(cb, struct charging_monitor_data, gpio_cb);
    
    // 启动去抖动定时器
    const struct charging_monitor_config *config = data->dev->config;
    k_timer_start(&data->debounce_timer, 
                  K_MSEC(config->debounce_ms), K_NO_WAIT);
}

static int charging_monitor_init(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    const struct charging_monitor_config *config = dev->config;
    int ret;
    
    data->dev = dev;
    data->current_status = CHARGING_STATUS_IDLE;
    data->last_status = CHARGING_STATUS_IDLE;
    data->current_fault = CHARGING_FAULT_NONE;
    data->fault_detected = false;
    data->charging_start_time = 0;
    data->chrg_state_changes = 0;
    data->max_charging_minutes = config->max_charging_minutes;
    
    // 计算每分钟最大允许脉冲数
    // TP4056故障时CHRG引脚以约1Hz闪烁，每分钟约60次状态变化
    // 设置阈值为正常值的2倍，避免误报
    data->max_pulse_count = config->max_pulse_per_minute;
    
    // 初始化回调链表和锁
    sys_slist_init(&data->callbacks);
    
    if (!device_is_ready(config->chrg_gpio.port)) {
        LOG_ERR("CHRG GPIO device is not ready");
        return -ENODEV;
    }
    
    // 配置CHRG引脚
    ret = gpio_pin_configure_dt(&config->chrg_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO: %d", ret);
        return ret;
    }
    
    // 配置中断
    ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, 
                                         GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure CHRG GPIO interrupt: %d", ret);
        return ret;
    }
    
    // 初始化工作、定时器和故障检查工作
    k_work_init(&data->work, charging_monitor_work_handler);
    k_work_init_delayable(&data->fault_check_work, fault_check_work_handler);
    k_timer_init(&data->debounce_timer, 
                 charging_monitor_debounce_timer_handler, NULL);
    
    // 设置GPIO回调
    gpio_init_callback(&data->gpio_cb, charging_monitor_gpio_callback,
                       BIT(config->chrg_gpio.pin));
    
    ret = gpio_add_callback(config->chrg_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    
    // 启动故障检查工作
    k_work_reschedule(&data->fault_check_work, 
                      K_MSEC(config->fault_check_interval_ms));
    
    // 执行初始读取
    k_work_submit(&data->work);
    
    LOG_INF("Charging monitor initialized on pin %s:%d", 
            config->chrg_gpio.port->name, config->chrg_gpio.pin);
    LOG_INF("Max charging time: %u minutes, Fault check interval: %u ms",
            config->max_charging_minutes, config->fault_check_interval_ms);
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int charging_monitor_pm_action(const struct device *dev,
                                     enum pm_device_action action) {
    const struct charging_monitor_config *config = dev->config;
    struct charging_monitor_data *data = dev->data;
    int ret;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        // 禁用中断
        ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio, 
                                             GPIO_INT_DISABLE);
        if (ret < 0) {
            LOG_WRN("Failed to disable interrupt on suspend: %d", ret);
        }
        
        // 取消故障检查工作
        k_work_cancel_delayable(&data->fault_check_work);
        return 0;
        
    case PM_DEVICE_ACTION_RESUME:
        // 重新启用中断
        ret = gpio_pin_interrupt_configure_dt(&config->chrg_gpio,
                                             GPIO_INT_EDGE_BOTH);
        if (ret < 0) {
            LOG_WRN("Failed to enable interrupt on resume: %d", ret);
        }
        
        // 重新启动故障检查
        k_work_reschedule(&data->fault_check_work, 
                         K_MSEC(config->fault_check_interval_ms));
        
        // 触发重新读取
        k_work_submit(&data->work);
        return 0;
        
    default:
        return -ENOTSUP;
    }
}

#endif /* CONFIG_PM_DEVICE */

// 驱动API函数实现
static enum charging_status charging_monitor_get_status_impl(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    return data->current_status;
}

static bool charging_monitor_is_charging_impl(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    return (data->current_status == CHARGING_STATUS_CHARGING);
}

static enum charging_fault_type charging_monitor_get_fault_type_impl(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    return data->current_fault;
}

static int charging_monitor_register_callback_impl(const struct device *dev,
                                                   struct charging_monitor_callback *callback) {
    struct charging_monitor_data *data = dev->data;
    k_spinlock_key_t key;
    
    if (callback == NULL || callback->function == NULL) {
        return -EINVAL;
    }
    
    key = k_spin_lock(&data->callbacks_lock);
    sys_slist_append(&data->callbacks, &callback->node);
    k_spin_unlock(&data->callbacks_lock, key);
    
    LOG_DBG("Callback registered: %p", callback->function);
    return 0;
}

static int charging_monitor_unregister_callback_impl(const struct device *dev,
                                                     struct charging_monitor_callback *callback) {
    struct charging_monitor_data *data = dev->data;
    k_spinlock_key_t key;
    bool found = false;
    struct charging_monitor_callback *cb;
    
    if (callback == NULL) {
        return -EINVAL;
    }
    
    key = k_spin_lock(&data->callbacks_lock);
    
    // 查找并移除回调
    SYS_SLIST_FOR_EACH_CONTAINER(&data->callbacks, cb, node) {
        if (cb == callback) {
            sys_slist_remove(&data->callbacks, NULL, &cb->node);
            found = true;
            break;
        }
    }
    
    k_spin_unlock(&data->callbacks_lock, key);
    
    if (!found) {
        return -ENOENT;
    }
    
    LOG_DBG("Callback unregistered: %p", callback->function);
    return 0;
}

static int charging_monitor_clear_fault_impl(const struct device *dev) {
    struct charging_monitor_data *data = dev->data;
    
    // 清除故障状态
    data->current_fault = CHARGING_FAULT_NONE;
    data->fault_detected = false;
    
    // 重置故障检测计数器
    atomic_set(&data->chrg_pulse_count, 0);
    data->chrg_state_changes = 0;
    data->charging_start_time = 0;
    
    LOG_DBG("Fault status cleared");
    return 0;
}

// 定义驱动API结构体
static const struct charging_monitor_driver_api charging_monitor_api = {
    .get_status = charging_monitor_get_status_impl,
    .is_charging = charging_monitor_is_charging_impl,
    .get_fault_type = charging_monitor_get_fault_type_impl,
    .register_callback = charging_monitor_register_callback_impl,
    .unregister_callback = charging_monitor_unregister_callback_impl,
    .clear_fault = charging_monitor_clear_fault_impl,
};

#define CHARGING_MONITOR_INIT(n) \
    static struct charging_monitor_data charging_monitor_data_##n; \
    static const struct charging_monitor_config charging_monitor_config_##n = { \
        .chrg_gpio = GPIO_DT_SPEC_INST_GET(n, chrg_gpios), \
        .debounce_ms = DT_INST_PROP_OR(n, debounce_ms, 50), \
        .fault_check_interval_ms = DT_INST_PROP_OR(n, fault_check_interval_ms, 60000), /* 默认1分钟 */ \
        .max_charging_minutes = DT_INST_PROP_OR(n, max_charging_minutes, 480), /* 默认8小时 */ \
        .max_pulse_per_minute = DT_INST_PROP_OR(n, max_pulse_per_minute, 120), /* 默认每分钟最大120次脉冲 */ \
    }; \
    PM_DEVICE_DT_INST_DEFINE(n, charging_monitor_pm_action); \
    DEVICE_DT_INST_DEFINE(n, \
                         charging_monitor_init, \
                         PM_DEVICE_DT_INST_GET(n), \
                         &charging_monitor_data_##n, \
                         &charging_monitor_config_##n, \
                         POST_KERNEL, \
                         CONFIG_CHARGING_MONITOR_INIT_PRIORITY, \
                         &charging_monitor_api);

DT_INST_FOREACH_STATUS_OKAY(CHARGING_MONITOR_INIT)