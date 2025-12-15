#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

// 充电状态枚举 - 使用原子操作
typedef enum {
    CHARGING_STATE_CHARGING = 0,    // 正在充电 (CHRG低电平)
    CHARGING_STATE_FULL,            // 已充满 (CHRG高电平)
    CHARGING_STATE_ERROR            // 错误状态
} charging_state_t;

// 充电状态变化回调函数类型
typedef void (*charging_state_changed_cb_t)(charging_state_t new_state);

// API接口
int charging_monitor_init(void);
int charging_monitor_register_callback(charging_state_changed_cb_t callback);
int charging_monitor_unregister_callback(void);
charging_state_t charging_monitor_get_state(void);
const char* charging_monitor_get_state_str(void);
bool charging_monitor_is_initialized(void);
void charging_monitor_set_polling_interval(uint32_t interval_ms);
void charging_monitor_pause(void);
void charging_monitor_resume(void);

#ifdef __cplusplus
}
#endif