#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

// 充电状态枚举
typedef enum {
    CHARGING_STATE_CHARGING = 0,    // 正在充电 (CHRG低电平)
    CHARGING_STATE_FULL,            // 已充满 (CHRG高电平)
    CHARGING_STATE_ERROR            // 错误状态
} charging_state_t;

// 错误类型枚举
typedef enum {
    ERROR_NONE = 0,
    ERROR_GPIO_READ,
    ERROR_GPIO_CONFIG,
    ERROR_DEVICE_NOT_READY,
    ERROR_MAX
} charging_error_t;

// 统计信息结构体
typedef struct {
    uint32_t total_state_changes;   // 总状态变化次数
    uint32_t total_errors;          // 总错误次数
    uint8_t consecutive_errors;     // 连续错误次数
    charging_state_t current_state; // 当前状态
    charging_error_t last_error;    // 最后错误类型
    int64_t last_error_time;        // 最后错误时间
    int64_t last_successful_read;   // 最后成功读取时间
    bool system_idle;               // 系统是否空闲
    bool polling_active;            // 轮询是否激活
} charging_monitor_stats;

// 充电状态变化回调函数类型
typedef void (*charging_state_changed_cb_t)(charging_state_t new_state);

// API接口
int charging_monitor_init(void);
int charging_monitor_register_callback(charging_state_changed_cb_t callback);
int charging_monitor_unregister_callback(void);
charging_state_t charging_monitor_get_state(void);
const char* charging_monitor_get_state_str(void);
bool charging_monitor_is_initialized(void);
int charging_monitor_get_stats(struct charging_monitor_stats *stats);

// 控制接口
void charging_monitor_set_polling_intervals(uint32_t charging_ms, uint32_t full_ms, uint32_t error_ms);
void charging_monitor_pause(void);
void charging_monitor_resume(void);
void charging_monitor_notify_activity(void);
void charging_monitor_force_check(void);

#ifdef __cplusplus
}
#endif