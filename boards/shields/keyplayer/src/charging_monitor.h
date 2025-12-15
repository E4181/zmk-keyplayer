#pragma once

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

// 充电状态枚举（简化）
enum charging_state {
    CHARGING_STATE_CHARGING = 0,    // 正在充电 (CHRG低电平)
    CHARGING_STATE_FULL,            // 已充满 (CHRG高电平)
    CHARGING_STATE_ERROR            // 错误状态
};

// 充电状态变化回调函数类型
typedef void (*charging_state_changed_cb_t)(enum charging_state new_state);

// 初始化充电监控器
int charging_monitor_init(void);

// 注册回调函数（只有一个回调足够了）
int charging_monitor_register_callback(charging_state_changed_cb_t callback);

// 获取当前充电状态
enum charging_state charging_monitor_get_state(void);

// 获取充电状态字符串
const char* charging_monitor_get_state_str(void);

#ifdef __cplusplus
}
#endif
