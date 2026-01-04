/*
 * TP4056充电呼吸灯控制头文件
 */

#ifndef CHARGING_BREATHE_H
#define CHARGING_BREATHE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 呼吸灯状态枚举 */
typedef enum {
    BREATHE_STATE_OFF = 0,     /* 呼吸灯关闭 */
    BREATHE_STATE_BREATHING,   /* 呼吸效果中 */
    BREATHE_STATE_ON           /* 常亮状态 */
} breathe_state_t;

/**
 * @brief 设置呼吸灯状态
 * 
 * @param enable_breathing true=启用呼吸效果，false=关闭呼吸效果
 */
void charging_breathe_set_state(bool enable_breathing);

/**
 * @brief 设置LED状态
 * 
 * @param led_on true=点亮LED，false=关闭LED
 */
void charging_breathe_set_led(bool led_on);

/**
 * @brief 获取当前呼吸灯状态
 * 
 * @return breathe_state_t 呼吸灯状态
 */
breathe_state_t charging_breathe_get_state(void);

/**
 * @brief 检查LED是否点亮
 * 
 * @return true LED点亮
 * @return false LED熄灭
 */
bool charging_breathe_is_led_on(void);

/**
 * @brief 手动触发充电状态检查
 * 
 * 立即检查充电状态并更新LED
 */
void charging_breathe_check_now(void);

/**
 * @brief 设置呼吸周期
 * 
 * @param period_ms 呼吸周期（毫秒）
 */
void charging_breathe_set_period(uint32_t period_ms);

/**
 * @brief 重新初始化呼吸灯控制
 * 
 * @return int 0=成功，负值=错误码
 */
int charging_breathe_reinit(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGING_BREATHE_H */