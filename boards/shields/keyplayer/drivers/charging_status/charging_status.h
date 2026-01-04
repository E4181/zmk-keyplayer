/*
 * TP4056充电状态检测驱动头文件
 * 纯自定义实现，专注于CHRG引脚电平读取
 */

#ifndef CHARGING_STATUS_H
#define CHARGING_STATUS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 充电状态枚举 */
typedef enum {
    CHARGING_STATE_UNKNOWN = 0,     /* 状态未知 */
    CHARGING_STATE_CHARGING,        /* 正在充电 (CHRG引脚低电平) */
    CHARGING_STATE_NOT_CHARGING     /* 未充电 (CHRG引脚高电平) */
} charging_state_t;

/**
 * @brief 获取当前充电状态
 * 
 * @return charging_state_t 充电状态枚举值
 */
charging_state_t charging_status_get_state(void);

/**
 * @brief 检查是否正在充电
 * 
 * @return true 正在充电
 * @return false 未充电
 */
bool charging_status_is_charging(void);

/**
 * @brief 获取CHRG引脚原始电平
 * 
 * @return int 引脚电平值: 0=低电平, 1=高电平, 负值=错误
 */
int charging_status_get_raw_pin_state(void);

/**
 * @brief 获取充电状态字符串
 * 
 * @return const char* 状态字符串
 */
const char* charging_status_get_state_string(void);

/**
 * @brief 手动触发状态更新
 * 
 * 立即读取CHRG引脚状态并更新
 */
void charging_status_update_now(void);

/**
 * @brief 获取状态变化次数
 * 
 * @return uint32_t 自启动以来的状态变化次数
 */
uint32_t charging_status_get_change_count(void);

/**
 * @brief 获取状态历史信息
 * 
 * @param current 输出当前状态（可为NULL）
 * @param last 输出上一次状态（可为NULL）
 */
void charging_status_get_history(charging_state_t *current, 
                                charging_state_t *last);

/**
 * @brief 重新初始化充电状态监测器
 * 
 * @return int 0=成功，负值=错误码
 */
int charging_status_reinit(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGING_STATUS_H */