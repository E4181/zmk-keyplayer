/*
 * TP4056充电状态检测驱动头文件
 * ZMK兼容接口
 */

#ifndef ZMK_CHARGING_STATUS_H
#define ZMK_CHARGING_STATUS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取当前充电状态
 * 
 * @return true 正在充电
 * @return false 未充电
 */
bool charging_status_is_charging(void);

/**
 * @brief 获取CHRG引脚原始电平状态
 * 
 * @return int 引脚电平 (0=低, 1=高)
 */
int charging_status_get_pin_state(void);

/**
 * @brief 手动刷新充电状态
 * 
 * 强制立即读取CHRG引脚状态并更新
 */
void charging_status_refresh(void);

/**
 * @brief 获取充电状态字符串
 * 
 * @return const char* "CHARGING" 或 "NOT_CHARGING"
 */
const char* charging_status_get_string(void);

/**
 * @brief 检查充电状态是否发生了变化
 * 
 * @return true 状态自上次查询后发生了变化
 * @return false 状态未变化
 */
bool charging_status_has_changed(void);

#ifdef __cplusplus
}
#endif

#endif /* ZMK_CHARGING_STATUS_H */