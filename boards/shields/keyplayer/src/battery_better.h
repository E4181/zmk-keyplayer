#ifndef BATTERY_BETTER_H
#define BATTERY_BETTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取优化后的电池电压
 * @return 电池电压（毫伏）
 */
uint16_t battery_better_get_voltage(void);

/**
 * @brief 获取优化后的电池电量百分比
 * @return 电量百分比（0-100）
 */
uint8_t battery_better_get_percentage(void);

/**
 * @brief 获取电池状态字符串
 * @return 状态字符串
 */
const char* battery_better_get_status_string(void);

/**
 * @brief 初始化优化电池监测系统
 * @return 0表示成功，负数表示错误
 */
int battery_better_init(void);

/**
 * @brief 电池校准
 * @param measured_voltage 实际测量的电池电压（毫伏）
 * @return 0表示成功，负数表示错误
 */
int battery_better_calibrate(uint16_t measured_voltage);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_BETTER_H */