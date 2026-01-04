/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化充电状态检测驱动
 * 
 * @return int 0=成功, 负值=错误
 */
int charging_status_init(void);

/**
 * @brief 检查是否正在充电
 * 
 * @return true 正在充电
 * @return false 未充电
 */
bool charging_status_is_charging(void);

/**
 * @brief 注册充电状态改变回调
 * 
 * @param callback 回调函数
 * @param user_data 用户数据
 * @return int 0=成功, 负值=错误
 */
int charging_status_register_callback(void (*callback)(bool is_charging, void *user_data), 
                                     void *user_data);

/**
 * @brief 获取原始 GPIO 电平值（调试用）
 * 
 * @return int 0=低电平, 1=高电平, 负值=错误
 */
int charging_status_get_raw_level(void);

/**
 * @brief 打印详细的充电状态信息
 */
void charging_status_log_detailed(void);

#ifdef __cplusplus
}
#endif