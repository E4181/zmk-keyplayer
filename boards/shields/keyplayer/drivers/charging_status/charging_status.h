/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

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
 * @brief 打印详细的充电状态信息
 */
void charging_status_log_detailed(void);

/**
 * @brief 获取原始GPIO电平值
 * 
 * @return int 0=低电平, 1=高电平, 负值=错误
 */
int charging_status_get_raw_level(void);

#ifdef __cplusplus
}
#endif