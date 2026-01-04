/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 传感器通道定义 */
enum charging_status_channel {
    /** 是否正在充电 */
    CHARGING_STATUS_CHAN_CHARGING,
    /** 充电状态改变事件 */
    CHARGING_STATUS_CHAN_CHANGE_EVENT,
};

/* 充电状态值 */
#define CHARGING_STATUS_NOT_CHARGING 0
#define CHARGING_STATUS_CHARGING     1

/**
 * @brief 获取充电状态
 * 
 * @param dev 充电状态设备
 * @return int 1=正在充电, 0=未充电, 负值=错误
 */
int charging_status_get_state(const struct device *dev);

/**
 * @brief 注册充电状态改变回调
 * 
 * @param dev 充电状态设备
 * @param callback 回调函数
 * @param user_data 用户数据
 * @return int 0=成功, 负值=错误
 */
int charging_status_register_callback(const struct device *dev,
                                     sensor_trigger_handler_t callback,
                                     void *user_data);

/**
 * @brief 检查设备是否正在充电
 * 
 * @param dev 设备
 * @return true 正在充电
 * @return false 未充电
 */
static inline bool charging_status_is_charging(const struct device *dev)
{
    return charging_status_get_state(dev) > 0;
}

#ifdef __cplusplus
}
#endif