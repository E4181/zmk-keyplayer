/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

#ifdef __cplusplus
extern "C" {
#endif

enum charger_state {
    CHARGER_STATE_UNKNOWN,
    CHARGER_STATE_NOT_CHARGING,
    CHARGER_STATE_CHARGING,
};

struct zmk_charger_state_changed {
    enum charger_state state;
};

ZMK_EVENT_DECLARE(zmk_charger_state_changed);

static inline struct zmk_charger_state_changed *
as_zmk_charger_state_changed(const zmk_event_t *eh) {
    return as_zmk_event(eh)->data;
}

/**
 * @brief Get the current charger state
 *
 * @return enum charger_state Current charger state
 */
enum charger_state zmk_charger_get_state(void);

/**
 * @brief Check if device is currently charging
 *
 * @return true if charging
 * @return false if not charging
 */
bool zmk_charger_is_charging(void);

#ifdef __cplusplus
}
#endif