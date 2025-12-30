/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

struct charging_state_changed {
    bool is_charging;
    int64_t timestamp;
};

ZMK_EVENT_DECLARE(charging_state_changed);