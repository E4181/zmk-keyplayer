/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zmk/events/charging_state_changed.h>

ZMK_EVENT_IMPL(charging_state_changed);