// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ros2.hpp"
#include <cstdint>

void setup_topic_data(int id, uint8_t topic_name[][MAX_TOPIC_NAME_LEN],
                      uint8_t topic_name_len[],
                      uint8_t type_name[][MAX_TOPIC_TYPE_NAME_LEN],
                      uint8_t type_name_len[], const uint8_t topic_name_0[],
                      uint8_t topic_name_len_0, const uint8_t type_name_0[],
                      uint8_t type_name_len_0);
