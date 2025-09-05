// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DURATION_HPP
#define DURATION_HPP

#include "timestamp.hpp"
#include <cstdint>

#define DURATION_SIZE TIMESTAMP_SIZE

typedef timestamp duration;

#define DURATION_ZERO                                                          \
    { 0x00000000, 0x00000000 }

const duration DURATION_INFINITE
    = {.seconds = 0x7fffffff, .fraction = 0xffffffff};
const duration DEFAULT_PARTICIPANT_LEASE_DURATION
    = {.seconds = 100, .fraction = 0};
const int64_t SPDP_LEASE_DURATION_DEFAULT
    = (static_cast<int64_t>(DEFAULT_PARTICIPANT_LEASE_DURATION.seconds) << 32)
      | DEFAULT_PARTICIPANT_LEASE_DURATION.fraction;

#endif // !DURATION_HPP
