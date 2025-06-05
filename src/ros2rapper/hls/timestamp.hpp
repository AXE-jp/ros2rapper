// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef TIMESTAMP_HPP
#define TIMESTAMP_HPP

#include <cstdint>

#define TIMESTAMP_SIZE 8

struct timestamp {
    int32_t  seconds;
    uint32_t fraction;
};

#define TIME_ZERO                                                              \
    { 0x00000000, 0x00000000 }
#define TIME_INVALID                                                           \
    { 0xffffffff, 0xffffffff }
#define TIME_INFINITE                                                          \
    { 0xffffffff, 0xfffffffe }

const timestamp DURATION_INFINITE
    = {.seconds = 0x7fffffff, .fraction = 0xffffffff};
const timestamp DEFAULT_PID_PARTICIPANT_LEASE_DURATION
    = {.seconds = 100, .fraction = 0};
const int64_t SPDP_LEASE_DURATION_DEFAULT
    = (static_cast<int64_t>(DEFAULT_PID_PARTICIPANT_LEASE_DURATION.seconds)
       << 32)
      | DEFAULT_PID_PARTICIPANT_LEASE_DURATION.fraction;

#endif // !TIMESTAMP_HPP
