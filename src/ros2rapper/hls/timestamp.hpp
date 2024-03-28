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

#endif // !TIMESTAMP_HPP
