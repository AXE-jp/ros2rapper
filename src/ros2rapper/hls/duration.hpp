// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DURATION_HPP
#define DURATION_HPP

#include "timestamp.hpp"
#include <cstdint>

#define DURATION_SIZE TIMESTAMP_SIZE

typedef timestamp duration;

#define DURATION_ZERO                                                          \
    { 0x00000000, 0x00000000 }
#define DURATION_INFINITE                                                      \
    { 0x7fffffff, 0xffffffff }
#define DEFAULT_PARTICIPANT_LEASE_DURATION                                     \
    { 100, 0 }
const int64_t SPDP_LEASE_DURATION_DEFAULT = (static_cast<int64_t>(100) << 32);

#endif // !DURATION_HPP
