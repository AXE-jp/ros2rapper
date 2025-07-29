// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "util.hpp"
#include "ros2.hpp"

/* Cyber func=inline */
void clear_txbuf(uint8_t buf[], int start_incl, int end_excl) {
#pragma HLS inline
#ifdef PUB_DATA_FF
    /* Cyber unroll_times=all */
#endif // PUB_DATA_FF
    for (auto j = start_incl; j < end_excl; j++) {
#ifdef PUB_DATA_FF
#pragma HLS unroll
#endif // PUB_DATA_FF
#ifdef PUB_DATA_RAM
#pragma HLS unroll factor = 2
#endif // PUB_DATA_RAM
        buf[j] = 0;
    }
}
