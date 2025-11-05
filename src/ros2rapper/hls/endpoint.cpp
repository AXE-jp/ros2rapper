// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "endpoint.hpp"
#include <cstdint>

/* Cyber func=inline */
uint32_t get_sedp_reader_tbl(const sedp_reader_tbl_t *tbl, unsigned int entry,
                             unsigned int word_index) {
#pragma HLS inline
    return tbl->ram[22 * entry + word_index];
}

/* Cyber func=inline */
void set_sedp_reader_tbl(uint32_t data, sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index) {
#pragma HLS inline
    tbl->ram[22 * entry + word_index] = data;
}
