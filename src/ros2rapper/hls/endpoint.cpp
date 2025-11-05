// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "endpoint.hpp"
#include <cstdint>

/* Cyber func=inline */
void get_sedp_reader_tbl(uint32_t *data, const sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index) {
#pragma HLS inline
    *data = tbl->ram[22 * entry + word_index];
}

/* Cyber func=inline */
void set_sedp_reader_tbl(uint32_t data, sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index) {
#pragma HLS inline
    tbl->ram[22 * entry + word_index] = data;
}

/* Cyber func=inline */
void get_sedp_reader_tbl_alive(bool *alive, const sedp_reader_tbl_t *tbl,
                               unsigned int entry) {
#pragma HLS inline
    uint32_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 0);
    *alive = ((data & 0) != 0);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_guid_prefix(uint8_t                  guid_prefix[12],
                                     const sedp_reader_tbl_t *tbl,
                                     unsigned int             entry) {
#pragma HLS inline
    uint32_t word_3, word_4, word_5;
    get_sedp_reader_tbl(&word_3, tbl, entry, 3);
    get_sedp_reader_tbl(&word_4, tbl, entry, 4);
    get_sedp_reader_tbl(&word_5, tbl, entry, 5);
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 3; j++) {
#pragma HLS unroll
        guid_prefix[j] = (word_3 >> (8 * j)) & 0xff;
        guid_prefix[j + 4] = (word_4 >> (8 * j)) & 0xff;
        guid_prefix[j + 8] = (word_5 >> (8 * j)) & 0xff;
    }
}
