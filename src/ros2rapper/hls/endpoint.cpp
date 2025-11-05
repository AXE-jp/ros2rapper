// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "endpoint.hpp"
#include "hls.hpp"
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
void get_sedp_reader_tbl_rd_seqnums(uint8_t                 *pubrd_wr_seqnum,
                                    uint8_t                 *pubrd_rd_seqnum,
                                    uint8_t                 *subrd_wr_seqnum,
                                    uint8_t                 *subrd_rd_seqnum,
                                    const sedp_reader_tbl_t *tbl,
                                    unsigned int             entry) {
#pragma HLS inline
    uint32_t data;
    get_set_reader_tbl(&data, tbl, entry, 1);
    *pubrd_wr_seqnum = data & 0xff;
    *pubrd_rd_seqnum = (data >> 8) & 0xff;
    *subrd_wr_seqnum = (data >> 16) & 0xff;
    *subrd_rd_seqnum = data >> 24;
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
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        guid_prefix[j] = (word_3 >> (8 * j)) & 0xff;
        guid_prefix[j + 4] = (word_4 >> (8 * j)) & 0xff;
        guid_prefix[j + 8] = (word_5 >> (8 * j)) & 0xff;
    }
}

/* Cyber func=inline */
void set_sedp_reader_tbl_rd_seqnums(
    uint8_t pubrd_wr_seqnum, uint8_t pubrd_rd_seqnum, uint8_t subrd_wr_seqnum,
    uint8_t subrd_rd_seqnum, sedp_reader_tbl_t *tbl, unsigned int entry) {
#pragma HLS inline
    uint32_t data = pubrd_wr_seqnum | (pubrd_rd_seqnum << 8)
                    | (subrd_wr_seqnum << 16) | (subrd_rd_seqnum << 24);
    set_sedp_reader_tbl(data, tbl, entry, 1);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_ip_addr(const uint8_t      ip_addr[4],
                                 sedp_reader_tbl_t *tbl, unsigned int entry) {
#pragma HLS inline
    uint32_t data = ip_addr[0] | (ip_addr[1] << 8) | (ip_addr[2] << 16)
                    | (ip_addr[3] << 24);
    set_sedp_reader_tbl(data, tbl, entry, 2);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_guid_prefix(const uint8_t      guid_prefix[12],
                                     sedp_reader_tbl_t *tbl,
                                     unsigned int       entry) {
#pragma HLS inline
    uint32_t word_3 = 0;
    uint32_t word_4 = 0;
    uint32_t word_5 = 0;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        word_3 |= guid_prefix[j] << (8 * j);
        word_4 |= guid_prefix[j + 4] << (8 * j);
        word_5 |= guid_prefix[j + 8] << (8 * j);
    }
    set_sedp_reader_tbl(word_3, tbl, entry, 3);
    set_sedp_reader_tbl(word_4, tbl, entry, 4);
    set_sedp_reader_tbl(word_5, tbl, entry, 5);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_pubwr_lastsn(int64_t            pubwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(pubwr_lastsn & 0xffffffff, tbl, entry, 6);
    set_sedp_reader_tbl(pubwr_lastsn >> 32, tbl, entry, 7);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_subwr_lastsn(int64_t            subwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(subwr_lastsn & 0xffffffff, tbl, entry, 8);
    set_sedp_reader_tbl(subwr_lastsn >> 32, tbl, entry, 9);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_pub_heartbeat_cnt(uint32_t cnt, sedp_reader_tbl_t *tbl,
                                           unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(cnt, tbl, entry, 10);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_sub_heartbeat_cnt(uint32_t cnt, sedp_reader_tbl_t *tbl,
                                           unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(cnt, tbl, entry, 11);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_pub_acknack_cnt(uint32_t cnt, sedp_reader_tbl_t *tbl,
                                         unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(cnt, tbl, entry, 12);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_sub_acknack_cnt(uint32_t cnt, sedp_reader_tbl_t *tbl,
                                         unsigned int entry) {
    set_sedp_reader_tbl(cnt, tbl, entry, 13);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_lease_duration(int64_t            lease_duration,
                                        sedp_reader_tbl_t *tbl,
                                        unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(lease_duration & 0xffffffff, tbl, entry, 14);
    set_sedp_reader_tbl(lease_duration >> 32, tbl, entry, 15);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_timestamp(int64_t            timestamp_i64,
                                   sedp_reader_tbl_t *tbl, unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(timestamp_i64 & 0xffffffff, tbl, entry, 16);
    set_sedp_reader_tbl(timestamp_i64 >> 32, tbl, entry, 17);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_children(const bool         children[APP_READER_MAX],
                                  sedp_reader_tbl_t *tbl, unsigned int entry) {
#pragma HLS inline
    hls_uint<32> data_0;
    hls_uint<32> data_1;
    hls_uint<32> data_2;
    hls_uint<32> data_3;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 32; j++) {
#pragma HLS unroll
        data_0[j] = children[j];
        data_1[j] = children[j + 32];
        data_2[j] = children[j + 64];
        data_3[j] = children[j + 96];
    }
    set_sedp_reader_tbl(data_0, tbl, entry, 18);
    set_sedp_reader_tbl(data_1, tbl, entry, 19);
    set_sedp_reader_tbl(data_2, tbl, entry, 20);
    set_sedp_reader_tbl(data_3, tbl, entry, 21);
}

/* Cyber func=inline */
void clear_sedp_reader_tbl_children(sedp_reader_tbl *tbl, unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(0, tbl, entry, 18);
    set_sedp_reader_tbl(0, tbl, entry, 19);
    set_sedp_reader_tbl(0, tbl, entry, 20);
    set_sedp_reader_tbl(0, tbl, entry, 21);
}
