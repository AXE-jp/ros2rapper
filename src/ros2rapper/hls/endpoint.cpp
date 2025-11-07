// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "endpoint.hpp"
#include "hls.hpp"
#include <cstdint>

/* Cyber func=inline */
void get_sedp_reader_tbl(uint64_t *data, const sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index) {
#pragma HLS inline
    *data = tbl->ram[11 * entry + word_index];
}

/* Cyber func=inline */
void set_sedp_reader_tbl(uint64_t data, sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index) {
#pragma HLS inline
    tbl->ram[11 * entry + word_index] = data;
}

/* Cyber func=inline */
void enable_sedp_reader_tbl_flags(uint8_t flags, sedp_reader_tbl_t *tbl,
                                  unsigned int entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 0);
    set_sedp_reader_tbl(data | flags, tbl, entry, 0);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
    uint8_t ip_addr[4], uint8_t *pubrd_wr_seqnum, uint8_t *pubrd_rd_seqnum,
    uint8_t *subrd_wr_seqnum, uint8_t *subrd_rd_seqnum,
    const sedp_reader_tbl_t *tbl, unsigned int entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 2);
    ip_addr[0] = data & 0xff;
    ip_addr[1] = (data >> 8) & 0xff;
    ip_addr[2] = (data >> 16) & 0xff;
    ip_addr[3] = (data >> 24) & 0xff;
    *pubrd_wr_seqnum = (data >> 32) & 0xff;
    *pubrd_rd_seqnum = (data >> 40) & 0xff;
    *subrd_wr_seqnum = (data >> 48) & 0xff;
    *subrd_rd_seqnum = (data >> 56) & 0xff;
}

/* Cyber func=inline */
void set_sedp_reader_tbl_ip_addr_and_rd_seqnums(
    const uint8_t ip_addr[4], uint8_t pubrd_wr_seqnum, uint8_t pubrd_rd_seqnum,
    uint8_t subrd_wr_seqnum, uint8_t subrd_rd_seqnum, sedp_reader_tbl_t *tbl,
    unsigned int entry) {
#pragma HLS inline
    uint64_t data = ip_addr[0];
    data |= static_cast<uint64_t>(ip_addr[1]) << 8;
    data |= static_cast<uint64_t>(ip_addr[2]) << 16;
    data |= static_cast<uint64_t>(ip_addr[3]) << 24;
    data |= static_cast<uint64_t>(pubrd_wr_seqnum) << 32;
    data |= static_cast<uint64_t>(pubrd_rd_seqnum) << 40;
    data |= static_cast<uint64_t>(subrd_wr_seqnum) << 48;
    data |= static_cast<uint64_t>(subrd_rd_seqnum) << 56;
    set_sedp_reader_tbl(data, tbl, entry, 2);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_pubwr_lastsn(int64_t            pubwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(pubwr_lastsn, tbl, entry, 3);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_subwr_lastsn(int64_t            subwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(subwr_lastsn, tbl, entry, 4);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_heartbeat_cnt(uint32_t           pub_heartbeat_cnt,
                                       uint32_t           sub_heartbeat_cnt,
                                       sedp_reader_tbl_t *tbl,
                                       unsigned int       entry) {
#pragma HLS inline
    uint64_t data
        = pub_heartbeat_cnt | (static_cast<uint64_t>(sub_heartbeat_cnt) << 32);
    set_sedp_reader_tbl(data, tbl, entry, 5);
}

/* Cyber func=inline */
void set_sedp_reader_tbl_acknack_cnt(uint32_t           pub_acknack_cnt,
                                     uint32_t           sub_acknack_cnt,
                                     sedp_reader_tbl_t *tbl,
                                     unsigned int       entry) {
#pragma HLS inline
    uint64_t data
        = pub_acknack_cnt | (static_cast<uint64_t>(sub_acknack_cnt) << 32);
    set_sedp_reader_tbl(data, tbl, entry, 6);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_lease_duration(int64_t                 *lease_duration,
                                        const sedp_reader_tbl_t *tbl,
                                        unsigned int             entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 7);
    *lease_duration = data;
}

/* Cyber func=inline */
void set_sedp_reader_tbl_lease_duration(int64_t            lease_duration,
                                        sedp_reader_tbl_t *tbl,
                                        unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(lease_duration, tbl, entry, 7);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_timestamp(int64_t                 *timestamp_i64,
                                   const sedp_reader_tbl_t *tbl,
                                   unsigned int             entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 8);
    *timestamp_i64 = data;
}

/* Cyber func=inline */
void set_sedp_reader_tbl_timestamp(int64_t            timestamp_i64,
                                   sedp_reader_tbl_t *tbl, unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(timestamp_i64, tbl, entry, 8);
}

/* Cyber func=inline */
void clear_sedp_reader_tbl_children(sedp_reader_tbl *tbl, unsigned int entry) {
#pragma HLS inline
    set_sedp_reader_tbl(0, tbl, entry, 9);
    set_sedp_reader_tbl(0, tbl, entry, 10);
}
