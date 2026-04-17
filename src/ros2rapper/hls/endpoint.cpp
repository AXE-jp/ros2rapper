// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "endpoint.hpp"
#include "hls.hpp"
#include <cstdint>

/* Cyber func=inline */
void get_app_reader_tbl(app_endpoint *reader, const app_reader_tbl_t *tbl,
                        unsigned int entry) {
#pragma HLS inline
    uint64_t data = tbl->ram[entry];
    reader->alive = ((data & 1) != 0);
    reader->app_ep_type = data >> 1;
    reader->topic_id = data >> 3;
    reader->parent_id = data >> 8;
    reader->udp_port[0] = data >> 16;
    reader->udp_port[1] = data >> 24;
    reader->entity_id[0] = data >> 32;
    reader->entity_id[1] = data >> 40;
    reader->entity_id[2] = data >> 48;
    reader->entity_id[3] = data >> 56;
}

/* Cyber func=inline */
void set_app_reader_tbl(const app_endpoint &reader, app_reader_tbl_t *tbl,
                        unsigned int entry) {
#pragma HLS inline
    uint64_t data = (reader.alive ? 1 : 0);
    data |= static_cast<uint64_t>(reader.app_ep_type) << 1;
    data |= static_cast<uint64_t>(reader.topic_id) << 3;
    data |= static_cast<uint64_t>(reader.parent_id) << 8;
    data |= static_cast<uint64_t>(reader.udp_port[0]) << 16;
    data |= static_cast<uint64_t>(reader.udp_port[1]) << 24;
    data |= static_cast<uint64_t>(reader.entity_id[0]) << 32;
    data |= static_cast<uint64_t>(reader.entity_id[1]) << 40;
    data |= static_cast<uint64_t>(reader.entity_id[2]) << 48;
    data |= static_cast<uint64_t>(reader.entity_id[3]) << 56;
    tbl->ram[entry] = data;
}

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
bool get_sedp_reader_tbl_guid_prefix(uint8_t                  guid_prefix[12],
                                     const sedp_reader_tbl_t *tbl,
                                     unsigned int             entry) {
#pragma HLS inline
    uint64_t data_0, data_1;
    get_sedp_reader_tbl(&data_0, tbl, entry, 0);
    get_sedp_reader_tbl(&data_1, tbl, entry, 1);

    /* Cyber unroll_times=all */
    for (auto k = 0; k < 4; k++) {
#pragma HLS unroll
        guid_prefix[k] = (data_0 >> (8 * (k + 4)));
    }
    /* Cyber unroll_times=all */
    for (auto k = 0; k < 8; k++) {
#pragma HLS unroll
        guid_prefix[k + 4] = (data_1 >> (8 * k));
    }

    return ((data_0 & SEDP_ENDPOINT_ALIVE) != 0);
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
void get_sedp_reader_tbl_ip_addr(uint8_t                  ip_addr[4],
                                 const sedp_reader_tbl_t *tbl,
                                 unsigned int             entry) {
#pragma HLS inline
    uint8_t sn_0, sn_1, sn_2, sn_3;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, &sn_0, &sn_1, &sn_2,
                                               &sn_3, tbl, entry);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_rd_seqnums(uint8_t                 *pubrd_wr_seqnum,
                                    uint8_t                 *pubrd_rd_seqnum,
                                    uint8_t                 *subrd_wr_seqnum,
                                    uint8_t                 *subrd_rd_seqnum,
                                    const sedp_reader_tbl_t *tbl,
                                    unsigned int             entry) {
#pragma HLS inline
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, pubrd_wr_seqnum,
                                               pubrd_rd_seqnum, subrd_wr_seqnum,
                                               subrd_rd_seqnum, tbl, entry);
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
void get_sedp_reader_tbl_pubwr_lastsn(int64_t                 *pubwr_lastsn,
                                      const sedp_reader_tbl_t *tbl,
                                      unsigned int             entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 3);
    *pubwr_lastsn = data;
}

/* Cyber func=inline */
void set_sedp_reader_tbl_pubwr_lastsn(int64_t            pubwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(pubwr_lastsn, tbl, entry, 3);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_subwr_lastsn(int64_t                 *subwr_lastsn,
                                      const sedp_reader_tbl_t *tbl,
                                      unsigned int             entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 4);
    *subwr_lastsn = data;
}

/* Cyber func=inline */
void set_sedp_reader_tbl_subwr_lastsn(int64_t            subwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(subwr_lastsn, tbl, entry, 4);
}

/* Cyber func=inline */
void get_sedp_reader_tbl_heartbeat_cnt(uint32_t *pub_heartbeat_cnt,
                                       uint32_t *sub_heartbeat_cnt,
                                       const sedp_reader_tbl_t *tbl,
                                       unsigned int             entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 5);
    *pub_heartbeat_cnt = data & 0xffffffff;
    *sub_heartbeat_cnt = data >> 32;
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
void get_sedp_reader_tbl_acknack_cnt(uint32_t                *pub_acknack_cnt,
                                     uint32_t                *sub_acknack_cnt,
                                     const sedp_reader_tbl_t *tbl,
                                     unsigned int             entry) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, entry, 6);
    *pub_acknack_cnt = data & 0xffffffff;
    *sub_acknack_cnt = data >> 32;
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
void clear_sedp_reader_tbl_children(sedp_reader_tbl_t *tbl,
                                    unsigned int       entry) {
#pragma HLS inline
    set_sedp_reader_tbl(0, tbl, entry, 9);
    set_sedp_reader_tbl(0, tbl, entry, 10);
}
