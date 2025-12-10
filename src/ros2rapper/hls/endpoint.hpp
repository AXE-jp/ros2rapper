// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ENDPOINT_HPP
#define ENDPOINT_HPP

#include "hls.hpp"
#include <cstdint>

#define SEDP_READER_MAX 128
#define APP_READER_MAX  128

#define PUB_TOPICS_MAX 4
#define SUB_TOPICS_MAX 4

typedef hls_uint<7> sedp_reader_id_t;
typedef hls_uint<7> app_reader_id_t;
typedef hls_uint<2> topic_id_t;

// There is a variable "unused_reader_id" of type sedp_reader_id_t in
// spdp_reader. "unused_reader_id" becomes SEDP_READER_MAX when sedp_reader_tbl
// is full.
static_assert(SEDP_READER_MAX <= 255,
              "sedp_reader_id_t should be able to represent SEDP_READER_MAX.");
// There is a variable "unused_app_reader_id" of type app_reader_id_t in
// sedp_reader. "unused_app_reader_id" becomes APP_READER_MAX when
// app_reader_tbl is full.
static_assert(APP_READER_MAX <= 255,
              "app_reader_id_t should be able to represent APP_READER_MAX.");
static_assert((PUB_TOPICS_MAX <= 4) && (SUB_TOPICS_MAX <= 4),
              "topic_id_t should be able to represent PUB_TOPICS_MAX - 1 and "
              "SUB_TOPICS_MAX - 1.");

#define SEDP_ENDPOINT_ALIVE             1
#define SEDP_ENDPOINT_PUBRD_ACKNACK_REQ 2
#define SEDP_ENDPOINT_SUBRD_ACKNACK_REQ 4

// struct sedp_endpoint {
//     // Word 0, bit 0
//     bool        alive;
//     // Word 0, bit 1
//     bool        builtin_pubrd_acknack_req;
//     // Word 0, bit 2
//     bool        builtin_subrd_acknack_req;
//     // Word 0, bit 8 and 9
//     hls_uint<2> initial_send_counter;
//     // Word 0, bit 16 ... 31
//     uint8_t     udp_port[2] /* Cyber array=EXPAND, array_index=const */;
//     // Word 0, bit 32 ... 63
//     // Word 1
//     uint8_t     guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
//     // Word 2, bit 0 ... 31
//     uint8_t     ip_addr[4] /* Cyber array=EXPAND, array_index=const */;
//     // Word 2, bit 32 ... 39
//     uint8_t     builtin_pubrd_wr_seqnum;
//     // Word 2, bit 40 ... 47
//     uint8_t     builtin_pubrd_rd_seqnum;
//     // Word 2, bit 48 ... 55
//     uint8_t     builtin_subrd_wr_seqnum;
//     // Word 2, bit 56 ... 63
//     uint8_t     builtin_subrd_rd_seqnum;
//     // Word 3
//     int64_t     builtin_pubwr_lastsn;
//     // Word 4
//     int64_t     builtin_subwr_lastsn;
//     // Word 5, bit 0 ... 31
//     uint32_t    pub_heartbeat_cnt;
//     // Word 5, bit 32 ... 63
//     uint32_t    sub_heartbeat_cnt;
//     // Word 6, bit 0 ... 31
//     uint32_t    pub_acknack_cnt;
//     // Word 6, bit 32 ... 63
//     uint32_t    sub_acknack_cnt;
//     // Word 7
//     int64_t     lease_duration;
//     // Word 8
//     int64_t     timestamp;
//     // Word 9 and 10
//     bool children[APP_READER_MAX] /* Cyber array=EXPAND, array_index=const
//     */;
// };

static_assert(APP_READER_MAX == 128,
              "You have to modify\n"
              "  - sedp_reader_tbl_t (in hls/endpoint.hpp),\n"
              "  - clear_sedp_reader_tbl_children (in hls/endpoint.cpp),\n"
              "  - ros2_in_sedp_pub, ros2_in_sedp_sub and remove_sedp_endpoint "
              "(in hls/ros2.cpp),\n"
              "  - get_sedp_reader_tbl_children and "
              "set_sedp_reader_tbl_children (in test/test_utils.cpp)\n"
              "when you change APP_READER_MAX.");
typedef struct {
    uint64_t ram[11 * SEDP_READER_MAX]
#ifdef SEDP_READER_TBL_FF
    /* Cyber array=EXPAND, array_index=const */
#endif
        ;
} sedp_reader_tbl_t;

using builtin_ep_type_t = hls_uint<2>;
using app_ep_type_t = hls_uint<2>;

const builtin_ep_type_t BUILTIN_EP_PUB = 0x01; // BUILTIN_PUBLICATIONS_READER
const builtin_ep_type_t BUILTIN_EP_SUB = 0x02; // BUILTIN_SUBSCRIPTIONS_READER
const app_ep_type_t     APP_EP_PUB
    = 0x01; // Application-defined Writer (ROS2rapper is publisher)
const app_ep_type_t APP_EP_SUB
    = 0x02; // Application-defined Reader (ROS2rapper is subscriber)

struct app_endpoint {
    sedp_reader_id_t parent_id;
    uint8_t          udp_port[2] /* Cyber array=EXPAND, array_index=const */;
    uint8_t          entity_id[4] /* Cyber array=EXPAND, array_index=const */;
    app_ep_type_t    app_ep_type;
    topic_id_t       topic_id;
    bool             alive;
};

void get_sedp_reader_tbl(uint64_t *data, const sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index);
void set_sedp_reader_tbl(uint64_t data, sedp_reader_tbl_t *tbl,
                         unsigned int entry, unsigned int word_index);

// Returns liveliness (true means alive)
bool get_sedp_reader_tbl_guid_prefix(uint8_t                  guid_prefix[12],
                                     const sedp_reader_tbl_t *tbl,
                                     unsigned int             entry);
void enable_sedp_reader_tbl_flags(uint8_t flags, sedp_reader_tbl_t *tbl,
                                  unsigned int entry);

void get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
    uint8_t ip_addr[4], uint8_t *pubrd_wr_seqnum, uint8_t *pubrd_rd_seqnum,
    uint8_t *subrd_wr_seqnum, uint8_t *subrd_rd_seqnum,
    const sedp_reader_tbl_t *tbl, unsigned int entry);
void get_sedp_reader_tbl_ip_addr(uint8_t                  ip_addr[4],
                                 const sedp_reader_tbl_t *tbl,
                                 unsigned int             entry);
void set_sedp_reader_tbl_ip_addr_and_rd_seqnums(
    const uint8_t ip_addr[4], uint8_t pubrd_wr_seqnum, uint8_t pubrd_rd_seqnum,
    uint8_t subrd_wr_seqnum, uint8_t subrd_rd_seqnum, sedp_reader_tbl_t *tbl,
    unsigned int entry);

void get_sedp_reader_tbl_pubwr_lastsn(int64_t                 *pubwr_lastsn,
                                      const sedp_reader_tbl_t *tbl,
                                      unsigned int             entry);
void set_sedp_reader_tbl_pubwr_lastsn(int64_t            pubwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry);

void get_sedp_reader_tbl_subwr_lastsn(int64_t                 *subwr_lastsn,
                                      const sedp_reader_tbl_t *tbl,
                                      unsigned int             entry);
void set_sedp_reader_tbl_subwr_lastsn(int64_t            subwr_lastsn,
                                      sedp_reader_tbl_t *tbl,
                                      unsigned int       entry);

void get_sedp_reader_tbl_heartbeat_cnt(uint32_t *pub_heartbeat_cnt,
                                       uint32_t *sub_heartbeat_cnt,
                                       const sedp_reader_tbl_t *tbl,
                                       unsigned int             entry);
void set_sedp_reader_tbl_heartbeat_cnt(uint32_t           pub_heartbeat_cnt,
                                       uint32_t           sub_heartbeat_cnt,
                                       sedp_reader_tbl_t *tbl,
                                       unsigned int       entry);

void get_sedp_reader_tbl_acknack_cnt(uint32_t                *pub_acknack_cnt,
                                     uint32_t                *sub_acknack_cnt,
                                     const sedp_reader_tbl_t *tbl,
                                     unsigned int             entry);
void set_sedp_reader_tbl_acknack_cnt(uint32_t           pub_acknack_cnt,
                                     uint32_t           sub_acknack_cnt,
                                     sedp_reader_tbl_t *tbl,
                                     unsigned int       entry);

void get_sedp_reader_tbl_lease_duration(int64_t                 *lease_duration,
                                        const sedp_reader_tbl_t *tbl,
                                        unsigned int             entry);
void set_sedp_reader_tbl_lease_duration(int64_t            lease_duration,
                                        sedp_reader_tbl_t *tbl,
                                        unsigned int       entry);

void get_sedp_reader_tbl_timestamp(int64_t                 *timestamp_i64,
                                   const sedp_reader_tbl_t *tbl,
                                   unsigned int             entry);
void set_sedp_reader_tbl_timestamp(int64_t            timestamp_i64,
                                   sedp_reader_tbl_t *tbl, unsigned int entry);

void clear_sedp_reader_tbl_children(sedp_reader_tbl_t *tbl, unsigned int entry);

#endif // !ENDPOINT_HPP
