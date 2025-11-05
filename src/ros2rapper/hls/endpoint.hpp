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

struct sedp_endpoint {
    // Word 0
    //  0         alive
    //  1         builtin_pubrd_acknack_req
    //  2         builtin_subrd_acknack_req
    //  8, 9      initial_send_counter
    // 15 ... 31  udp_port
    bool        alive;
    bool        builtin_pubrd_acknack_req;
    bool        builtin_subrd_acknack_req;
    hls_uint<2> initial_send_counter;
    uint8_t     udp_port[2] /* Cyber array=EXPAND, array_index=const */;
    // Word 1
    uint8_t     builtin_pubrd_wr_seqnum;
    uint8_t     builtin_pubrd_rd_seqnum;
    uint8_t     builtin_subrd_wr_seqnum;
    uint8_t     builtin_subrd_rd_seqnum;
    // 2, 3, 4
    uint8_t     guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
    // 5
    uint8_t     ip_addr[4] /* Cyber array=EXPAND, array_index=const */;
    // 6, 7
    int64_t     builtin_pubwr_lastsn;
    // 8, 9
    int64_t     builtin_subwr_lastsn;
    // 10
    uint32_t    pub_heartbeat_cnt;
    // 11
    uint32_t    sub_heartbeat_cnt;
    // 12
    uint32_t    pub_acknack_cnt;
    // 13
    uint32_t    sub_acknack_cnt;
    // 14, 15
    int64_t     lease_duration;
    // 16, 17
    int64_t     timestamp;
    // 18, 19, 20, 21
    bool children[APP_READER_MAX] /* Cyber array=EXPAND, array_index=const */;
};

using builtin_ep_type_t = hls_uint<2>;
using app_ep_type_t = hls_uint<2>;

const builtin_ep_type_t BUILTIN_EP_PUB = 0x01; // BUILTIN_PUBLICATIONS_READER
const builtin_ep_type_t BUILTIN_EP_SUB = 0x02; // BUILTIN_SUBSCRIPTIONS_READER
const app_ep_type_t     APP_EP_PUB
    = 0x01; // Application-defined Writer (ROS2rapper is publisher)
const app_ep_type_t APP_EP_SUB
    = 0x02; // Application-defined Reader (ROS2rapper is subscriber)

struct app_endpoint {
    uint8_t       ip_addr[4] /* Cyber array=EXPAND, array_index=const */;
    uint8_t       udp_port[2] /* Cyber array=EXPAND, array_index=const */;
    uint8_t       guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
    uint8_t       entity_id[4] /* Cyber array=EXPAND, array_index=const */;
    app_ep_type_t app_ep_type;
    topic_id_t    topic_id;
    bool          alive;
};

#endif // !ENDPOINT_HPP
