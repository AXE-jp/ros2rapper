// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ENDPOINT_HPP
#define ENDPOINT_HPP

#include "hls.hpp"
#include <cstdint>

#define SEDP_READER_MAX 4
#define APP_READER_MAX  4

#define SUB_TOPICS_MAX 4

typedef hls_uint<3> sedp_reader_id_t;
typedef hls_uint<3> app_reader_id_t;

struct sedp_endpoint {
    uint8_t     ip_addr[4] /* Cyber array=EXPAND, array_index=const */;
    uint8_t     udp_port[2] /* Cyber array=EXPAND, array_index=const */;
    uint8_t     guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
    uint8_t     builtin_pubrd_wr_seqnum;
    uint8_t     builtin_pubrd_rd_seqnum;
    bool        builtin_pubrd_acknack_req;
    uint8_t     builtin_subrd_wr_seqnum;
    uint8_t     builtin_subrd_rd_seqnum;
    bool        builtin_subrd_acknack_req;
    int64_t     builtin_subwr_lastsn;
    hls_uint<2> initial_send_counter;
    uint32_t    pub_heartbeat_cnt;
    uint32_t    sub_heartbeat_cnt;
    uint32_t    pub_acknack_cnt;
    uint32_t    sub_acknack_cnt;
    bool        alive;
    hls_uint<APP_READER_MAX> children;
    int64_t                  lease_duration;
    int64_t                  timestamp;
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
    bool          alive;
};

#endif // !ENDPOINT_HPP
