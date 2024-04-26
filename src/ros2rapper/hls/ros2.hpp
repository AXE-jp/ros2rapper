// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ROS2_HPP
#define ROS2_HPP

#include "common.hpp"
#include "hls.hpp"
#include <cstdint>

#define MAX_NODE_NAME_LEN       32
#define MAX_TOPIC_NAME_LEN      32
#define MAX_TOPIC_TYPE_NAME_LEN 64
#define MAX_APP_DATA_LEN        64

typedef struct {
    uint8_t  ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t  subnet_mask[4] /* Cyber array=EXPAND */;
    uint8_t  node_name[MAX_NODE_NAME_LEN] /* Cyber array=EXPAND */;
    uint8_t  node_name_len;
    uint8_t  node_udp_port[2] /* Cyber array=EXPAND */;
    uint8_t  rx_udp_port[2] /* Cyber array=EXPAND */;
    uint16_t port_num_seed;
    uint32_t fragment_expiration;
    uint8_t  guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
    uint8_t  pub_topic_name
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len;
    uint8_t pub_topic_type_name
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len;
    uint8_t sub_topic_name
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t sub_topic_name_len;
    uint8_t sub_topic_type_name
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t sub_topic_type_name_len;
    bool    ignore_ip_checksum;
} config_t;

#endif // !ROS2_HPP
