// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ROS2_HPP
#define ROS2_HPP

#define MAX_NODE_NAME_LEN       32
#define MAX_TOPIC_NAME_LEN      32
#define MAX_TOPIC_TYPE_NAME_LEN 64
#define MAX_APP_DATA_LEN        64

#include "common.hpp"
#include "hls.hpp"
#include <cstdint>

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
    uint8_t  pub_topic_name_0
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len_0;
    uint8_t pub_topic_type_name_0
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len_0;
    uint8_t pub_topic_name_1
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len_1;
    uint8_t pub_topic_type_name_1
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len_1;
    uint8_t pub_topic_name_2
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len_2;
    uint8_t pub_topic_type_name_2
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len_2;
    uint8_t pub_topic_name_3
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len_3;
    uint8_t pub_topic_type_name_3
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len_3;
    uint8_t sub_topic_name
        [SUB_TOPICS_MAX]
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t sub_topic_name_len
        [SUB_TOPICS_MAX] /* Cyber array=EXPAND, array_index=const */;
    uint8_t sub_topic_type_name
        [SUB_TOPICS_MAX]
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t sub_topic_type_name_len
        [SUB_TOPICS_MAX] /* Cyber array=EXPAND, array_index=const */;
    bool ignore_ip_checksum;
} config_t;

#endif // !ROS2_HPP
