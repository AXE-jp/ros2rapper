// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ROS2_HPP
#define ROS2_HPP

#include "duration.hpp"
#include "hls.hpp"
#include <cstdint>

#define MAX_NODE_NAME_LEN       32
#define MAX_TOPIC_NAME_LEN      32
#define MAX_TOPIC_TYPE_NAME_LEN 64
#define MAX_APP_DATA_LEN        1024

typedef hls_uint<11> app_data_len_t;
static_assert(MAX_APP_DATA_LEN <= 2047,
              "app_data_len_t should be able to represent MAX_APP_DATA_LEN.");

// #define PUB_DATA_FF
#define PUB_DATA_RAM

// #define SEDP_READER_TBL_FF
#define SEDP_READER_TBL_RAM

// #define APP_READER_TBL_FF
#define APP_READER_TBL_RAM

#include "common.hpp"

typedef struct {
    uint8_t  ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t  subnet_mask[4] /* Cyber array=EXPAND */;
    uint16_t port_num_seed;
    uint32_t fragment_expiration;
    uint8_t  guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
    uint8_t  pub_topic_name
        [PUB_TOPICS_MAX]
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len
        [PUB_TOPICS_MAX] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name
        [PUB_TOPICS_MAX]
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len
        [PUB_TOPICS_MAX] /* Cyber array=EXPAND, array_index=const */;
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
} receiver_config_t;

typedef struct {
    uint16_t port_num_seed;
    duration participant_lease_duration;
} config_t;

typedef struct {
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t vendor_id[2] /* Cyber array=EXPAND */;
    uint8_t node_name[MAX_NODE_NAME_LEN] /* Cyber array=EXPAND */;
    uint8_t node_name_len;
    uint8_t node_udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
    uint8_t pub_topic_name
        [PUB_TOPICS_MAX]
        [MAX_TOPIC_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_name_len
        [PUB_TOPICS_MAX] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name
        [PUB_TOPICS_MAX]
        [MAX_TOPIC_TYPE_NAME_LEN] /* Cyber array=EXPAND, array_index=const */;
    uint8_t pub_topic_type_name_len
        [PUB_TOPICS_MAX] /* Cyber array=EXPAND, array_index=const */;
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
} sender_config_t;

#include "ros2_receiver.hpp"

void ros2_in(hls_stream<rtps_data_t> &in, sedp_reader_tbl_t *sedp_reader_tbl,
             app_reader_tbl_t        *app_reader_tbl,
             hls_uint<PUB_TOPICS_MAX> pub_enable,
             hls_uint<SUB_TOPICS_MAX> sub_enable, int64_t timestamp_i64,
             sedp_reader_id_t *sedp_reader_cnt,
             app_reader_id_t  *app_reader_cnt);

#endif // !ROS2_HPP
