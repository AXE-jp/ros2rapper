// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ROS2_HPP
#define ROS2_HPP

#include "common.hpp"
#include "hls.hpp"
#include <cstdint>

#define MAX_NODE_NAME_LEN 32
#define MAX_TOPIC_NAME_LEN 32
#define MAX_TOPIC_TYPE_NAME_LEN 64
#define MAX_APP_DATA_LEN 64

typedef struct {
  uint8_t ip_addr[4] /* Cyber array=EXPAND */;
  uint8_t node_name[MAX_NODE_NAME_LEN] /* Cyber array=EXPAND */;
  uint8_t node_name_len;
  uint8_t node_udp_port[2] /* Cyber array=EXPAND */;
  uint8_t cpu_udp_port[2] /* Cyber array=EXPAND */;
  uint16_t port_num_seed;
  uint32_t tx_period;
  uint32_t fragment_expiration;
  uint8_t guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
  uint8_t pub_topic_name
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
} config_t;

void ros2(hls_stream<uint8_t> &in, hls_stream<uint8_t> &out,
          uint32_t udp_rxbuf[], uint32_t udp_txbuf[], hls_uint<1> pub_enable,
          hls_uint<1> sub_enable, const config_t *conf,
          volatile const uint8_t app_data[MAX_APP_DATA_LEN],
          volatile const uint8_t app_data_len,
          volatile const uint8_t app_rx_data[MAX_APP_DATA_LEN],
          volatile const uint8_t app_rx_data_len,
          volatile uint8_t *app_rx_data_rel,
          volatile uint8_t *app_rx_data_grant, volatile uint8_t *app_data_req,
          volatile uint8_t *app_data_rel, volatile uint8_t *app_data_grant,
          volatile uint8_t *udp_rxbuf_rel, volatile uint8_t *udp_rxbuf_grant,
          volatile uint8_t *udp_txbuf_rel, volatile uint8_t *udp_txbuf_grant);

#endif // !ROS2_HPP
