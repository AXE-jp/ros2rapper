/*
    Copyright © 2021-2022 AXE, Inc. All Rights Reserved.

    This file is part of ROS2rapper.

    ROS2rapper is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ROS2rapper is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with ROS2rapper.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef ROS2_HPP
#define ROS2_HPP

#include <cstdint>
#include "hls.hpp"
#include "common.hpp"

#define MAX_NODE_NAME_LEN        32
#define MAX_TOPIC_NAME_LEN       32
#define MAX_TOPIC_TYPE_NAME_LEN  64
#define MAX_APP_DATA_LEN         64

typedef struct {
		uint8_t ip_addr[4];
		uint8_t node_name[MAX_NODE_NAME_LEN];
		uint8_t node_name_len;
		uint8_t node_udp_port[2];
		uint8_t cpu_udp_port[2];
		uint16_t port_num_seed;
		uint32_t tx_period;
		uint8_t guid_prefix[12];
		uint8_t topic_name[MAX_TOPIC_NAME_LEN];
		uint8_t topic_name_len;
		uint8_t topic_type_name[MAX_TOPIC_TYPE_NAME_LEN];
		uint8_t topic_type_name_len;
		uint8_t app_data[MAX_APP_DATA_LEN];
		uint8_t app_data_len;
} config_t;

#define CTRL_ENABLE 0x1

void ros2(
		hls_stream<uint8_t> &in,
		hls_stream<uint8_t> &out,
		config_t &conf,
		volatile uint8_t *app_data_req,
	  volatile uint8_t *app_data_rel,
	  volatile uint8_t *app_data_grant,
	  volatile uint8_t *udp_rxbuf_rel,
	  volatile uint8_t *udp_rxbuf_grant
);

#endif // !ROS2_HPP
