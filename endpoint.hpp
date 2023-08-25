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

#ifndef ENDPOINT_HPP
#define ENDPOINT_HPP

#include <cstdint>
#include "hls.hpp"

#define SEDP_READER_MAX	4

typedef hls_uint<3> sedp_reader_id_t;

struct sedp_endpoint {
	uint8_t ip_addr[4]/* Cyber array=EXPAND, array_index=const */;
	uint8_t udp_port[2]/* Cyber array=EXPAND, array_index=const */;
	uint8_t guid_prefix[12]/* Cyber array=EXPAND, array_index=const */;
};

#define APP_READER_MAX	4

typedef hls_uint<3> app_reader_id_t;

struct app_endpoint {
	uint8_t ip_addr[4]/* Cyber array=EXPAND, array_index=const */;
	uint8_t udp_port[2]/* Cyber array=EXPAND, array_index=const */;
	uint8_t guid_prefix[12]/* Cyber array=EXPAND, array_index=const */;
	uint8_t entity_id[4]/* Cyber array=EXPAND, array_index=const */;
};

#endif // !ENDPOINT_HPP
