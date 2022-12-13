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

#ifndef UDP_HPP
#define UDP_HPP

#include <cstdint>
#include "hls.hpp"
#include "ros2.hpp"

#define UDP_HDR_SIZE	8

#define UDP_HDR_OFFSET_SPORT	0	// Source Port
#define UDP_HDR_OFFSET_DPORT	2	// Destination Port
#define UDP_HDR_OFFSET_ULEN	4	// Length
#define UDP_HDR_OFFSET_SUM	6	// Checksum

#define PSEUDO_HDR_SIZE	12

#define PSEUDO_HDR_OFFSET_SADDR		0	// Source Address
#define PSEUDO_HDR_OFFSET_DADDR		4	// Destination Address
#define PSEUDO_HDR_OFFSET_PROTOCOL	8	// Protocol
#define PSEUDO_HDR_OFFSET_ULEN		10	// Length

#define PSEUDO_HDR_PROTOCOL	0x11 // UDP

void udp_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
	    const uint8_t cpu_udp_port[2],
	    uint32_t udp_rxbuf[UDP_RXBUF_DEPTH],
	    volatile uint8_t *udp_rxbuf_rel,
	    volatile uint8_t *udp_rxbuf_grant,
	    bool &parity_error);

void udp_out(const uint8_t src_addr[4],
	     const uint8_t src_port[2],
	     const uint8_t dst_addr[4],
	     const uint8_t dst_port[2],
	     const uint8_t udp_data[],
	     const uint16_t udp_data_process_len,
	     const uint16_t udp_data_real_len,
	     uint8_t buf[]);

#endif // !UDP_HPP
