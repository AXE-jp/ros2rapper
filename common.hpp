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

#ifndef COMMON_HPP
#define COMMON_HPP

#include "sedp.hpp"
#include "spdp.hpp"
#include "ip.hpp"
#include "udp.hpp"
#include "app.hpp"

#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define MIN(x,y) ((x) > (y) ? (y) : (x))

#define TARGET_CLOCK_FREQ	100000000 // Arty A7-100T: 100 MHz
#define TARGET_PARTICIPANT_ID	1

#define RAWUDP_RXBUF_LEN 256
#define RAWUDP_TXBUF_LEN 256

#define MAX_RAWUDP_OUT_PAYLOAD_LEN \
	(RAWUDP_TXBUF_LEN - 12)
#define MAX_RAWUDP_OUT_UDP_PKT_LEN \
	(UDP_HDR_SIZE + MAX_RAWUDP_OUT_PAYLOAD_LEN)
#define MAX_RAWUDP_OUT_IP_PKT_LEN \
	(IP_HDR_SIZE + MAX_RAWUDP_OUT_UDP_PKT_LEN)

#define TX_BUF_LEN (MAX(MAX(MAX(MAX(MAX(MAX_RAWUDP_OUT_IP_PKT_LEN, SPDP_WRITER_IP_PKT_LEN), SEDP_WRITER_IP_PKT_LEN), SEDP_HEARTBEAT_IP_PKT_LEN), SEDP_ACKNACK_IP_PKT_LEN), APP_WRITER_IP_PKT_LEN))

#define MAX_TX_UDP_PAYLOAD_LEN (TX_BUF_LEN - (IP_HDR_SIZE + UDP_HDR_SIZE))

#define ENTITYID_APP_WRITER	{0x00, 0x00, 0x10, 0x03}
#define ENTITYID_APP_READER	{0x00, 0x00, 0x10, 0x04}

#define SBM_ENDIAN_LITTLE
// #define SBM_ENDIAN_BIG

#ifdef SBM_ENDIAN_LITTLE
#define S_BYTE0(x)	((x) & 0xff)
#define S_BYTE1(x)	(((x) >> 8) & 0xff)
#define L_BYTE0(x)	((x) & 0xff)
#define L_BYTE1(x)	(((x) >> 8) & 0xff)
#define L_BYTE2(x)	(((x) >> 16) & 0xff)
#define L_BYTE3(x)	(((x) >> 24) & 0xff)
#endif // SBM_ENDIAN_LITTLE

#ifdef SBM_ENDIAN_BIG
#define S_BYTE0(x)	(((x) >> 8) & 0xff)
#define S_BYTE1(x)	((x) & 0xff)
#define L_BYTE0(x)	(((x) >> 24) & 0xff)
#define L_BYTE1(x)	(((x) >> 16) & 0xff)
#define L_BYTE2(x)	(((x) >> 8) & 0xff)
#define L_BYTE3(x)	((x) & 0xff)
#endif // SBM_ENDIAN_BIG

#endif // !COMMON_HPP
