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

#ifndef RTPS_HPP
#define RTPS_HPP

#include <cstdint>
#include "hls.hpp"

#define RTPS_HDR_SIZE	20

#define RTPS_HDR_OFFSET_PROTOCOL_ID		0
#define RTPS_HDR_OFFSET_PROTOCOL_VERSION	4
#define RTPS_HDR_OFFSET_VENDOR_ID		6
#define RTPS_HDR_OFFSET_GUID_PREFIX		8

#define RTPS_HDR_PROTOCOL_VERSION	0x0203	// 2.3
#define RTPS_HDR_VENDOR_ID		0x010f	// 01.15

#define SBM_HDR_SIZE	4

#define SBM_HDR_OFFSET_SUBMESSAGE_ID		0
#define SBM_HDR_OFFSET_FLAGS			1
#define SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER	2

#define SBM_ID_ACKNACK		0x06
#define SBM_ID_HEARTBEAT	0x07
#define SBM_ID_INFO_TS		0x09
#define SBM_ID_INFO_DST		0x0e
#define SBM_ID_DATA		0x15

#define SBM_FLAGS_ENDIANNESS	0x01
#define SBM_FLAGS_DATA		0x04

#define SBM_DATA_HDR_SIZE	20

#define SBM_DATA_HDR_OFFSET_EXTRA_FLAGS			0
#define SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS	2
#define SBM_DATA_HDR_OFFSET_READER_ID			4
#define SBM_DATA_HDR_OFFSET_WRITER_ID			8
#define SBM_DATA_HDR_OFFSET_WRITER_SN			12

#define SBM_DATA_HDR_OCTETS_TO_INLINE_QOS	0x0010 // 16

#define SBM_HEARTBEAT_DATA_SIZE	28

#define SBM_HEARTBEAT_DATA_OFFSET_READER_ID	0
#define SBM_HEARTBEAT_DATA_OFFSET_WRITER_ID	4
#define SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN	8
#define SBM_HEARTBEAT_DATA_OFFSET_LAST_SN	16
#define SBM_HEARTBEAT_DATA_OFFSET_COUNT		24

#define SBM_ACKNACK_DATA_SIZE		28

#define SBM_ACKNACK_DATA_OFFSET_READER_ID	0
#define SBM_ACKNACK_DATA_OFFSET_WRITER_ID	4
#define SBM_ACKNACK_DATA_OFFSET_READER_SN_STATE	8
#define SBM_ACKNACK_DATA_OFFSET_COUNT		24

#define SP_HDR_SIZE	4

#define SP_HDR_OFFSET_REPRESENTATION_ID		0
#define SP_HDR_OFFSET_REPRESENTATION_OPTIONS	2

#define SP_ID_CDR_BE	0x0000
#define SP_ID_CDR_LE	0x0001
#define SP_ID_PL_CDR_BE	0x0002
#define SP_ID_PL_CDR_LE	0x0003
#define	SP_ID_PL_CDR	0x0002

#define SP_DATA_LEN_SIZE	4

#define ROUND_UP(x, size)	(((x) + ((size) - 1)) & ~((size) - 1))
#define SP_DATA_SIZE(len)	(SP_DATA_LEN_SIZE + ROUND_UP(len, 4))

#define GUID_PREFIX_SIZE	12

#define GUID_PREFIX_UNKNOWN					\
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	\
	 0x00, 0x00, 0x00, 0x00}

#define ENTITYID_PARTICIPANT			{0x00, 0x00, 0x01, 0xc1}
#define ENTITYID_BUILTIN_PARTICIPANT_WRITER	{0x00, 0x01, 0x00, 0xc2}
#define ENTITYID_BUILTIN_PARTICIPANT_READER	{0x00, 0x01, 0x00, 0xc7}
#define ENTITYID_BUILTIN_PUBLICATIONS_WRITER	{0x00, 0x00, 0x03, 0xc2}
#define ENTITYID_BUILTIN_PUBLICATIONS_READER	{0x00, 0x00, 0x03, 0xc7}
#define ENTITYID_BUILTIN_SUBSCRIPTIONS_WRITER	{0x00, 0x00, 0x04, 0xc2}
#define ENTITYID_BUILTIN_SUBSCRIPTIONS_READER	{0x00, 0x00, 0x04, 0xc7}

#define ENDPOINT_PARTICIPANT_ANNOUNCER			0x00000001
#define ENDPOINT_PARTICIPANT_DETECTOR			0x00000002
#define ENDPOINT_PUBLICATIONS_ANNOUNCER			0x00000004
#define ENDPOINT_PUBLICATIONS_DETECTOR			0x00000008
#define ENDPOINT_SUBSCRIPTIONS_ANNOUNCER		0x00000010
#define ENDPOINT_SUBSCRIPTIONS_DETECTOR			0x00000020
#define ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER	0x00000400
#define ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER	0x00000800

#define PID_SENTINEL			0x0001
#define PID_PARTICIPANT_LEASE_DURATION	0x0002
#define PID_TOPIC_NAME			0x0005
#define PID_OWNERSHIP_STRENGTH		0x0006
#define PID_TYPE_NAME			0x0007
#define PID_PROTOCOL_VERSION		0x0015
#define PID_VENDOR_ID			0x0016
#define PID_RELIABILITY			0x001a
#define PID_LIVELINESS			0x001b
#define PID_DURABILITY			0x001d
#define PID_OWNERSHIP			0x001f
#define PID_DEADLINE			0x0023
#define PID_DESTINATION_ORDER		0x0025
#define PID_LATENCY_BUDGET		0x0027
#define PID_LIFESPAN			0x002b
#define PID_USER_DATA			0x002c
#define PID_UNICAST_LOCATOR		0x002f
#define PID_DEFAULT_UNICAST_LOCATOR	0x0031
#define PID_METATRAFFIC_UNICAST_LOCATOR	0x0032
#define PID_PARTICIPANT_GUID		0x0050
#define PID_BUILTIN_ENDPOINT_SET	0x0058
#define PID_ENDPOINT_GUID		0x005a
#define PID_TYPE_MAX_SIZE_SERIALIZED	0x0060
#define PID_ENTITY_NAME			0x0062
#define PID_KEY_HASH			0x0070

#define PID_PARTICIPANT_LEASE_DURATION_SIZE	8
#define PID_OWNERSHIP_STRENGTH_SIZE		4
#define PID_PROTOCOL_VERSION_SIZE		4
#define PID_VENDOR_ID_SIZE			4
#define PID_RELIABILITY_SIZE			12
#define PID_LIVELINESS_SIZE			12
#define PID_DURABILITY_SIZE			4
#define PID_OWNERSHIP_SIZE			4
#define PID_DEADLINE_SIZE			8
#define PID_DESTINATION_ORDER_SIZE		4
#define PID_LATENCY_BUDGET_SIZE			8
#define PID_LIFESPAN_SIZE			8
#define PID_UNICAST_LOCATOR_SIZE		24
#define PID_DEFAULT_UNICAST_LOCATOR_SIZE	24
#define PID_METATRAFFIC_UNICAST_LOCATOR_SIZE	24
#define PID_PARTICIPANT_GUID_SIZE		16
#define PID_BUILTIN_ENDPOINT_SET_SIZE		4
#define PID_ENDPOINT_GUID_SIZE			16
#define PID_TYPE_MAX_SIZE_SERIALIZED_SIZE	4
#define PID_KEY_HASH_SIZE			16

#define LOCATOR_KIND_INVALID	-1
#define LOCATOR_KIND_RESERVED	0
#define LOCATOR_KIND_UDPv4	1
#define LOCATOR_KIND_UDPv6	2

#define PB	7400
#define DG	250
#define PG	2
#define D0	0
#define D1	10
#define D2	1
#define D3	11

#define DISCOVERY_TRAFFIC_MULTICAST_PORT(domain_id)			\
	{(PB + DG * (domain_id) + D0) >> 8,				\
	 (PB + DG * (domain_id) + D0) & 0xff}
#define DISCOVERY_TRAFFIC_MULTICAST_PORT_0(seed)			\
	(((seed) + D0) >> 8)
#define DISCOVERY_TRAFFIC_MULTICAST_PORT_1(seed)			\
	(((seed) + D0) & 0xff)

#define DISCOVERY_TRAFFIC_UNICAST_PORT(domain_id, participant_id)	\
	{(PB + DG * (domain_id) + D1 + PG * (participant_id)) >> 8,	\
	 (PB + DG * (domain_id) + D1 + PG * (participant_id)) & 0xff}
#define DISCOVERY_TRAFFIC_UNICAST_PORT_0(seed, participant_id)	\
	(((seed) + D1 + PG * (participant_id)) >> 8)
#define DISCOVERY_TRAFFIC_UNICAST_PORT_1(seed, participant_id)	\
	(((seed) + D1 + PG * (participant_id)) & 0xff)

#define USER_TRAFFIC_MULTICAST_PORT(domain_id)				\
	{(PB + DG * (domain_id) + D2) >> 8,				\
	 (PB + DG * (domain_id) + D2) & 0xff}
#define USER_TRAFFIC_MULTICAST_PORT_0(seed)				\
	(((seed) + D2) >> 8)
#define USER_TRAFFIC_MULTICAST_PORT_1(seed)				\
	(((seed) + D2) & 0xff)

#define USER_TRAFFIC_UNICAST_PORT(domain_id, participant_id)		\
	{(PB + DG * (domain_id) + D3 + PG * (participant_id)) >> 8,	\
	 (PB + DG * (domain_id) + D3 + PG * (participant_id)) & 0xff}
#define USER_TRAFFIC_UNICAST_PORT_0(seed, participant_id)		\
	(((seed) + D3 + PG * (participant_id)) >> 8)
#define USER_TRAFFIC_UNICAST_PORT_1(seed, participant_id)		\
	(((seed) + D3 + PG * (participant_id)) & 0xff)

#define DOMAIN_ID(udp_port)	(((udp_port) - PB) / DG)

bool rtps_compare_protocol(const hls_uint<5> offset, const uint8_t x);
bool rtps_compare_reader_id(const hls_uint<5> offset, const uint8_t x,
			    const uint8_t entity_id[4]);

#endif // !RTPS_HPP
