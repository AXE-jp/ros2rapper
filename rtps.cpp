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

#include "common.hpp"

#include "rtps.hpp"

/* Cyber func=inline */
bool rtps_compare_protocol(const hls_uint<5> offset, const uint8_t x)
{
#pragma HLS inline
	switch (offset) {
	case RTPS_HDR_OFFSET_PROTOCOL_ID:
		return x == 'R' ? true : false;
	case RTPS_HDR_OFFSET_PROTOCOL_ID + 1:
		return x == 'T' ? true : false;
	case RTPS_HDR_OFFSET_PROTOCOL_ID + 2:
		return x == 'P' ? true : false;
	case RTPS_HDR_OFFSET_PROTOCOL_ID + 3:
		return x == 'S' ? true : false;
	case RTPS_HDR_OFFSET_PROTOCOL_VERSION:
		return x == (RTPS_HDR_PROTOCOL_VERSION >> 8) ? true : false;
	case RTPS_HDR_OFFSET_PROTOCOL_VERSION + 1:
		return x == (RTPS_HDR_PROTOCOL_VERSION & 0xff) ? true : false;
	default:
		return true;
	}
}

/* Cyber func=inline */
bool rtps_compare_reader_id(const hls_uint<5> offset, const uint8_t x,
			    const uint8_t entity_id[4])
{
#pragma HLS inline
	switch (offset) {
	case SBM_DATA_HDR_OFFSET_READER_ID:
		return x == entity_id[0] ? true : false;
	case SBM_DATA_HDR_OFFSET_READER_ID + 1:
		return x == entity_id[1] ? true : false;
	case SBM_DATA_HDR_OFFSET_READER_ID + 2:
		return x == entity_id[2] ? true : false;
	case SBM_DATA_HDR_OFFSET_READER_ID + 3:
		return x == entity_id[3] ? true : false;
	default:
		return true;
	}
}
