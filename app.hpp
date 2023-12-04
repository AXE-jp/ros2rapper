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

#ifndef APP_HPP
#define APP_HPP

#include <cstdint>
#include "rtps.hpp"
#include "timestamp.hpp"

#define APP_OCTETS_TO_NEXT_HEADER(len)				\
	(SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SP_DATA_SIZE(len))

#define APP_TOT_LEN(len)					\
	(RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE +	\
	 SBM_HDR_SIZE + TIMESTAMP_SIZE +			\
	 SBM_HDR_SIZE + APP_OCTETS_TO_NEXT_HEADER(len))
#define APP_WRITER_RTPS_PKT_LEN				\
	APP_TOT_LEN(MAX_APP_DATA_LEN)
#define APP_WRITER_UDP_PKT_LEN				\
	(UDP_HDR_SIZE + APP_WRITER_RTPS_PKT_LEN)
#define APP_WRITER_IP_PKT_LEN				\
	(IP_HDR_SIZE + APP_WRITER_UDP_PKT_LEN)

void app_writer(const uint8_t writer_guid_prefix[12],
		const uint8_t writer_entity_id[4],
		const uint8_t reader_guid_prefix[12],
		const uint8_t reader_entity_id[4],
		const int64_t seqnum,
		volatile const uint8_t app_data[],
		uint32_t app_data_len,
		uint8_t buf[]);

#endif // !APP_HPP
