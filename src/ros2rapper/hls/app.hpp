// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef APP_HPP
#define APP_HPP

#include "ros2.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"
#include <cstdint>

#define APP_OCTETS_TO_NEXT_HEADER(len)                                         \
    (SBM_DATA_HDR_SIZE + SP_HDR_SIZE + ROUND_UP(len, 4))

#define APP_TOT_LEN(len)                                                       \
    (RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE + SBM_HDR_SIZE            \
     + TIMESTAMP_SIZE + SBM_HDR_SIZE + APP_OCTETS_TO_NEXT_HEADER(len))
#define APP_WRITER_RTPS_PKT_LEN(app_data_len) APP_TOT_LEN(app_data_len)
#define APP_WRITER_UDP_PKT_LEN(app_data_len)                                   \
    (UDP_HDR_SIZE + APP_WRITER_RTPS_PKT_LEN(app_data_len))
#define APP_WRITER_IP_PKT_LEN(app_data_len)                                    \
    (IP_HDR_SIZE + APP_WRITER_UDP_PKT_LEN(app_data_len))

void app_writer(const uint8_t writer_guid_prefix[12],
                const uint8_t writer_entity_id[4],
                const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id[4], const int64_t seqnum,
                volatile const uint8_t app_data[MAX_APP_DATA_LEN],
                uint32_t app_data_len, uint8_t buf[]);

void app_reader(hls_stream<hls_uint<9>> &in,
                const uint8_t            reader_guid_prefix[12],
                const uint8_t            reader_entity_id[4],
                volatile uint8_t        *sub_app_data_recv,
                volatile uint8_t        *sub_app_data_grant,
                uint8_t                  sub_app_data[MAX_APP_DATA_LEN],
                volatile uint8_t        *sub_app_data_len,
                volatile uint16_t       *sub_app_data_rep_id);

#endif // !APP_HPP
