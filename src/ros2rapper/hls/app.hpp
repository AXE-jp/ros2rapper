// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef APP_HPP
#define APP_HPP

#include "endpoint.hpp"
#include "hls.hpp"
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

void app_writer(const uint8_t vendor_id[2],
                const uint8_t writer_guid_prefix[12],
                const uint8_t writer_entity_id[4],
                const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id[4], const int64_t seqnum,
#ifdef PUB_DATA_FF
                VOLATILE
#endif // PUB_DATA_FF
                const uint32_t app_data[MAX_APP_DATA_LEN / 4],
                uint32_t app_data_len, hls_stream<uint8_t> &out, timestamp now);

void app_reader(hls_uint<9> in, const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id_list[SUB_TOPICS_MAX][4],
                hls_uint<SUB_TOPICS_MAX> sub_enabled,
                VOLATILE hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                VOLATILE hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                VOLATILE hls_uint<SUB_TOPICS_MAX> *sub_app_data_grant,
                uint8_t               sub_app_data_0[MAX_APP_DATA_LEN],
                uint8_t               sub_app_data_1[MAX_APP_DATA_LEN],
                uint8_t               sub_app_data_2[MAX_APP_DATA_LEN],
                uint8_t               sub_app_data_3[MAX_APP_DATA_LEN],
                hls_stream<uint64_t> &sub_app_data_recvinfo);

#endif // !APP_HPP
