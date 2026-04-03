// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef SEDP_HPP
#define SEDP_HPP

#include "common.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"
#include <cstdint>

#define SEDP_DATA_SIZE (232 + MAX_TOPIC_NAME_LEN + MAX_TOPIC_TYPE_NAME_LEN)

#define SEDP_WRITER_OCTETS_TO_NEXT_HEADER                                      \
    (SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SEDP_DATA_SIZE)

#define SEDP_WRITER_TOT_LEN                                                    \
    (RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE + SBM_HDR_SIZE            \
     + TIMESTAMP_SIZE + SBM_HDR_SIZE + SEDP_WRITER_OCTETS_TO_NEXT_HEADER)
#define SEDP_WRITER_RTPS_PKT_LEN SEDP_WRITER_TOT_LEN
#define SEDP_WRITER_UDP_PKT_LEN  (UDP_HDR_SIZE + SEDP_WRITER_RTPS_PKT_LEN)
#define SEDP_WRITER_IP_PKT_LEN   (IP_HDR_SIZE + SEDP_WRITER_UDP_PKT_LEN)

#define SEDP_HEARTBEAT_TOT_LEN                                                 \
    (RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE + SBM_HDR_SIZE            \
     + SBM_HEARTBEAT_DATA_SIZE)
#define SEDP_HEARTBEAT_RTPS_PKT_LEN SEDP_HEARTBEAT_TOT_LEN
#define SEDP_HEARTBEAT_UDP_PKT_LEN  (UDP_HDR_SIZE + SEDP_HEARTBEAT_RTPS_PKT_LEN)
#define SEDP_HEARTBEAT_IP_PKT_LEN   (IP_HDR_SIZE + SEDP_HEARTBEAT_UDP_PKT_LEN)

#define SEDP_ACKNACK_TOT_LEN                                                   \
    (RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE + SBM_HDR_SIZE            \
     + SBM_ACKNACK_DATA_SIZE)
#define SEDP_ACKNACK_RTPS_PKT_LEN SEDP_ACKNACK_TOT_LEN
#define SEDP_ACKNACK_UDP_PKT_LEN  (UDP_HDR_SIZE + SEDP_ACKNACK_RTPS_PKT_LEN)
#define SEDP_ACKNACK_IP_PKT_LEN   (IP_HDR_SIZE + SEDP_ACKNACK_UDP_PKT_LEN)

void sedp_writer(
    const uint8_t vendor_id[2], const uint8_t writer_guid_prefix[12],
    const uint8_t writer_entity_id[4], const uint8_t reader_guid_prefix[12],
    const uint8_t reader_entity_id[4], int64_t seqnum,
    const uint8_t usertraffic_addr[4], const uint8_t usertraffic_port[2],
    const uint8_t app_entity_id[4], hls_stream<uint8_t> &out,
    const uint8_t topic_name[], uint8_t topic_name_len,
    const uint8_t type_name[], uint8_t type_name_len, timestamp now);

void sedp_heartbeat(const uint8_t vendor_id[2],
                    const uint8_t writer_guid_prefix[12],
                    const uint8_t writer_entity_id[4],
                    const uint8_t reader_guid_prefix[12],
                    const uint8_t reader_entity_id[4],
                    const int64_t first_seqnum, const int64_t last_seqnum,
                    const uint32_t cnt, hls_stream<uint8_t> &out);

void sedp_acknack(const uint8_t vendor_id[2],
                  const uint8_t writer_guid_prefix[12],
                  const uint8_t writer_entity_id[4],
                  const uint8_t reader_guid_prefix[12],
                  const uint8_t reader_entity_id[4], uint8_t snstate_base,
                  bool snstate_is_empty, const uint32_t cnt,
                  hls_stream<uint8_t> &out);

#endif // !SEDP_HPP
