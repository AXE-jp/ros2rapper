// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef SPDP_HPP
#define SPDP_HPP

#include "duration.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"
#include <cstdint>

#define SPDP_DATA_SIZE 156

#define SPDP_WRITER_OCTETS_TO_NEXT_HEADER                                      \
    (SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SPDP_DATA_SIZE)

#define SPDP_WRITER_TOT_LEN                                                    \
    (RTPS_HDR_SIZE + SBM_HDR_SIZE + TIMESTAMP_SIZE + SBM_HDR_SIZE              \
     + SPDP_WRITER_OCTETS_TO_NEXT_HEADER)
#define SPDP_WRITER_RTPS_PKT_LEN SPDP_WRITER_TOT_LEN
#define SPDP_WRITER_UDP_PKT_LEN  (UDP_HDR_SIZE + SPDP_WRITER_RTPS_PKT_LEN)
#define SPDP_WRITER_IP_PKT_LEN   (IP_HDR_SIZE + SPDP_WRITER_UDP_PKT_LEN)

void compare_guid_prefix_of_sedp_endpoint(
    const uint8_t x, const sedp_endpoint tbl[SEDP_READER_MAX], const int idx,
    bool unmatched[SEDP_READER_MAX]);

void reset_sedp_unmatched(bool unmatched[SEDP_READER_MAX]);
void reset_app_unmatched(bool unmatched[APP_READER_MAX]);
#define reset_sedp_endpoint_children reset_app_unmatched

void spdp_reader(hls_uint<9> in, hls_stream<rtps_data_t> &out,
                 hls_uint<1> enable, const uint8_t ip_addr[4],
                 const uint8_t subnet_mask[4], uint16_t port_num_seed);

void spdp_writer(const uint8_t writer_guid_prefix[12],
                 const uint8_t metatraffic_addr[4],
                 const uint8_t metatraffic_port[2],
                 const uint8_t default_addr[4], const uint8_t default_port[2],
                 duration lease_duration, uint8_t buf[SPDP_WRITER_TOT_LEN],
                 const uint8_t entity_name[], uint8_t entity_name_len,
                 timestamp now);

#endif // !SPDP_HPP
