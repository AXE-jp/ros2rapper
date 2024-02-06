// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef SPDP_HPP
#define SPDP_HPP

#include "endpoint.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"
#include <cstdint>

#define SPDP_DATA_SIZE 156

#define SPDP_WRITER_OCTETS_TO_NEXT_HEADER                                      \
  (SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SPDP_DATA_SIZE)

#define SPDP_WRITER_TOT_LEN                                                    \
  (RTPS_HDR_SIZE + SBM_HDR_SIZE + TIMESTAMP_SIZE + SBM_HDR_SIZE +              \
   SPDP_WRITER_OCTETS_TO_NEXT_HEADER)
#define SPDP_WRITER_RTPS_PKT_LEN SPDP_WRITER_TOT_LEN
#define SPDP_WRITER_UDP_PKT_LEN (UDP_HDR_SIZE + SPDP_WRITER_RTPS_PKT_LEN)
#define SPDP_WRITER_IP_PKT_LEN (IP_HDR_SIZE + SPDP_WRITER_UDP_PKT_LEN)

void spdp_reader(hls_stream<hls_uint<9>> &in, sedp_reader_id_t &reader_cnt,
                 sedp_endpoint reader_tbl[SEDP_READER_MAX], hls_uint<1> enable,
                 const uint8_t ip_addr[4], const uint8_t subnet_mask[4],
                 uint16_t port_num_seed);

void spdp_writer(const uint8_t writer_guid_prefix[12],
                 const uint8_t metatraffic_addr[4],
                 const uint8_t metatraffic_port[2],
                 const uint8_t default_addr[4], const uint8_t default_port[2],
                 uint8_t buf[SPDP_WRITER_TOT_LEN], const uint8_t entity_name[],
                 uint8_t entity_name_len);

#endif // !SPDP_HPP
