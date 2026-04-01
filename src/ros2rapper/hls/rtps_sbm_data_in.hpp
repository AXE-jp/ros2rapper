// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef RTPS_SBM_DATA_IN_HPP
#define RTPS_SBM_DATA_IN_HPP

#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"

void rtps_sbm_data_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                      hls_uint<PUB_TOPICS_MAX> pub_enable,
                      hls_uint<SUB_TOPICS_MAX> sub_enable, uint8_t sbm_flags,
                      const uint8_t src_guid_prefix[GUID_PREFIX_SIZE],
                      hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                      hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                      hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
                      uint8_t                  sub_app_data_0[MAX_APP_DATA_LEN],
                      uint8_t                  sub_app_data_1[MAX_APP_DATA_LEN],
                      uint8_t                  sub_app_data_2[MAX_APP_DATA_LEN],
                      uint8_t                  sub_app_data_3[MAX_APP_DATA_LEN],
                      hls_stream<uint64_t>    &sub_app_data_recvinfo,
                      const receiver_config_t &conf);

#endif // !RTPS_SBM_DATA_IN_HPP
