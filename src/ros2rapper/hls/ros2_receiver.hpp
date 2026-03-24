// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ROS2_RECEIVER_HPP
#define ROS2_RECEIVER_HPP

#include "hls.hpp"
#include <cstdint>

typedef hls_uint<3> rtps_type_t;
#define RTPS_TYPE_SPDP               0
#define RTPS_TYPE_SEDP_HEARTBEAT_PUB 1
#define RTPS_TYPE_SEDP_HEARTBEAT_SUB 2
#define RTPS_TYPE_SEDP_PUB_SN_ONLY   3
#define RTPS_TYPE_SEDP_SUB_SN_ONLY   4
#define RTPS_TYPE_SEDP_PUB           5
#define RTPS_TYPE_SEDP_SUB           6
#define RTPS_TYPE_RM_ENDPOINT        7

typedef struct {
    rtps_type_t type;
    uint8_t     guid_prefix[12] /* Cyber array=EXPAND */;
    uint8_t     data[14] /* Cyber array=EXPAND */;
} rtps_data_t;

#include "ros2.hpp"

void ros2_receiver(hls_stream<hls_uint<9>> &in, hls_stream<rtps_data_t> &out,
                   hls_uint<PUB_TOPICS_MAX>  pub_enable,
                   hls_uint<SUB_TOPICS_MAX>  sub_enable,
                   hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                   hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                   hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
                   uint8_t                   sub_app_data_0[MAX_APP_DATA_LEN],
                   uint8_t                   sub_app_data_1[MAX_APP_DATA_LEN],
                   uint8_t                   sub_app_data_2[MAX_APP_DATA_LEN],
                   uint8_t                   sub_app_data_3[MAX_APP_DATA_LEN],
                   hls_stream<uint64_t>     &sub_app_data_recvinfo,
                   const receiver_config_t  &conf);

#endif // !ROS2_RECEIVER_HPP
