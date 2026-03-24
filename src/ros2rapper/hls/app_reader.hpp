#pragma once

#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include <cstdint>

void app_reader(hls_uint<9> x, hls_uint<SUB_TOPICS_MAX> app_matched,
                hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
                uint8_t                   sub_app_data_0[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_1[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_2[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_3[MAX_APP_DATA_LEN],
                hls_stream<uint64_t>     &sub_app_data_recvinfo);
