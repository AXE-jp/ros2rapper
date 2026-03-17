#pragma once

#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

void rtps_data_in(hls_uint<9> x, hls_uint<10> *out,
                  hls_stream<rtps_data_t> &rtps_data_stream, uint8_t sbm_flags,
                  const uint8_t guid_prefix[GUID_PREFIX_SIZE]);
