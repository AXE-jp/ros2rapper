#pragma once

#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

void sedp_heartbeat_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                       const uint8_t guid_prefix[GUID_PREFIX_SIZE],
                       uint8_t       sbm_flags);
