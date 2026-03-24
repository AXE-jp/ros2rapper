#pragma once
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

void rtps_sbm_heartbeat_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                           bool          sbm_le,
                           const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]);
