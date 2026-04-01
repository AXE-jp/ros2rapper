// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef RTPS_SBM_HEARTBEAT_IN_HPP
#define RTPS_SBM_HEARTBEAT_IN_HPP

#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

void rtps_sbm_heartbeat_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                           bool          sbm_le,
                           const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]);

#endif // !RTPS_SBM_HEARTBEAT_IN_HPP
