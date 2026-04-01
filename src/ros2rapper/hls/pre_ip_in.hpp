// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef PRE_IP_IN_HPP
#define PRE_IP_IN_HPP

#include "hls.hpp"
#include <cstdint>

void pre_ip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out);

#endif // !PRE_IP_IN_HPP
