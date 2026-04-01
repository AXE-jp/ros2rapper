// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef UDP_IP_IN_HPP
#define UDP_IP_IN_HPP

#include "hls.hpp"
#include "ip.hpp"
#include <cstdint>

#define ERR_UDP_IP_IN_CANNOT_PROCESS 1
#define ERR_UDP_IP_IN_NO_ROOM        2
#define ERR_UDP_IP_IN_INVALID_LENGTH 3

void udp_ip_in(
    hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
    uint8_t  ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    uint32_t fragment_expiration, uint8_t *error);

#endif // !UDP_IP_IN_HPP
