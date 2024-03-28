// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef UDP_HPP
#define UDP_HPP

#include "common.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include <cstdint>

#define UDP_HDR_SIZE 8

#define UDP_HDR_OFFSET_SPORT 0 // Source Port
#define UDP_HDR_OFFSET_DPORT 2 // Destination Port
#define UDP_HDR_OFFSET_ULEN  4 // Length
#define UDP_HDR_OFFSET_SUM   6 // Checksum

#define PSEUDO_HDR_SIZE 12

#define PSEUDO_HDR_OFFSET_SADDR    0  // Source Address
#define PSEUDO_HDR_OFFSET_DADDR    4  // Destination Address
#define PSEUDO_HDR_OFFSET_PROTOCOL 8  // Protocol
#define PSEUDO_HDR_OFFSET_ULEN     10 // Length

#define PSEUDO_HDR_PROTOCOL 0x11 // UDP

void udp_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
            hls_uint<1> &enable, const uint8_t rx_udp_port[2],
            uint32_t udp_rxbuf[], volatile uint8_t *udp_rxbuf_rel,
            volatile uint8_t *udp_rxbuf_grant, bool &parity_error);

void udp_out(const uint8_t src_addr[4], const uint8_t src_port[2],
             const uint8_t dst_addr[4], const uint8_t dst_port[2],
             const uint8_t udp_data[], const uint16_t udp_data_process_len,
             const uint16_t udp_data_real_len, uint8_t buf[]);

void udp_set_header(const uint8_t src_port[2], const uint8_t dst_port[2],
                    const uint16_t udp_data_len, uint8_t udp_hdr[]);

void udp_set_checksum(uint8_t buf[]);

#endif // !UDP_HPP
