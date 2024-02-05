// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "common.hpp"
#include "hls.hpp"
#include "ip.hpp"

static const uint8_t ip_packet[] = {
    0x45, 0x00, 0x00, 0x24, 0x8d, 0xa6, 0x40, 0x00, 0x40, 0x11, 0x28, 0xd2,
    0xc0, 0xa8, 0x01, 0x01, 0xc0, 0xa8, 0x01, 0xff, 0x05, 0xfe, 0x05, 0xfe,
    0x00, 0x10, 0x84, 0x72, 0x54, 0x43, 0x46, 0x32, 0x04, 0x00, 0x00, 0x00};

void process_ip_packet(const uint8_t *packet, size_t packet_size) {
  uint8_t ip_error;
  hls_stream<hls_uint<9>> in;
  hls_stream<hls_uint<9>> out;
  bool parity_error;
  static uint8_t
      payloadmem[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS];
  const uint32_t fragment_exp = 10;

  size_t i = 0;
  int prev_ip_error = 0;
  int parity_error_reported = 0;

  while (1) {
    hls_uint<9> x = packet[i];
    if (i == packet_size - 1)
      x |= 0x100;

    if (i < packet_size && in.write_nb(x))
      i++;
    ip_in(&ip_error, in, out, payloadmem, fragment_exp, parity_error);

    hls_uint<9> y;
    if (out.read_nb(y)) {
      //printf("%% out: %02x\n", y & 0xff);
      if (y & 0x100)
        break;
    }
  }
}

int main() {
  process_ip_packet(ip_packet, sizeof(ip_packet));
  process_ip_packet(ip_packet, sizeof(ip_packet));
  process_ip_packet(ip_packet, sizeof(ip_packet));
  process_ip_packet(ip_packet, sizeof(ip_packet));
  process_ip_packet(ip_packet, sizeof(ip_packet));
  process_ip_packet(ip_packet, sizeof(ip_packet));

  return 0;
}
