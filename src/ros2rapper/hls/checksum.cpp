// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "checksum.hpp"
#include "hls.hpp"

/* Cyber func=inline */
void checksum(uint16_t &sum, const uint16_t offset, const uint8_t x) {
#pragma HLS inline
  hls_uint<17> r = sum;

  if (offset & 0x1)
    r += x;
  else
    r += x << 8;

  r = (r & 0xffff) + (r & 0x10000 ? 1 : 0);
  r = (r & 0xffff) + (r & 0x10000 ? 1 : 0);

  sum = r;
}
