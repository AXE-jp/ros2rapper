// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "slip.hpp"

/* Cyber func=inline */
void slip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out) {
#pragma HLS inline
  static hls_uint<2> state;
  static uint8_t r;

  uint8_t x;

  if (!in.read_nb(x))
    return;

  switch (state) {
  case 0:
    switch (x) {
    case END:
      break;
    case ESC:
      state = 2;
      break;
    default:
      r = x;
      state = 1;
    }
    break;
  case 1:
    switch (x) {
    case END:
      out.write(0x100 | r);
      state = 0;
      break;
    case ESC:
      out.write(r);
      state = 2;
      break;
    default:
      out.write(r);
      r = x;
    }
    break;
  case 2:
    if (x == ESC_END)
      r = END;
    else if (x == ESC_ESC)
      r = ESC;
    state = 1;
  }
}

/* Cyber func=inline */
void slip_out(hls_stream<hls_uint<9>> &in, hls_stream<uint8_t> &out) {
#pragma HLS inline
  static bool start = false;

  hls_uint<9> x;

  if (!in.read_nb(x))
    return;

  uint8_t data = x & 0xff;
  bool end = x & 0x100;

  if (!start) {
    out.write(END);
    start = true;
  }

  switch (data) {
  case END:
    out.write(ESC);
    out.write(ESC_END);
    break;
  case ESC:
    out.write(ESC);
    out.write(ESC_ESC);
    break;
  default:
    out.write(data);
  }

  if (end) {
    out.write(END);
    start = false;
  }
}
