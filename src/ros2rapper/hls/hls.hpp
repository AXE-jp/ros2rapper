// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HLS_HPP
#define HLS_HPP

#if defined(__VITIS_HLS__)
#define VITIS_HLS
#else
#define CWB_HLS
#endif

#ifdef VITIS_HLS
#include <gmp.h>
#define __gmp_const const
#include <hls_stream.h>
template <typename T> using hls_stream = hls::stream<T>;
#include <ap_int.h>
template <int W> using hls_int = ap_int<W>;
template <int W> using hls_uint = ap_uint<W>;
#define WAIT_CLOCK ap_wait()
#endif // VITIS_HLS

#ifdef CWB_HLS
#include <cwb_cpp.h>
template <typename T> using hls_stream = cwb::cwb_stream<T>;
#ifndef SC_INCLUDE_FX
#define SC_INCLUDE_FX
#endif // !SC_INCLUDE_FX
#include <systemc.h>
template <int W> using hls_int = sc_int<W>;
template <int W> using hls_uint = sc_uint<W>;
#define __SYNTHESIS__
#define WAIT_CLOCK cwb::cwb_clk()
#endif // CWB_HLS

#endif // !HLS_HPP
