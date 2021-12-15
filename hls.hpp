#ifndef HLS_HPP
#define HLS_HPP

#define VITIS_HLS
// #define CWB_HLS

#ifdef VITIS_HLS
#include <hls_stream.h>
template <typename T>
using hls_stream = hls::stream<T>;
#include <ap_int.h>
template <int W>
using hls_int = ap_int<W>;
template <int W>
using hls_uint = ap_uint<W>;
#endif // VITIS_HLS

#ifdef CWB_HLS
#include <cwb_cpp.h>
template <typename T>
using hls_stream = cwb::cwb_stream<T>;
#ifndef SC_INCLUDE_FX
#define SC_INCLUDE_FX
#endif // !SC_INCLUDE_FX
#include <systemc.h>
template <int W>
using hls_int = sc_int<W>;
template <int W>
using hls_uint = sc_uint<W>;
#endif // CWB_HLS

#endif // !HLS_HPP
