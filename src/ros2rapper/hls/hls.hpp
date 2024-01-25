/*
    Copyright © 2021-2022 AXE, Inc. All Rights Reserved.

    This file is part of ROS2rapper.

    ROS2rapper is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ROS2rapper is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with ROS2rapper.  If not, see <https://www.gnu.org/licenses/>.
*/

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
