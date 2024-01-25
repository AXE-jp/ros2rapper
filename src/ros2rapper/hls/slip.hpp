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

#ifndef SLIP_HPP
#define SLIP_HPP

#include "hls.hpp"
#include <cstdint>

// from RFC1055
#define END 0300     // indicates end of packet
#define ESC 0333     // indicates byte stuffing
#define ESC_END 0334 // ESC ESC_END means END data byte
#define ESC_ESC 0335 // ESC ESC_ESC means ESC data byte

void slip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out);
void slip_out(hls_stream<hls_uint<9>> &in, hls_stream<uint8_t> &out);

#endif // !SLIP_HPP
