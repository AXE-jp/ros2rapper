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

#include "common.hpp"

#include "checksum.hpp"
#include "hls.hpp"

/* Cyber func=inline */
void checksum(uint16_t &sum, const uint16_t offset, const uint8_t x)
{
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
