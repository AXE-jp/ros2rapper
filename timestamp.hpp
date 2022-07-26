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

#ifndef TIMESTAMP_HPP
#define TIMESTAMP_HPP

#include <cstdint>

#define TIMESTAMP_SIZE	8

struct timestamp {
	int32_t seconds;
	uint32_t fraction;
};

#define TIME_ZERO	{0x00000000, 0x00000000}
#define TIME_INVALID	{0xffffffff, 0xffffffff}
#define TIME_INFINITE	{0xffffffff, 0xfffffffe}

#endif // !TIMESTAMP_HPP
