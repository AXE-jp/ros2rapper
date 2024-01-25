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

#ifndef DURATION_HPP
#define DURATION_HPP

#include "timestamp.hpp"

#define DURATION_SIZE TIMESTAMP_SIZE

typedef timestamp duration;

#define DURATION_ZERO                                                          \
  { 0x00000000, 0x00000000 }
#define DURATION_INFINITE                                                      \
  { 0x7fffffff, 0xffffffff }

#endif // !DURATION_HPP
