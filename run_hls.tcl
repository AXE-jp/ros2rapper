#    Copyright © 2021-2022 AXE, Inc. All Rights Reserved.
#
#    This file is part of ROS2rapper.
#
#    ROS2rapper is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    ROS2rapper is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with ROS2rapper.  If not, see <https://www.gnu.org/licenses/>.

open_project -reset proj_ros2

add_files app.cpp
add_files checksum.cpp
add_files ip.cpp
add_files ros2.cpp
add_files rtps.cpp
add_files sedp.cpp
add_files slip.cpp
add_files spdp.cpp
add_files udp.cpp
add_files -tb ros2_test.cpp

set_top ros2

open_solution -reset solution1

set_part xc7a100tcsg324-1
create_clock -period 100MHz

csim_design
csynth_design
cosim_design
export_design -format ip_catalog

exit
