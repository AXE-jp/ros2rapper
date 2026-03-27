# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2_receiver_au25p

add_files hls/app.cpp
add_files hls/checksum.cpp
add_files hls/ip.cpp
add_files hls/remove_endpoints.cpp
add_files hls/ros2_receiver.cpp
add_files hls/rtps.cpp
add_files hls/sedp.cpp
add_files hls/slip.cpp
add_files hls/spdp.cpp
add_files hls/udp.cpp

set_top ros2_receiver

open_solution -reset solution1

set_part xcau25p-ffvb676-1-e
config_rtl -reset all -reset_level low -reset_async
create_clock -period 100MHz

csynth_design
#export_design -format ip_catalog

exit
