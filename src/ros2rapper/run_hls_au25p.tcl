# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2_main_au25p

add_files hls/endpoint.cpp
add_files hls/message_metadata.cpp
add_files hls/remove_endpoints.cpp
add_files hls/ros2.cpp
add_files hls/spdp.cpp

set_top ros2_main

open_solution -reset solution1

set_part xcau25p-ffvb676-1-e
config_rtl -reset all -reset_level low -reset_async
create_clock -period 100MHz

csynth_design
#export_design -format ip_catalog

exit
