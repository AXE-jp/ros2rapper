# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2_main_au25p

add_files hls/endpoint.cpp
add_files hls/message_metadata.cpp
add_files hls/remove_endpoints.cpp
add_files hls/ros2.cpp
add_files hls/spdp.cpp

set_top ros2_main

source run_hls_au25p_common.tcl

exit
