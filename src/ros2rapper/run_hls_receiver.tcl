# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2_receiver

add_files hls/app_reader.cpp
add_files hls/ip.cpp
add_files hls/ros2_receiver.cpp
add_files hls/rtps.cpp
add_files hls/rtps_sbm_data_in.cpp
add_files hls/rtps_sbm_heartbeat_in.cpp
add_files hls/sedp_reader.cpp
add_files hls/spdp_reader.cpp

set_top ros2_receiver
source run_hls_artya7_common.tcl
exit
