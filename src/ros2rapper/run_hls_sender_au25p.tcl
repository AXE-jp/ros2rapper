# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2_sender_au25p

add_files hls/app.cpp
add_files hls/ip.cpp
add_files hls/message_metadata.cpp
add_files hls/ros2_sender.cpp
add_files hls/sedp.cpp
add_files hls/slip.cpp
add_files hls/spdp.cpp
add_files hls/udp.cpp

set_top ros2_sender

source run_hls_au25p_common.tcl

exit
