# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_ros2test

add_files hls/app.cpp
add_files hls/app_reader.cpp
add_files hls/checksum.cpp
add_files hls/endpoint.cpp
add_files hls/ip.cpp
add_files hls/message_metadata.cpp
add_files hls/pre_ip_in.cpp
add_files hls/remove_endpoints.cpp
add_files hls/ros2.cpp
add_files hls/ros2_receiver.cpp
add_files hls/ros2_sender.cpp
add_files hls/rtps.cpp
add_files hls/rtps_sbm_data_in.cpp
add_files hls/rtps_sbm_heartbeat_in.cpp
add_files hls/sedp.cpp
add_files hls/sedp_reader.cpp
add_files hls/slip.cpp
add_files hls/spdp.cpp
add_files hls/spdp_reader.cpp
add_files hls/udp.cpp
add_files hls/udp_ip_in.cpp
add_files -tb -cflags "-Ihls" test/test.cpp
add_files -tb -cflags "-Ihls" test/test_message_metadata.cpp
add_files -tb -cflags "-Ihls" test/test_multi_topic_subscription.cpp
add_files -tb -cflags "-Ihls" test/test_remove_endpoints.cpp
add_files -tb -cflags "-Ihls" test/test_sedp_reader_2.cpp
add_files -tb -cflags "-Ihls" test/test_sedp_reader_heartbeat.cpp
add_files -tb -cflags "-Ihls" test/test_spdp_reader.cpp
add_files -tb -cflags "-Ihls" test/test_udp_ip_in.cpp
add_files -tb -cflags "-Ihls" test/test_utils.cpp

open_solution -reset solution1

set_part xc7a100tcsg324-1
config_rtl -reset all -reset_level low -reset_async
create_clock -period 80MHz

csim_design

exit
