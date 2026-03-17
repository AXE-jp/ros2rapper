# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_sedp_heartbeat_in
add_files hls/rtps.cpp
add_files hls/sedp_heartbeat_in.cpp
set_top sedp_heartbeat_in
source run_hls_artya7_common.tcl
exit
