# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset hls_pub
add_files hls/hls_pub.cpp
set_top hls_pub
source "../../src/ros2rapper/run_hls_artya7_common.tcl"
export_design -rtl verilog -format ip_catalog
exit
