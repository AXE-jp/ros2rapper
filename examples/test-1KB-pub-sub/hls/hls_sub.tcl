# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset hls_sub
add_files hls/hls_sub.cpp
set_top hls_sub
source "../../src/ros2rapper/run_hls_artya7_common.tcl"
export_design -rtl verilog -format ip_catalog
exit
