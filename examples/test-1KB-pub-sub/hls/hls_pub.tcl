# Copyright (c) 2021-2025 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset hls_pub
add_files hls/hls_pub.cpp
set_top hls_pub
open_solution -reset solution1
set_part xc7a100tcsg324-1
config_rtl -reset all -reset_level low -reset_async
create_clock -period 80MHz
csynth_design
export_design -rtl verilog -format ip_catalog
exit
