# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_pre_ip_in
add_files hls/pre_ip_in.cpp
set_top pre_ip_in
source run_hls_artya7_common.tcl
exit
