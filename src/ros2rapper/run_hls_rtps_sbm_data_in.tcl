# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_rtps_sbm_data_in
add_files hls/rtps_in.cpp
add_files hls/rtps_sbm_data_in.cpp
set_top rtps_sbm_data_in
source run_hls_artya7_common.tcl
exit
