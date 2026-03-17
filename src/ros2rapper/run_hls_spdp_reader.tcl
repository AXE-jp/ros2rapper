# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_spdp_reader
add_files hls/ip.cpp
add_files hls/rtps.cpp
add_files hls/spdp_reader.cpp
set_top spdp_reader
source run_hls_artya7_common.tcl
exit
