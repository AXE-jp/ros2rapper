# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_udp_ip_in_au25p
add_files hls/udp_ip_in.cpp
set_top udp_ip_in
source run_hls_au25p_common.tcl
exit
