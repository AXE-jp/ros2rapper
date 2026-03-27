# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_pre_ip_in_au25p
add_files hls/pre_ip_in.cpp
set_top pre_ip_in
source run_hls_au25p_common.tcl
exit
