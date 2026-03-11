# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_test_input
add_files hls/test_input.cpp
set_top test_input
source run_hls_artya7_common.tcl
exit
