# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_app_reader
add_files {hls/app_reader.cpp hls/rtps.cpp}
set_top app_reader
source run_hls_artya7_common.tcl
exit
