# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_project -reset proj_test_input

add_files hls/test_input.cpp

set_top test_input

open_solution -reset solution1

set_part xc7a100tcsg324-1
config_rtl -reset all -reset_level low -reset_async
create_clock -period 80MHz

csynth_design
export_design -format ip_catalog

exit
