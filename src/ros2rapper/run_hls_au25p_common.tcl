# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

open_solution -reset solution1

set_part xcau25p-ffvb676-1-e
config_rtl -reset all -reset_level low -reset_async
create_clock -period 100MHz

csynth_design
#export_design -format ip_catalog
