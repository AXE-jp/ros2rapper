# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

cd "ros2rapper-1KB"
open_project "ros2rapper-1KB.xpr"

set_property part xc7a100tcsg324-1 [current_project]

reset_run blk_mem_gen_0_synth_1
launch_runs blk_mem_gen_0_synth_1 -jobs 16
wait_on_run blk_mem_gen_0_synth_1
if {[get_property PROGRESS [get_runs blk_mem_gen_0_synth_1]] != "100%"} {
	error "ERROR: blk_mem_gen_0_synth_1 failed"
}

reset_run clk_wiz_0_synth_1
launch_runs clk_wiz_0_synth_1 -jobs 16
wait_on_run clk_wiz_0_synth_1
if {[get_property PROGRESS [get_runs clk_wiz_0_synth_1]] != "100%"} {
	error "ERROR: clk_wiz_0_synth_1 failed"
}

reset_run hls_pub_0_synth_1
launch_runs hls_pub_0_synth_1 -jobs 16
wait_on_run hls_pub_0_synth_1
if {[get_property PROGRESS [get_runs hls_pub_0_synth_1]] != "100%"} {
	error "ERROR: hls_pub_0_synth_1 failed"
}

reset_run hls_sub_0_synth_1
launch_runs hls_sub_0_synth_1 -jobs 16
wait_on_run hls_sub_0_synth_1
if {[get_property PROGRESS [get_runs hls_sub_0_synth_1]] != "100%"} {
	error "ERROR: hls_sub_0_synth_1 failed"
}

reset_run synth_1
launch_runs synth_1 -jobs 16
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
	error "ERROR: synth_1 failed"
}

launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
	error "ERROR: impl_1 failed"
}

close_project
