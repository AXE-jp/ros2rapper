# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

# run_tb.tcl  Tcl script for running testbench

set     project_directory   [file dirname [info script]]
set     project_name        "proj_tb"
set     device_part         "xc7a100tcsg324-1"

# Create project
create_project -force $project_name $project_directory/$project_name

# Set project properties
set_property "part"               $device_part     [current_project]
set_property "default_lib"        "xil_defaultlib" [current_project]
set_property "simulator_language" "Mixed"          [current_project]
set_property "target_language"    "Verilog"        [current_project]

# Create fileset "sources_1"
if {[string equal [get_filesets -quiet sources_1] ""]} {
    create_fileset -srcset sources_1
}
set obj [get_filesets sources_1]
set_property -name "loop_count" -value "1000" -objects $obj
set_property -name "verilog_define" -value "TARGET_XILINX=1 ROS2RAPPER_HLS_VITIS=1 XILINX_CLKIN_STYLE_BUFR=1" -objects $obj
set_property -name "verilog_version" -value "verilog_2001" -objects $obj

# Create fileset "constrs_1"
if {[string equal [get_filesets -quiet constrs_1] ""]} {
    create_fileset -constrset constrs_1
}

# Create fileset "sim_1"
if {[string equal [get_filesets -quiet sim_1] ""]} {
    create_fileset -simset sim_1
}

# Import sources
add_files -norecurse -fileset sources_1 ./include/ros2_ether_config.vh
add_files -norecurse -fileset sources_1 ./rtl/raw_eth_rx_adapter.v
add_files -norecurse -fileset sources_1 ./verilog-ethernet/axis_async_fifo.v
add_files -norecurse -fileset sim_1 ./tb/test_axis_async_fifo.sv
add_files -norecurse -fileset sim_1 ./tb/test_raw_eth_rx_adapter.sv

# Run simulation
foreach top_module {test_axis_async_fifo test_raw_eth_rx_adapter} {
    set_property top $top_module [get_filesets sim_1]
    launch_simulation
    source $project_name/$project_name.sim/sim_1/behav/xsim/$top_module.tcl
    run -all
    if {[get_value -radix unsigned /$top_module/error_code] != "0"} {
        error "Simulation $top_module is failed"
    }
}

close_project
