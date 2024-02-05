# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

# create_project.tcl  Tcl script for creating project

set     project_directory   [file dirname [info script]]
set     project_name        "ros2rapper-udp"
set     device_part         "xc7a100tcsg324-1"
set     design_xdc_file     [ glob ./constrs/* ]

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
set_property -name "verilog_define" -value "TARGET_XILINX=1 ROS2RAPPER_HLS_VITIS=1" -objects $obj
set_property -name "verilog_version" -value "verilog_2001" -objects $obj

# Create fileset "constrs_1"
if {[string equal [get_filesets -quiet constrs_1] ""]} {
    create_fileset -constrset constrs_1
}

# Create fileset "sim_1"
if {[string equal [get_filesets -quiet sim_1] ""]} {
    create_fileset -simset sim_1
}

# Create run "synth_1" and set property
set synth_1_flow     "Vivado Synthesis 2023"
set synth_1_strategy "Vivado Synthesis Defaults"
if {[string equal [get_runs -quiet synth_1] ""]} {
    create_run -name synth_1 -flow $synth_1_flow -strategy $synth_1_strategy -constrset constrs_1
} else {
    set_property flow     $synth_1_flow     [get_runs synth_1]
    set_property strategy $synth_1_strategy [get_runs synth_1]
}
current_run -synthesis [get_runs synth_1]

# Create run "impl_1" and set property
set impl_1_flow      "Vivado Implementation 2023"
set impl_1_strategy  "Vivado Implementation Defaults"
if {[string equal [get_runs -quiet impl_1] ""]} {
    create_run -name impl_1 -flow $impl_1_flow -strategy $impl_1_strategy -constrset constrs_1 -parent_run synth_1
} else {
    set_property flow     $impl_1_flow      [get_runs impl_1]
    set_property strategy $impl_1_strategy  [get_runs impl_1]
}
current_run -implementation [get_runs impl_1]

# Import sources
add_files -norecurse -fileset sources_1 [ glob ./*.v ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/include/*.vh ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/proj_ros2/solution1/syn/verilog/*.v ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ether/include/*.vh ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ether/rtl/*.v ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ether/verilog-ethernet/*.v ]

# Import xdc files
if {[info exists design_xdc_file]} {
    add_files    -fileset constrs_1 -norecurse $design_xdc_file
}
