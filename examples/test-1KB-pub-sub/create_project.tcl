# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

# create_project.tcl  Tcl script for creating project

set     project_directory   [file dirname [info script]]
set     project_name        "ros2rapper-1KB"
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
if {[string equal [lindex $argv 0] "vitis"]} {
    set_property -name "verilog_define" -value "TARGET_XILINX=1 ROS2RAPPER_HLS_VITIS=1 XILINX_CLKIN_STYLE_BUFR=1" -objects $obj
} elseif {[string equal [lindex $argv 0] "cwb"]} {
    set_property -name "verilog_define" -value "TARGET_XILINX=1 ROS2RAPPER_HLS_CWB=1 XILINX_CLKIN_STYLE_BUFR=1" -objects $obj
}
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
add_files -norecurse -fileset sources_1 [ glob ./rtl/*.v ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/include/*.vh ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/rtl/*.v ]

if {[string equal [lindex $argv 0] "vitis"]} {
    add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/proj_ros2_main/solution1/syn/verilog/*.v ]
    add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/proj_ros2_receiver/solution1/syn/verilog/*.v ]
    add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/proj_ros2_sender/solution1/syn/verilog/*.v ]
} elseif {[string equal [lindex $argv 0] "cwb"]} {
    add_files -norecurse -fileset sources_1 [ glob ../../src/ros2rapper/*.v ]
}

add_files -norecurse -fileset sources_1 [ glob ../../src/ether/include/*.vh ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ether/rtl/*.v ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ether/lib/*.v ]
add_files -norecurse -fileset sources_1 [ glob ../../src/ether/verilog-ethernet/*.v ]

set_property  ip_repo_paths  {./hls_pub ./hls_sub} [current_project]

# Create IPs
if {[string equal [get_ips -quiet blk_mem_gen_0] ""]} {
    create_ip -name blk_mem_gen -vendor xilinx.com -library ip -version 8.4 -module_name blk_mem_gen_0
    set_property -dict [list \
        CONFIG.Memory_Type {Simple_Dual_Port_RAM} \
        CONFIG.Write_Depth_A {1024} \
        CONFIG.Write_Width_A {8} \
        CONFIG.Write_Width_B {16} \
    ] [get_ips blk_mem_gen_0]
    generate_target all [get_files ./$project_name/$project_name.srcs/sources_1/ip/blk_mem_gen_0/blk_mem_gen_0.xci]
    create_ip_run [get_files -of_objects [get_fileset sources_1] ./$project_name/$project_name.srcs/sources_1/ip/blk_mem_gen_0/blk_mem_gen_0.xci]
}

if {[string equal [get_ips -quiet hls_pub_0] ""]} {
   create_ip -name hls_pub -library hls -module_name hls_pub_0
   generate_target all [get_files ./$project_name/$project_name.srcs/sources_1/ip/hls_pub_0/hls_pub_0.xci]
   create_ip_run [get_files -of_objects [get_fileset sources_1] ./$project_name/$project_name.srcs/sources_1/ip/hls_pub_0/hls_pub_0.xci]
} else {
   upgrade_ip -quiet [get_ips hls_pub_0]
}

if {[string equal [get_ips -quiet hls_sub_0] ""]} {
    create_ip -name hls_sub -library hls -module_name hls_sub_0
    generate_target all [get_files ./$project_name/$project_name.srcs/sources_1/ip/hls_sub_0/hls_sub_0.xci]
    create_ip_run [get_files -of_objects [get_fileset sources_1] ./$project_name/$project_name.srcs/sources_1/ip/hls_sub_0/hls_sub_0.xci]
} else {
    upgrade_ip -quiet [get_ips hls_sub_0]
}

# Designate the top module
set_property top top [get_filesets sources_1]

# Import xdc files
add_files -fileset constrs_1 -norecurse "./constrs/arty_a7_eth.xdc ./constrs/eth_mac_fifo.tcl ./constrs/axis_async_fifo.tcl ./constrs/sync_reset.tcl"
