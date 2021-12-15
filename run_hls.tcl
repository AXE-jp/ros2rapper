open_project -reset proj_ros2

add_files app.cpp
add_files checksum.cpp
add_files ip.cpp
add_files ros2.cpp
add_files rtps.cpp
add_files sedp.cpp
add_files slip.cpp
add_files spdp.cpp
add_files udp.cpp
add_files -tb ros2_test.cpp

set_top ros2

open_solution -reset solution1

set_part xc7a100tcsg324-1
create_clock -period 100MHz

csim_design
csynth_design
cosim_design
export_design -format ip_catalog -evaluate verilog

exit
