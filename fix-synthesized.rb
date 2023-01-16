#!/usr/bin/env ruby

ros2_src = "proj_ros2/solution1/syn/verilog/ros2.v"
ros2_fixed = File.read(ros2_src)
  .gsub(/ros2_payloads #(.*?);/m, "")
  .gsub(/udp_txbuf_q0,/, "udp_txbuf_q0,\n        payloads_address0,\n        payloads_ce0,\n        payloads_we0,\n        payloads_d0,\n        payloads_q0,")
  .gsub(/reg.*\[11:0\] payloads_address0;/, "output reg   [11:0] payloads_address0;")
  .gsub(/reg.*payloads_ce0;/,             "output reg    payloads_ce0;")
  .gsub(/reg.*payloads_we0;/,             "output reg    payloads_we0;")
  .gsub(/wire.*\[7:0\] payloads_d0;/,       "output wire   [7:0] payloads_d0;")
  .gsub(/wire.*\[7:0\] payloads_q0;/,       "input wire   [7:0] payloads_q0;")
File.write(ros2_src, ros2_fixed)

Dir.glob('proj_ros2/solution1/syn/verilog/*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/bx/, "b0")
  File.write(filename, fixed)
end
