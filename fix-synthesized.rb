#!/usr/bin/env ruby

ros2_src = "proj_ros2/solution1/syn/verilog/ros2.v"
ros2_fixed = File.read(ros2_src)
  .gsub(/ros2_payloads #(.*?);/m, "")
  .gsub(/udp_txbuf_q0,/, "udp_txbuf_q0,\n        payloads_address0,\n        payloads_ce0,\n        payloads_we0,\n        payloads_d0,\n        payloads_q0,")
File.write(ros2_src, ros2_fixed)

Dir.glob('proj_ros2/solution1/syn/verilog/*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/bx/, "b0")
  File.write(filename, fixed)
end
