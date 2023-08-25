#!/usr/bin/env ruby

SRCDIR = './gensrc/'

[SRCDIR+'ros2_app_writer_out.v'].each do |filename|
  fixed = File.read(filename)
    .gsub(/always @ \(posedge ap_clk\) begin(.*?)if \(ap_rst == 1'b1\) begin/m,
          "always @ (posedge ap_clk or posedge ap_rst) begin\n    if (ap_rst) begin")
  File.write(filename, fixed)
end

[SRCDIR+'ros2.v'].each do |filename|
  fixed = File.read(filename)
    .gsub(/ros2_payloads #(.*?);/m, "")
    .gsub(/udp_txbuf_q0,\s+enable,/, "udp_txbuf_q0,\n        payloads_address0,\n        payloads_ce0,\n        payloads_we0,\n        payloads_d0,\n        payloads_q0,\n        enable,")
    .gsub(/^reg.*\[11:0\] payloads_address0;/, "output reg   [11:0] payloads_address0;")
    .gsub(/^reg.*payloads_ce0;/,             "output reg    payloads_ce0;")
    .gsub(/^reg.*payloads_we0;/,             "output reg    payloads_we0;")
    .gsub(/^wire.*\[7:0\] payloads_d0;/,       "output wire   [7:0] payloads_d0;")
    .gsub(/^wire.*\[7:0\] payloads_q0;/,       "input wire   [7:0] payloads_q0;")
    .gsub(/always @ \(posedge ap_clk\) begin(.*?)if \(ap_rst_n_inv == 1'b1\) begin/m,
          "always @ (posedge ap_clk or negedge ap_rst_n) begin\n    if (!ap_rst_n) begin")
    .gsub(/always @ \(posedge ap_clk\) begin(.*?)end/m,
          "always @ (posedge ap_clk or negedge ap_rst_n) begin\nif (!ap_rst_n) begin\\1end else begin\\1end\nend")
  File.write(filename, fixed)
end

Dir.glob(SRCDIR+'ros2_fifo_*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/always @ \(posedge clk\) begin\s+if \(reset == 1'b1\)/m,
          "always @ (posedge clk or posedge reset) begin\n    if (reset)")
    .gsub(/always @ \(posedge clk\)$/m,
          "always @ (posedge clk or posedge reset)")
    .gsub(/if \(ce\)/m,
          "if (reset) begin\nfor (i=0;i<DEPTH;i=i+1) SRL_SIG[i] <= 0;\nend\nelse if (ce)")
    .gsub(/clk,\s+data,/m,
          "clk,\n    reset,\n    data,")
    .gsub(/^input ce;/m,
          "input ce;\ninput reset;")
    .gsub(/.clk\(clk\),\s+.data\(shiftReg_data\),/m,
          ".clk(clk),\n    .reset(reset),\n    .data(shiftReg_data),")
    .gsub(/mOutPtr = (.*?);/,
          "mOutPtr;")
    .gsub(/internal_empty_n = (.*?);/,
          "internal_empty_n;")
    .gsub(/internal_full_n = (.*?);/,
          "internal_full_n;")
  File.write(filename, fixed)
end

Dir.glob(SRCDIR+'*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/bx/, "b0")
    .gsub(/\/\/ power-on initialization(.*?)^end$/m, "")
  File.write(filename, fixed)
end
