// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

// Target
// Define TARGET_XILINX or TARGET_ASIC.

`ifdef TARGET_XILINX
  // Clock input style ("BUFG", "BUFR", "BUFIO", "BUFIO2")
  // Use BUFR for Virtex-5, Virtex-6, 7-series
  // Use BUFG for Ultrascale
  // Use BUFIO2 for Spartan-6

  //`define XILINX_CLKIN_STYLE_BUFG
  //`define XILINX_CLKIN_STYLE_BUFR
  //`define XILINX_CLKIN_STYLE_BUFIO
  //`define XILINX_CLKIN_STYLE_BUFIO2

  `ifndef XILINX_CLKIN_STYLE_BUFG
    `ifndef XILINX_CLKIN_STYLE_BUFR
      `ifndef XILINX_CLKIN_STYLE_BUFIO
        `ifndef XILINX_CLKIN_STYLE_BUFIO2
          "Error: Define Clock Input Style!"
        `endif
      `endif
    `endif
  `endif
`endif

`define MAC_TX_FIFO_DEPTH  2048
`define MAC_RX_FIFO_DEPTH  2048
`define EXT_TX_FIFO_DEPTH  2
`define EXT_RX_FIFO_DEPTH  2048
`define UDP_PAYLOAD_FIFO_DEPTH 2048
`define UDP_HEADER_FIFO_DEPTH  2

`define ARP_CACHE_ADDR_WIDTH       7
