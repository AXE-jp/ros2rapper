// Target
//`define TARGET_XILINX
//`define TARGET_ASIC

`ifdef TARGET_XILINX
  // Clock input style ("BUFG", "BUFR", "BUFIO", "BUFIO2")
  // Use BUFR for Virtex-5, Virtex-6, 7-series
  // Use BUFG for Ultrascale
  // Use BUFIO2 for Spartan-6

  //`define XILINX_CLKIN_STYLE_BUFG
  `define XILINX_CLKIN_STYLE_BUFR
  //`define XILINX_CLKIN_STYLE_BUFIO
  //`define XILINX_CLKIN_STYLE_BUFIO2
`endif

// ROS2rapper
//`define DISABLE_ROS2ETHER
`define ROS2_MAX_NODE_NAME_LEN        32
`define ROS2_MAX_TOPIC_NAME_LEN       32
`define ROS2_MAX_TOPIC_TYPE_NAME_LEN  64
`define ROS2_MAX_APP_DATA_LEN         64

`define PAYLOADSMEM_DEPTH   2960
`define PAYLOADSMEM_AWIDTH  ($clog2(`PAYLOADSMEM_DEPTH))

// raw UDP datagram
`define UDP_RXBUF_AWIDTH 6
`define UDP_TXBUF_AWIDTH 6

// Ethernet
`define MAC_TX_FIFO_DEPTH  512
`define MAC_RX_FIFO_DEPTH  2048
`define EXT_TX_FIFO_DEPTH  2
`ifdef TARGET_XILINX
    // RX FIFO may overflow due to slow clock frequency,so increase depth.
    `define EXT_RX_FIFO_DEPTH  2048
`else
    `define EXT_RX_FIFO_DEPTH  2048
`endif

// ARP
`define ARP_CACHE_ADDR_WIDTH       4
