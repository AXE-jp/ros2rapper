// synthesis target
//`define TARGET_ASIC
//`define TARGET_XILINX

`define MAC_TX_FIFO_DEPTH  512
`define MAC_RX_FIFO_DEPTH  2048

`define ARP_CACHE_ADDR_WIDTH       4
`define ARP_REQUEST_RETRY_COUNT    4
`define ARP_REQUEST_RETRY_INTERVAL (125000000*2)
`define ARP_REQUEST_TIMEOUT        (125000000*30)

`define EXT_TX_FIFO_DEPTH  4096
`define EXT_RX_FIFO_DEPTH  4096
