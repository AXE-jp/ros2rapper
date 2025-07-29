// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`define ROS2CLK_HZ 80_000_000

`define ROS2_MAX_NODE_NAME_LEN        32
`define ROS2_MAX_TOPIC_NAME_LEN       32
`define ROS2_MAX_TOPIC_TYPE_NAME_LEN  64
`define ROS2_MAX_APP_DATA_LEN         1024
`define ROS2_APP_DATA_LEN_WIDTH       11

// `define ROS2_PUB_DATA_FF
`define ROS2_PUB_DATA_RAM

`define PAYLOADSMEM_DEPTH   2960
`define PAYLOADSMEM_AWIDTH  ($clog2(`PAYLOADSMEM_DEPTH))

`define UDP_RXBUF_AWIDTH 6
`define UDP_TXBUF_AWIDTH 6
