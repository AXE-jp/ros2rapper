// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`define ROS2CLK_HZ 100_000_000

`define ROS2_MAX_NODE_NAME_LEN        32
`define ROS2_MAX_TOPIC_NAME_LEN       32
`define ROS2_MAX_TOPIC_TYPE_NAME_LEN  64
`define ROS2_MAX_APP_DATA_LEN         64

`define PAYLOADSMEM_DEPTH   2960
`define PAYLOADSMEM_AWIDTH  ($clog2(`PAYLOADSMEM_DEPTH))
