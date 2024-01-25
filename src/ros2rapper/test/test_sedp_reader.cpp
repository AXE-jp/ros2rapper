// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include <cstdint>
#include <cstdio>

#include "hls.hpp"
#include "ros2.hpp"
#include "endpoint.hpp"
#include "sedp.hpp"



// PC is listener
static const unsigned char	rtps_pkt_d0[] = {
0x52,0x54,0x50,0x53,0x02,0x03,0x01,0x0f,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x0e,0x01,0x0c,0x00,
0x01,0x0f,0x37,0xad,0xde,0x09,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,
0xd1,0x66,0x5c,0x65,0x00,0x4c,0x91,0xe8,0x15,0x05,0x50,0x01,0x00,0x00,0x10,0x00,
0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,
0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x28,0x00,
0x22,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,
0x67,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x52,0x65,
0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x00,0x07,0x00,0x38,0x00,0x32,0x00,0x00,0x00,
0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,0x65,0x73,0x3a,0x3a,
0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x47,0x65,0x74,0x50,0x61,
0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,
0x5f,0x00,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x02,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x02,0x04,0x15,0x00,0x04,0x00,
0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,
0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,
0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,
0xd1,0x66,0x5c,0x65,0x00,0xe4,0x97,0xe8,0x15,0x05,0x58,0x01,0x00,0x00,0x10,0x00,
0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,
0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x2c,0x00,
0x27,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,
0x67,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x5f,0x74,0x79,
0x70,0x65,0x73,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x07,0x00,0x3c,0x00,
0x36,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x47,
0x65,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x54,0x79,0x70,0x65,0x73,
0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x00,0x70,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x04,0x04,
0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x04,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,
0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0x68,0x9d,0xe8,
0x15,0x05,0x50,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,
0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,
0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x01,0xc1,0x05,0x00,0x28,0x00,0x22,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,
0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,0x73,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,
0x6d,0x65,0x74,0x65,0x72,0x73,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x00,
0x07,0x00,0x38,0x00,0x32,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,
0x72,0x66,0x61,0x63,0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,
0x5f,0x3a,0x3a,0x53,0x65,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,
0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x00,0x70,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x06,0x04,
0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x06,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,
0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0x1c,0xa3,0xe8,
0x15,0x05,0x64,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,
0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,
0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x01,0xc1,0x05,0x00,0x34,0x00,0x2d,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,
0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,0x73,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,
0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x61,0x74,0x6f,0x6d,0x69,0x63,0x61,0x6c,0x6c,
0x79,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x00,0x00,0x07,0x00,0x40,0x00,
0x3c,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x53,
0x65,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x41,0x74,0x6f,0x6d,
0x69,0x63,0x61,0x6c,0x6c,0x79,0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x5f,0x00,
0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x08,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x08,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,
0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,
0x00,0x00,0xa9,0xe8,0x15,0x05,0x58,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,
0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x03,0x00,0x00,
0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x2c,0x00,0x27,0x00,0x00,0x00,
0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,0x64,0x65,0x73,0x63,
0x72,0x69,0x62,0x65,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x52,
0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x07,0x00,0x3c,0x00,0x37,0x00,0x00,0x00,
0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,0x65,0x73,0x3a,0x3a,
0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x44,0x65,0x73,0x63,0x72,
0x69,0x62,0x65,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x52,0x65,
0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0a,0x04,0x5a,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0a,0x04,
0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,
0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,
0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0xe0,0xae,0xe8,0x15,0x05,0x50,0x01,
0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,
0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,
0x05,0x00,0x28,0x00,0x23,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,
0x6e,0x65,0x72,0x2f,0x6c,0x69,0x73,0x74,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,
0x65,0x72,0x73,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x07,0x00,0x38,0x00,
0x33,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x4c,
0x69,0x73,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x52,0x65,
0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0c,0x04,0x5a,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0c,0x04,
0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,
0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,
0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0x3c,0xc1,0xe8,0x15,0x05,0x38,0x01,
0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,
0x07,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,
0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,
0x05,0x00,0x18,0x00,0x14,0x00,0x00,0x00,0x72,0x74,0x2f,0x70,0x61,0x72,0x61,0x6d,
0x65,0x74,0x65,0x72,0x5f,0x65,0x76,0x65,0x6e,0x74,0x73,0x00,0x07,0x00,0x30,0x00,
0x2b,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x6d,0x73,0x67,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x50,
0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x45,0x76,0x65,0x6e,0x74,0x5f,0x00,0x00,
0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x0f,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x0f,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,
0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,
0x00,0xd8,0xc8,0xe8,0x15,0x05,0x24,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,
0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x03,0x00,0x00,
0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x10,0x00,0x0b,0x00,0x00,0x00,
0x72,0x74,0x2f,0x63,0x68,0x61,0x74,0x74,0x65,0x72,0x00,0x00,0x07,0x00,0x24,0x00,
0x1d,0x00,0x00,0x00,0x73,0x74,0x64,0x5f,0x6d,0x73,0x67,0x73,0x3a,0x3a,0x6d,0x73,
0x67,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x53,0x74,0x72,0x69,0x6e,0x67,0x5f,
0x00,0x00,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x10,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x10,0x04,0x15,0x00,0x04,0x00,
0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,
0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
};
static const uint32_t	rtps_pkt_d0_size = sizeof(rtps_pkt_d0);


// PC is listener
static const uint8_t	rtps_pkt_d1[] = {
0x52,0x54,0x50,0x53,0x02,0x03,0x01,0x0f,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x0e,0x01,0x0c,0x00,
0x01,0x0f,0x37,0xad,0xde,0x09,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,
0xd1,0x66,0x5c,0x65,0x00,0x4c,0x91,0xe8,0x15,0x05,0x50,0x01,0x00,0x00,0x10,0x00,
0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,
0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x28,0x00,
0x22,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,
0x67,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x52,0x65,
0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x00,0x07,0x00,0x38,0x00,0x32,0x00,0x00,0x00,
0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,0x65,0x73,0x3a,0x3a,
0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x47,0x65,0x74,0x50,0x61,
0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,
0x5f,0x00,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x02,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x02,0x04,0x15,0x00,0x04,0x00,
0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,
0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,
0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,
0xd1,0x66,0x5c,0x65,0x00,0xe4,0x97,0xe8,0x15,0x05,0x58,0x01,0x00,0x00,0x10,0x00,
0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,
0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x2c,0x00,
0x27,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,
0x67,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x5f,0x74,0x79,
0x70,0x65,0x73,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x07,0x00,0x3c,0x00,
0x36,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x47,
0x65,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x54,0x79,0x70,0x65,0x73,
0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x00,0x70,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x04,0x04,
0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x04,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,
0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0x68,0x9d,0xe8,
0x15,0x05,0x50,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,
0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,
0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x01,0xc1,0x05,0x00,0x28,0x00,0x22,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,
0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,0x73,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,
0x6d,0x65,0x74,0x65,0x72,0x73,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x00,
0x07,0x00,0x38,0x00,0x32,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,
0x72,0x66,0x61,0x63,0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,
0x5f,0x3a,0x3a,0x53,0x65,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,
0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x00,0x70,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x06,0x04,
0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x06,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,
0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0x1c,0xa3,0xe8,
0x15,0x05,0x64,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,
0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,
0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x01,0xc1,0x05,0x00,0x34,0x00,0x2d,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,
0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,0x73,0x65,0x74,0x5f,0x70,0x61,0x72,0x61,
0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x61,0x74,0x6f,0x6d,0x69,0x63,0x61,0x6c,0x6c,
0x79,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x00,0x00,0x07,0x00,0x40,0x00,
0x3c,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x53,
0x65,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x41,0x74,0x6f,0x6d,
0x69,0x63,0x61,0x6c,0x6c,0x79,0x5f,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x5f,0x00,
0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x08,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x08,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,
0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,
0x00,0x00,0xa9,0xe8,0x15,0x05,0x58,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,
0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x03,0x00,0x00,
0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x2c,0x00,0x27,0x00,0x00,0x00,
0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,0x6e,0x65,0x72,0x2f,0x64,0x65,0x73,0x63,
0x72,0x69,0x62,0x65,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x52,
0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x07,0x00,0x3c,0x00,0x37,0x00,0x00,0x00,
0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,0x65,0x73,0x3a,0x3a,
0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x44,0x65,0x73,0x63,0x72,
0x69,0x62,0x65,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x52,0x65,
0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0a,0x04,0x5a,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0a,0x04,
0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,
0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,
0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0xe0,0xae,0xe8,0x15,0x05,0x50,0x01,
0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,
0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,
0x05,0x00,0x28,0x00,0x23,0x00,0x00,0x00,0x72,0x71,0x2f,0x6c,0x69,0x73,0x74,0x65,
0x6e,0x65,0x72,0x2f,0x6c,0x69,0x73,0x74,0x5f,0x70,0x61,0x72,0x61,0x6d,0x65,0x74,
0x65,0x72,0x73,0x52,0x65,0x71,0x75,0x65,0x73,0x74,0x00,0x00,0x07,0x00,0x38,0x00,
0x33,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x73,0x72,0x76,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x4c,
0x69,0x73,0x74,0x50,0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x73,0x5f,0x52,0x65,
0x71,0x75,0x65,0x73,0x74,0x5f,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0c,0x04,0x5a,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0c,0x04,
0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,
0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,
0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,0x00,0x3c,0xc1,0xe8,0x15,0x05,0x38,0x01,
0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,
0x07,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,
0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,
0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,
0x05,0x00,0x18,0x00,0x14,0x00,0x00,0x00,0x72,0x74,0x2f,0x70,0x61,0x72,0x61,0x6d,
0x65,0x74,0x65,0x72,0x5f,0x65,0x76,0x65,0x6e,0x74,0x73,0x00,0x07,0x00,0x30,0x00,
0x2b,0x00,0x00,0x00,0x72,0x63,0x6c,0x5f,0x69,0x6e,0x74,0x65,0x72,0x66,0x61,0x63,
0x65,0x73,0x3a,0x3a,0x6d,0x73,0x67,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x50,
0x61,0x72,0x61,0x6d,0x65,0x74,0x65,0x72,0x45,0x76,0x65,0x6e,0x74,0x5f,0x00,0x00,
0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x0f,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x0f,0x04,0x15,0x00,0x04,0x00,0x02,0x03,0x00,0x00,
0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x27,0x00,0x08,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,0x02,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,
0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x25,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x09,0x01,0x08,0x00,0xd1,0x66,0x5c,0x65,
0x00,0xd8,0xc8,0xe8,0x15,0x05,0x24,0x01,0x00,0x00,0x10,0x00,0x00,0x00,0x04,0xc7,
0x00,0x00,0x04,0xc2,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x03,0x00,0x00,
0x2f,0x00,0x18,0x00,0x01,0x00,0x00,0x00,0xf5,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xa8,0x01,0x0a,0x43,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x01,0xc1,0x05,0x00,0x10,0x00,0x0b,0x00,0x00,0x00,
0x72,0x74,0x2f,0x63,0x68,0x61,0x74,0x74,0x65,0x72,0x00,0x00,0x07,0x00,0x24,0x00,
0x1d,0x00,0x00,0x00,0x73,0x74,0x64,0x5f,0x6d,0x73,0x67,0x73,0x3a,0x3a,0x6d,0x73,
0x67,0x3a,0x3a,0x64,0x64,0x73,0x5f,0x3a,0x3a,0x53,0x74,0x72,0x69,0x6e,0x67,0x5f,
0x00,0x00,0x00,0x00,0x70,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,0x36,0x30,0x00,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x10,0x04,0x5a,0x00,0x10,0x00,0x01,0x0f,0xbd,0x21,
0x36,0x30,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x10,0x04,0x15,0x00,0x04,0x00,
0x02,0x03,0x00,0x00,0x16,0x00,0x04,0x00,0x01,0x0f,0x00,0x00,0x1d,0x00,0x04,0x00,
0x00,0x00,0x00,0x00,0x23,0x00,0x08,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,
0x27,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x0c,0x00,
0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1a,0x00,0x0c,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9a,0x99,0x99,0x19,0x2b,0x00,0x08,0x00,
0xff,0xff,0xff,0x7f,0xff,0xff,0xff,0xff,0x1f,0x00,0x04,0x00,0x00,0x00,0x00,0x00,
0x25,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
};
static const uint32_t	rtps_pkt_d1_size = sizeof(rtps_pkt_d1);






static void  print_guid( const uint8_t  guid[] )
{
	int	ii;
	for( ii = 0 ; ii < 12 ; ii += 4 ){
		printf("%02X%02X%02X%02X ", guid[ii+0], guid[ii+1], guid[ii+2], guid[ii+3]);
	}
	printf("\n");
}
static void  print_entity_id( const uint8_t  ent_id[4] )
{
	int	ii;
	for( ii = 0 ; ii < 4 ; ii += 4 ){
		printf("%02X%02X%02X%02X ", ent_id[ii+0], ent_id[ii+1], ent_id[ii+2], ent_id[ii+3]);
	}
	printf("\n");
}
static void  print_ipaddr( const uint8_t  ipaddr[] )
{
	printf("%d.%d.%d.%d\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
}
static void  print_port( const uint8_t  portval[] )
{
	uint16_t	wval;

	wval = portval[0];
	wval <<= 8;
	wval |= portval[1];
	printf("%d\n", wval);
}

int main()
{
	hls_stream<hls_uint<9>> in;
	hls_uint<9>	x;
	int	ii;
	int n = 0;

	const uint8_t	own_guid_prefix[12] = {
			0x01, 0x0f, 0x37, 0xad,
			0xde, 0x09, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00
			};
	const uint8_t	sedp_pub_reader_entity_id[4] = {
			0x00, 0x00, 0x03, 0xc7
			};
	const uint8_t	sedp_sub_reader_entity_id[4] = {
			0x00, 0x00, 0x04, 0xc7
			};

	app_reader_id_t app_reader_cnt = (app_reader_id_t)0;
	app_endpoint app_reader_tbl[APP_READER_MAX];

	const uint16_t	port_num_seed = 7400;

	const uint8_t	pub_topic_name[] = "rt/chatter";
	const uint8_t	pub_topic_name_len = sizeof(pub_topic_name);
	const uint8_t	pub_type_name[] = "std_msgs::msg::dds_::String_";
	const uint8_t	pub_type_name_len = sizeof(pub_type_name);

	const uint8_t	sub_topic_name[] = "rt/chatter";
	const uint8_t	sub_topic_name_len = sizeof(sub_topic_name);
	const uint8_t	sub_type_name[] = "std_msgs::msg::dds_::String_";
	const uint8_t	sub_type_name_len = sizeof(sub_type_name);

	/*****************************************************/

	for( ii = 0 ; ii < sizeof(rtps_pkt_d0) ; ii++ ){
		x = rtps_pkt_d0[ii];
		if (ii == (sizeof(rtps_pkt_d0) - 1)) {
			x |= 0x100;
		}
		in.write( x );

		sedp_reader(
			in,
			app_reader_cnt,
			app_reader_tbl,
			port_num_seed,
			own_guid_prefix,
			pub_topic_name,
			pub_topic_name_len,
			pub_type_name,
			pub_type_name_len,
			sub_topic_name,
			sub_topic_name_len,
			sub_type_name,
			sub_type_name_len
		);
	}

	std::cout << "reader_cnt = " << app_reader_cnt << std::endl;
	for( ii = 0 ; ii < app_reader_cnt ; ii++ ){
		printf("tbl[%d] ****\n", ii);
		print_guid( app_reader_tbl[ii].guid_prefix );
		print_entity_id( app_reader_tbl[ii].entity_id );
		print_ipaddr( app_reader_tbl[ii].ip_addr );
		print_port( app_reader_tbl[ii].udp_port );
		std::cout << std::hex << app_reader_tbl[ii].ep_type << std::endl;
	}

	/*****************************************************/

	for( ii = 0 ; ii < sizeof(rtps_pkt_d1) ; ii++ ){
		x = rtps_pkt_d1[ii];
		if (ii == (sizeof(rtps_pkt_d1) - 1)) {
			x |= 0x100;
		}
		in.write( x );

		sedp_reader(
			in,
			app_reader_cnt,
			app_reader_tbl,
			port_num_seed,
			own_guid_prefix,
			pub_topic_name,
			pub_topic_name_len,
			pub_type_name,
			pub_type_name_len,
			sub_topic_name,
			sub_topic_name_len,
			sub_type_name,
			sub_type_name_len
		);
	}

	std::cout << "reader_cnt = " << app_reader_cnt << std::endl;
	for( ii = 0 ; ii < app_reader_cnt ; ii++ ){
		printf("tbl[%d] ****\n", ii);
		print_guid( app_reader_tbl[ii].guid_prefix );
		print_entity_id( app_reader_tbl[ii].entity_id );
		print_ipaddr( app_reader_tbl[ii].ip_addr );
		print_port( app_reader_tbl[ii].udp_port );
		std::cout << std::hex << app_reader_tbl[ii].ep_type << std::endl;
	}

	return 0;
}

