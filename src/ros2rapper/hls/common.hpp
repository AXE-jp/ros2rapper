// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef COMMON_HPP
#define COMMON_HPP

#include "app.hpp"
#include "ip.hpp"
#include "sedp.hpp"
#include "spdp.hpp"
#include "udp.hpp"

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) > (y) ? (y) : (x))

#define TARGET_CLOCK_FREQ     100000000 // Arty A7-100T: 100 MHz
#define TARGET_PARTICIPANT_ID 1

#define RAWUDP_RXBUF_LEN 256
#define RAWUDP_TXBUF_LEN 256

#define MAX_RAWUDP_OUT_PAYLOAD_LEN (RAWUDP_TXBUF_LEN - 12)
#define MAX_RAWUDP_OUT_UDP_PKT_LEN (UDP_HDR_SIZE + MAX_RAWUDP_OUT_PAYLOAD_LEN)
#define MAX_RAWUDP_OUT_IP_PKT_LEN  (IP_HDR_SIZE + MAX_RAWUDP_OUT_UDP_PKT_LEN)

#define TX_BUF_LEN                                                             \
    (MAX(MAX(MAX(MAX(MAX(MAX_RAWUDP_OUT_IP_PKT_LEN, SPDP_WRITER_IP_PKT_LEN),   \
                     SEDP_WRITER_IP_PKT_LEN),                                  \
                 SEDP_HEARTBEAT_IP_PKT_LEN),                                   \
             SEDP_ACKNACK_IP_PKT_LEN),                                         \
         APP_WRITER_IP_PKT_LEN(MAX_APP_DATA_LEN)))

#define MAX_TX_UDP_PAYLOAD_LEN (TX_BUF_LEN - (IP_HDR_SIZE + UDP_HDR_SIZE))

#define ENTITYID_APP_WRITER                                                    \
    { 0x00, 0x00, 0x10, 0x03 }
#define ENTITYID_APP_READER                                                    \
    { 0x00, 0x00, 0x10, 0x04 }

#define SBM_ENDIAN_LITTLE
// #define SBM_ENDIAN_BIG

#ifdef SBM_ENDIAN_LITTLE
#define S_BYTE0(x) ((x)&0xff)
#define S_BYTE1(x) (((x) >> 8) & 0xff)
#define L_BYTE0(x) ((x)&0xff)
#define L_BYTE1(x) (((x) >> 8) & 0xff)
#define L_BYTE2(x) (((x) >> 16) & 0xff)
#define L_BYTE3(x) (((x) >> 24) & 0xff)
#endif // SBM_ENDIAN_LITTLE

#ifdef SBM_ENDIAN_BIG
#define S_BYTE0(x) (((x) >> 8) & 0xff)
#define S_BYTE1(x) ((x)&0xff)
#define L_BYTE0(x) (((x) >> 24) & 0xff)
#define L_BYTE1(x) (((x) >> 16) & 0xff)
#define L_BYTE2(x) (((x) >> 8) & 0xff)
#define L_BYTE3(x) ((x)&0xff)
#endif // SBM_ENDIAN_BIG

#endif // !COMMON_HPP
