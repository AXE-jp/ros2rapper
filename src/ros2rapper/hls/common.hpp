// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef COMMON_HPP
#define COMMON_HPP

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) > (y) ? (y) : (x))

#define TARGET_PARTICIPANT_ID 1

#define ENTITYID_APP_WRITER_LIST                                               \
    {                                                                          \
        {0x00, 0x00, 0x10, 0x03}, {0x00, 0x00, 0x11, 0x03},                    \
            {0x00, 0x00, 0x12, 0x03}, {                                        \
            0x00, 0x00, 0x13, 0x03                                             \
        }                                                                      \
    }
#define ENTITYID_APP_READER_LIST                                               \
    {                                                                          \
        {0x00, 0x00, 0x10, 0x04}, {0x00, 0x00, 0x11, 0x04},                    \
            {0x00, 0x00, 0x12, 0x04}, {                                        \
            0x00, 0x00, 0x13, 0x04                                             \
        }                                                                      \
    }

#define SBM_ENDIAN_LITTLE
// #define SBM_ENDIAN_BIG

#ifdef SBM_ENDIAN_LITTLE
#define S_BYTE0(x) ((x) & 0xff)
#define S_BYTE1(x) (((x) >> 8) & 0xff)
#define L_BYTE0(x) ((x) & 0xff)
#define L_BYTE1(x) (((x) >> 8) & 0xff)
#define L_BYTE2(x) (((x) >> 16) & 0xff)
#define L_BYTE3(x) (((x) >> 24) & 0xff)
#endif // SBM_ENDIAN_LITTLE

#ifdef SBM_ENDIAN_BIG
#define S_BYTE0(x) (((x) >> 8) & 0xff)
#define S_BYTE1(x) ((x) & 0xff)
#define L_BYTE0(x) (((x) >> 24) & 0xff)
#define L_BYTE1(x) (((x) >> 16) & 0xff)
#define L_BYTE2(x) (((x) >> 8) & 0xff)
#define L_BYTE3(x) ((x) & 0xff)
#endif // SBM_ENDIAN_BIG

#endif // !COMMON_HPP
