// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include <cstdint>
#include <cstdio>

#include "app.hpp"
#include "hls.hpp"
#include "ros2.hpp"

static const unsigned char rtps_pkt_data[] = {
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03,             /* .+RTPS.. */
    0x01, 0x0f, 0x01, 0x0f, 0xbd, 0x21, 0x60, 0x14, /* .....!`. */
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x01, /* ........ */
    0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, /* ....7... */
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x09, 0x01, /* ........ */
    0x08, 0x00, 0x6d, 0x9e, 0x55, 0x65, 0x00, 0xfc, /* ..m.Ue.. */
    0x14, 0xc0, 0x15, 0x05, 0x2c, 0x00, 0x00, 0x00, /* ....,... */
    0x10, 0x00, 0x00, 0x00, 0x10, 0x04, 0x00, 0x00, /* ........ */
    0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x27, 0x03, /* ......'. */
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x0d, 0x00, /* ........ */
    0x00, 0x00, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, /* ..Hello  */
    0x77, 0x6f, 0x72, 0x6c, 0x64, 0x21, 0x00, 0x00, /* world!.. */
    0x00, 0x00                                      /* .. */
};

int main() {
    hls_stream<hls_uint<9>> in;
    hls_uint<9>             x;
    int                     ii;
    int                     n = 0;

    const uint8_t c_reader_guid_prefix[12] = {
        0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    const uint8_t c_reader_entity_id[4] = {0x00, 0x00, 0x10, 0x04};

    uint32_t       app_data_len = 0;
    uint8_t        app_data[64] = {0};
    const uint32_t app_data_max_len = sizeof(app_data);

    for (ii = 0; ii < sizeof(rtps_pkt_data); ii++) {
        x = rtps_pkt_data[ii];
        if (ii == (sizeof(rtps_pkt_data) - 1)) {
            x |= 0x100;
        }
        in.write(x);

        app_reader(in, c_reader_guid_prefix, c_reader_entity_id, app_data,
                   app_data_len, app_data_max_len);
    }

    if (app_data_len > 0) {
        printf("app_data_len = %d\n", app_data_len);
        for (ii = 0; ii < app_data_max_len; ii++) {
            printf("0x%02X ", app_data[ii]);
            if ((ii & 0x0F) == 0x0F) {
                printf("\n");
            }
        }
    }

    return 0;
}
