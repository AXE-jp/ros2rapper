// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>

typedef struct {
    int16_t data[512];
} output_t;

void hls_pub(int16_t pub_data_seed, output_t *pub_app_data)
{
#pragma HLS interface ap_ctrl_none port=return
#pragma HLS interface ap_none port=pub_data_seed
#pragma HLS array_reshape variable=pub_app_data->data complete dim=1
#pragma HLS interface ap_hs port=pub_app_data
    output_t buf;
#pragma HLS array_partition variable=buf.data complete dim=1
    for (auto j = 0; j < 512; j++) {
        buf.data[j] = pub_data_seed + j;
    }
    *pub_app_data = buf;
}
