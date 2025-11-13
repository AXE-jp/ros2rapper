// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>

void hls_pub(int16_t pub_data_seed, int16_t pub_app_data[512])
{
#pragma HLS interface ap_ctrl_hs port=return
#pragma HLS interface ap_none port=pub_data_seed
#pragma HLS interface ap_memory port=pub_app_data storage_type=ram_1p
    for (auto j = 0; j < 512; j++) {
        pub_app_data[j] = pub_data_seed + j;
    }
}
