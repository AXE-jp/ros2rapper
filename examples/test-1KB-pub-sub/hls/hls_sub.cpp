// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>

void hls_sub(uint16_t sub_app_data[512], bool *sub_data_result)
{
#pragma HLS interface ap_ctrl_none port=return
#pragma HLS interface ap_memory port=sub_app_data latency=2 storage_type=rom_1p
#pragma HLS interface ap_hs port=sub_data_result
    bool valid = true;
    uint16_t base = sub_app_data[0];
    for (auto j = 1; j < 512; j++) {
        if (sub_app_data[j] != (base + j)) {
            valid = false;
        }
    }
    *sub_data_result = valid;
}
