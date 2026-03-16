#pragma once

#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

#define RTPS_IN_DATA_SBM_ID      GUID_PREFIX_SIZE
#define RTPS_IN_DATA_SBM_FLAGS   (GUID_PREFIX_SIZE + 1)
#define RTPS_IN_DATA_SBM_PAYLOAD (GUID_PREFIX_SIZE + 2)

typedef struct {
    hls_uint<4> index;
    hls_uint<9> data;
} rtps_in_data_t;

void rtps_in(hls_stream<hls_uint<9>> &in, hls_stream<rtps_in_data_t> &out,
             hls_uint<1>   enable,
             const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE]);
