#pragma once

#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

void rtps_in_send_output(hls_stream<hls_uint<10>> &out, uint8_t data, bool end,
                         bool valid);

void rtps_in(hls_stream<hls_uint<9>>  &in,
             hls_stream<hls_uint<10>> &out_guid_prefix,
             hls_stream<hls_uint<10>> &out_sbm_heartbeat,
             hls_stream<hls_uint<10>> &out_sbm_data, hls_uint<1> enable,
             const uint8_t guid_prefix[GUID_PREFIX_SIZE]);
