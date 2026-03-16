#pragma once

#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

void rtps_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
             bool enable, const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE],
             uint8_t guid_prefix_out[GUID_PREFIX_SIZE], uint8_t *sbm_id_out,
             uint8_t *sbm_flags_out, uint16_t *sbm_len_out);
