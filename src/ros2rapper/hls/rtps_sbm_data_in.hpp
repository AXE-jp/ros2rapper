#pragma once

#include "hls.hpp"

void rtps_sbm_data_in(hls_stream<hls_uint<10>> &in,
                      hls_stream<hls_uint<1>>  &out_status_info,
                      hls_stream<hls_uint<10>> &out_spdp_reader,
                      hls_stream<hls_uint<10>> &out_sedp_reader,
                      hls_stream<hls_uint<10>> &out_app_reader);
