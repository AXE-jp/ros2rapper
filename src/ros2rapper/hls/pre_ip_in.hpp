#pragma once

#include "hls.hpp"
#include <cstdint>

void pre_ip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out);
