#pragma once

#include "hls.hpp"
#include "ip.hpp"
#include <cstdint>

void ip_in(
    hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
    uint8_t  ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    uint32_t fragment_expiration, bool ignore_checksum, bool *parity_error);
