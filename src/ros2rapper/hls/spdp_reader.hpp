#pragma once

#include "hls.hpp"
#include <cstdint>

typedef struct {
    hls_uint<1> valid;
    uint8_t     ip_addr[4];
    uint8_t     udp_port[2];
    uint8_t     lease_duration[8];
} spdp_reader_t;

void spdp_reader(hls_stream<hls_uint<10>> &in, hls_stream<spdp_reader_t> &out,
                 const uint8_t reader_ip_addr[4], const uint8_t subnet_mask[4],
                 uint16_t port_num_seed);
