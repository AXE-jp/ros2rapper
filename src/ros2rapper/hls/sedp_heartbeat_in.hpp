#pragma once

#include "endpoint.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

typedef struct {
    builtin_ep_type_t ep_type;
    uint8_t           first_sn;
    uint8_t           last_sn;
} sedp_heartbeat_in_t;

void sedp_heartbeat_in(hls_stream<hls_uint<10>>        &in,
                       hls_stream<sedp_heartbeat_in_t> &out);
