// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ENDPOINT_HPP
#define ENDPOINT_HPP

#include "hls.hpp"
#include <cstdint>

#define SEDP_READER_MAX 4

typedef hls_uint<3> sedp_reader_id_t;

struct sedp_endpoint {
  uint8_t ip_addr[4] /* Cyber array=EXPAND, array_index=const */;
  uint8_t udp_port[2] /* Cyber array=EXPAND, array_index=const */;
  uint8_t guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
};

#define APP_READER_MAX 4

typedef hls_uint<3> app_reader_id_t;

using ep_type_t = hls_uint<2>;

const ep_type_t APP_EP_PUB = 0x01;
const ep_type_t APP_EP_SUB = 0x02;

struct app_endpoint {
  uint8_t ip_addr[4] /* Cyber array=EXPAND, array_index=const */;
  uint8_t udp_port[2] /* Cyber array=EXPAND, array_index=const */;
  uint8_t guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
  uint8_t entity_id[4] /* Cyber array=EXPAND, array_index=const */;
  ep_type_t ep_type;
};

#endif // !ENDPOINT_HPP
