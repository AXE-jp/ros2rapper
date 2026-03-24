#pragma once

#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

bool spdp_set_locator(uint16_t offset, uint8_t data, bool param_le,
                      uint8_t ip_addr[4], uint8_t udp_port[2],
                      const uint8_t reader_ip_addr[4],
                      const uint8_t subnet_mask[4], uint16_t port_num_seed);

void spdp_reader(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                 const uint8_t reader_ip_addr[4], const uint8_t subnet_mask[4],
                 uint16_t      port_num_seed,
                 const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]);
