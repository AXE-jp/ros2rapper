// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef REMOVE_ENDPOINTS_HPP
#define REMOVE_ENDPOINTS_HPP

#include "common.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include <cstdint>

void remove_sedp_endpoint(sedp_reader_id_t   sedp_idx,
                          sedp_reader_tbl_t *sedp_reader_tbl,
                          app_endpoint       app_reader_tbl[APP_READER_MAX]);

void update_liveliness(hls_uint<9> in, hls_stream<rtps_data_t> &out,
                       const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE]);

void remove_dead_endpoints(sedp_reader_id_t   id,
                           sedp_reader_tbl_t *sedp_reader_tbl,
                           app_endpoint       app_reader_tbl[APP_READER_MAX],
                           int64_t            timestamp_i64);

#endif // !REMOVE_ENDPOINTS_HPP
