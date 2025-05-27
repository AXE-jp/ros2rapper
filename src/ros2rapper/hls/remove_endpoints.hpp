// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef REMOVE_ENDPOINTS_HPP
#define REMOVE_ENDPOINTS_HPP

#include "common.hpp"

void update_liveliness(hls_uint<9>   in,
                       const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE],
                       sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
                       app_endpoint  app_reader_tbl[APP_READER_MAX],
                       bool *reading_rtps_message, int64_t timestamp_i64);

void remove_dead_endpoints(sedp_reader_id_t tx_progress,
                           sedp_endpoint    sedp_reader_tbl[SEDP_READER_MAX],
                           app_endpoint     app_reader_tbl[APP_READER_MAX],
                           int64_t          timestamp_i64);

#endif // !REMOVE_ENDPOINTS_HPP
