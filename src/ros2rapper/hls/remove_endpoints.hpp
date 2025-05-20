// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef RM_PARTICIPANT_HPP
#define RM_PARTICIPANT_HPP

#include "common.hpp"

void update_liveliness(hls_uint<9> in, const config_t *conf,
                       sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
                       app_endpoint  app_reader_tbl[APP_READER_MAX],
                       bool         *reading_rtps_message);

void collect_dead_endpoint(hls_uint<2>       tx_progress,
                           sedp_reader_id_t &sedp_reader_cnt,
                           sedp_endpoint     sedp_reader_tbl[SEDP_READER_MAX],
                           app_reader_id_t  &app_reader_cnt,
                           app_endpoint      app_reader_tbl[APP_READER_MAX],
                           uint32_t sedp_pub_heartbeat_cnt[SEDP_READER_MAX],
                           uint32_t sedp_sub_heartbeat_cnt[SEDP_READER_MAX],
                           uint32_t sedp_pub_acknack_cnt[SEDP_READER_MAX],
                           uint32_t sedp_sub_acknack_cnt[SEDP_READER_MAX]);

#endif // !RM_PARTICIPANT_HPP
