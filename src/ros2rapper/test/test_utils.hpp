// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "endpoint.hpp"
#include "ros2.hpp"
#include <cstdint>

// Set topic data:
//   topic_name[id]     <- topic_name_0
//   topic_name_len[id] <- topic_name_len_0
//   type_name[id]      <- type_name_0
//   type_name_len[id]  <- type_name_len_0
void setup_topic_data(int id, uint8_t topic_name[][MAX_TOPIC_NAME_LEN],
                      uint8_t topic_name_len[],
                      uint8_t type_name[][MAX_TOPIC_TYPE_NAME_LEN],
                      uint8_t type_name_len[], const uint8_t topic_name_0[],
                      uint8_t topic_name_len_0, const uint8_t type_name_0[],
                      uint8_t type_name_len_0);

bool is_sedp_endpoint_alive(const sedp_reader_tbl_t *tbl, unsigned int idx);

// Get alive, builtin_pubrd_acknack_req and builtin_subrd_acknack_req
void get_sedp_reader_tbl_flags(uint8_t *flags, const sedp_reader_tbl_t *tbl,
                               unsigned int idx);

// These two functions also change
// - builtin_pubrd_acknack_req,
// - builtin_subrd_acknack_req,
// - initial_send_counter, and
// - udp_port
// to zero.
void set_sedp_reader_tbl_liveliness_and_guid_prefix(
    bool alive, const uint8_t guid_prefix[12], sedp_reader_tbl_t *tbl,
    unsigned int idx);
void set_sedp_reader_tbl_liveliness_and_guid_prefix_unknown(
    bool alive, sedp_reader_tbl_t *tbl, unsigned int idx);

// These two functions do not work in CWB if APP_READER_MAX > 64.
void get_sedp_reader_tbl_children(hls_uint<APP_READER_MAX> *children,
                                  const sedp_reader_tbl_t  *tbl,
                                  unsigned int              idx);
void set_sedp_reader_tbl_children(hls_uint<APP_READER_MAX> children,
                                  sedp_reader_tbl_t *tbl, unsigned int idx);

sedp_reader_id_t get_sedp_reader_cnt(const sedp_reader_tbl_t *tbl);
app_reader_id_t  get_app_reader_cnt(const app_reader_tbl_t *tbl);

void call_ros2_in(hls_stream<rtps_data_t> &in,
                  sedp_reader_tbl_t       *sedp_reader_tbl,
                  app_reader_tbl_t        *app_reader_tbl,
                  hls_uint<PUB_TOPICS_MAX> pub_enable,
                  hls_uint<SUB_TOPICS_MAX> sub_enable, int64_t timestamp_i64,
                  hls_uint<2> *spdp_initial_send_counter);

void call_remove_dead_endpoints(sedp_reader_id_t   id,
                                sedp_reader_tbl_t *sedp_reader_tbl,
                                app_reader_tbl_t  *app_reader_tbl,
                                int64_t            timestamp_i64);
