// Copyright (c) 2021-2025 AXE, Inc.
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

void get_sedp_reader_tbl_flags(uint8_t *flags, const sedp_reader_tbl_t *tbl,
                               unsigned int idx);

void set_sedp_reader_tbl_liveliness_and_guid_prefix(
    bool alive, const uint8_t guid_prefix[12], sedp_reader_tbl_t *tbl,
    unsigned int idx);

void set_sedp_reader_tbl_liveliness_and_guid_prefix_unknown(
    bool alive, sedp_reader_tbl_t *tbl, unsigned int idx);

void get_sedp_reader_tbl_children(hls_uint<APP_READER_MAX> *children,
                                  const sedp_reader_tbl_t  *tbl,
                                  unsigned int              idx);
void set_sedp_reader_tbl_children(hls_uint<APP_READER_MAX> children,
                                  sedp_reader_tbl_t *tbl, unsigned int idx);
