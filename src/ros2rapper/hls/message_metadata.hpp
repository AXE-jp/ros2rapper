// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MESSAGE_METADATA
#define MESSAGE_METADATA

#include "duration.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include <cstdint>

typedef enum {
    MSG_TYPE_SPDP,
    MSG_TYPE_SEDP_PUB,
    MSG_TYPE_SEDP_SUB,
    MSG_TYPE_SEDP_HEARTBEAT_PUB,
    MSG_TYPE_SEDP_HEARTBEAT_SUB,
    MSG_TYPE_SEDP_ACKNACK_PUB,
    MSG_TYPE_SEDP_ACKNACK_SUB,
    MSG_TYPE_APP,
    MSG_TYPE_RAWUDP
} message_type_t;

typedef hls_uint<2> topic_id_t;

static_assert(PUB_TOPICS_MAX <= 4,
              "topic_id_t should be able to represent PUB_TOPICS_MAX - 1.");
static_assert(SUB_TOPICS_MAX <= 4,
              "topic_id_t should be able to represent SUB_TOPICS_MAX - 1.");

typedef struct {
    uint8_t  metatraffic_port[2];
    uint8_t  default_port[2];
    duration lease_duration;
} spdp_message_metadata_t;

typedef struct {
    uint8_t reader_guid_prefix[12];
    int64_t seqnum;
    uint8_t usertraffic_port[2];
    uint8_t app_entity_id[4];
} sedp_message_metadata_t;

typedef struct {
    uint8_t  reader_guid_prefix[12];
    int64_t  first_seqnum;
    int64_t  last_seqnum;
    uint32_t cnt;
} sedp_heartbeat_message_metadata_t;

typedef struct {
    uint8_t  reader_guid_prefix[12];
    uint8_t  snstate_base;
    bool     snstate_empty;
    uint32_t cnt;
} sedp_acknack_message_metadata_t;

typedef struct {
    uint8_t reader_guid_prefix[12];
    uint8_t reader_entity_id[4];
    uint8_t writer_entity_id[4];
} app_message_metadata_t;

typedef struct {
    message_type_t message_type;
    topic_id_t     topic_id;
    uint8_t        dst_addr[4];
    uint8_t        ttl;
    uint16_t       ip_data_real_len;
    uint8_t        dst_port[4];
    uint16_t       udp_data_len;
    union {
        spdp_message_metadata_t           spdp;
        sedp_message_metadata_t           sedp;
        sedp_heartbeat_message_metadata_t sedp_hb;
        sedp_acknack_message_metadata_t   sedp_an;
        app_message_metadata_t            app;
    } rtps;
} message_metadata_t;

#endif // !MESSAGE_METADATA
