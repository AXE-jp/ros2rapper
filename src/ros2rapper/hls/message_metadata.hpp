// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MESSAGE_METADATA_HPP
#define MESSAGE_METADATA_HPP

#include "duration.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "timestamp.hpp"
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
    message_type_t message_type;
    topic_id_t     topic_id;
    uint8_t        dst_addr[4] /* Cyber array=EXPAND */;
    uint8_t        dst_port[4] /* Cyber array=EXPAND */;
    timestamp      now;
    uint8_t        rtps_data[32] /* Cyber array=EXPAND */;
} message_metadata_t;

void serialize_spdp_metadata(const uint8_t       metatraffic_port[2],
                             const uint8_t       default_port[2],
                             duration            lease_duration,
                             message_metadata_t *msg_metadata);
void deserialize_spdp_metadata(uint8_t                   metatraffic_port[2],
                               uint8_t                   default_port[2],
                               duration                 *lease_duration,
                               const message_metadata_t *msg_metadata);

void serialize_sedp_metadata(const uint8_t reader_guid_prefix[12],
                             int64_t seqnum, const uint8_t usertraffic_port[2],
                             const uint8_t       app_entity_id[4],
                             message_metadata_t *msg_metadata);
void deserialize_sedp_metadata(uint8_t reader_guid_prefix[12], int64_t *seqnum,
                               uint8_t                   usertraffic_port[2],
                               uint8_t                   app_entity_id[4],
                               const message_metadata_t *msg_metadata);

void serialize_sedp_heartbeat_metadata(const uint8_t reader_guid_prefix[12],
                                       int64_t       first_seqnum,
                                       int64_t last_seqnum, uint32_t cnt,
                                       message_metadata_t *msg_metadata);
void deserialize_sedp_heartbeat_metadata(
    uint8_t reader_guid_prefix[12], int64_t *first_seqnum, int64_t *last_seqnum,
    uint32_t *cnt, const message_metadata_t *msg_metadata);

void serialize_sedp_acknack_metadata(const uint8_t reader_guid_prefix[12],
                                     uint8_t snstate_base, bool snstate_empty,
                                     uint32_t            cnt,
                                     message_metadata_t *msg_metadata);
void deserialize_sedp_acknack_metadata(uint8_t  reader_guid_prefix[12],
                                       uint8_t *snstate_base,
                                       bool *snstate_empty, uint32_t *cnt,
                                       const message_metadata_t *msg_metadata);

void serialize_app_metadata(const uint8_t reader_guid_prefix[12],
                            const uint8_t reader_entity_id[4],
                            const uint8_t writer_entity_id[4], int64_t seqnum,
                            message_metadata_t *msg_metadata);
void deserialize_app_metadata(uint8_t reader_guid_prefix[12],
                              uint8_t reader_entity_id[4],
                              uint8_t writer_entity_id[4], int64_t *seqnum,
                              const message_metadata_t *msg_metadata);

#endif // !MESSAGE_METADATA_HPP
