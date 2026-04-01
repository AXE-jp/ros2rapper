// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "message_metadata.hpp"
#include "duration.hpp"
#include <cstdint>

/* Cyber func=inline */
static void copy_bytes(const uint8_t src[], uint8_t dst[], unsigned int len) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto i = 0; i < len; i++) {
#pragma HLS unroll
        dst[i] = src[i];
    }
}

/* Cyber func=inline */
void set_common_message_metadata(message_type_t message_type,
                                 topic_id_t topic_id, const uint8_t dst_addr[4],
                                 const uint8_t       dst_port[2],
                                 message_metadata_t *msg_metadata) {
#pragma HLS inline
    msg_metadata->message_type = message_type;
    msg_metadata->topic_id = topic_id;
    copy_bytes(dst_addr, msg_metadata->dst_addr, 4);
    copy_bytes(dst_port, msg_metadata->dst_port, 2);
}

/* Cyber func=inline */
static void serialize_u32(uint32_t data, uint8_t serialized[4]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto i = 0; i < 4; i++) {
#pragma HLS unroll
        serialized[i] = (data >> (8 * i)) & 0xff;
    }
}

/* Cyber func=inline */
static uint32_t deserialize_u32(const uint8_t serialized[4]) {
#pragma HLS inline
    uint32_t x = 0;
    /* Cyber unroll_times=all */
    for (auto i = 0; i < 4; i++) {
#pragma HLS unroll
        x |= static_cast<uint32_t>(serialized[i]) << (8 * i);
    }
    return x;
}

/* Cyber func=inline */
static void serialize_u64(uint64_t data, uint8_t serialized[8]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto i = 0; i < 8; i++) {
#pragma HLS unroll
        serialized[i] = (data >> (8 * i)) & 0xff;
    }
}

/* Cyber func=inline */
static uint64_t deserialize_u64(const uint8_t serialized[8]) {
#pragma HLS inline
    uint64_t x = 0;
    /* Cyber unroll_times=all */
    for (auto i = 0; i < 8; i++) {
#pragma HLS unroll
        x |= static_cast<uint64_t>(serialized[i]) << (8 * i);
    }
    return x;
}

/* Cyber func=inline */
void serialize_spdp_metadata(const uint8_t       metatraffic_port[2],
                             const uint8_t       default_port[2],
                             duration            lease_duration,
                             message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(metatraffic_port, msg_metadata->rtps_data, 2);
    copy_bytes(default_port, msg_metadata->rtps_data + 2, 2);
    serialize_u32(lease_duration.seconds, msg_metadata->rtps_data + 4);
    serialize_u32(lease_duration.fraction, msg_metadata->rtps_data + 8);
}

/* Cyber func=inline */
void deserialize_spdp_metadata(uint8_t                   metatraffic_port[2],
                               uint8_t                   default_port[2],
                               duration                 *lease_duration,
                               const message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(msg_metadata->rtps_data, metatraffic_port, 2);
    copy_bytes(msg_metadata->rtps_data + 2, default_port, 2);
    lease_duration->seconds = deserialize_u32(msg_metadata->rtps_data + 4);
    lease_duration->fraction = deserialize_u32(msg_metadata->rtps_data + 8);
}

/* Cyber func=inline */
void serialize_sedp_metadata(const uint8_t reader_guid_prefix[12],
                             int64_t seqnum, const uint8_t usertraffic_port[2],
                             const uint8_t       app_entity_id[4],
                             message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(reader_guid_prefix, msg_metadata->rtps_data, 12);
    serialize_u64(seqnum, msg_metadata->rtps_data + 12);
    copy_bytes(usertraffic_port, msg_metadata->rtps_data + 20, 2);
    copy_bytes(app_entity_id, msg_metadata->rtps_data + 22, 4);
}

/* Cyber func=inline */
void deserialize_sedp_metadata(uint8_t reader_guid_prefix[12], int64_t *seqnum,
                               uint8_t                   usertraffic_port[2],
                               uint8_t                   app_entity_id[4],
                               const message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(msg_metadata->rtps_data, reader_guid_prefix, 12);
    *seqnum = deserialize_u64(msg_metadata->rtps_data + 12);
    copy_bytes(msg_metadata->rtps_data + 20, usertraffic_port, 2);
    copy_bytes(msg_metadata->rtps_data + 22, app_entity_id, 4);
}

/* Cyber func=inline */
void serialize_sedp_heartbeat_metadata(const uint8_t reader_guid_prefix[12],
                                       int64_t       first_seqnum,
                                       int64_t last_seqnum, uint32_t cnt,
                                       message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(reader_guid_prefix, msg_metadata->rtps_data, 12);
    serialize_u64(first_seqnum, msg_metadata->rtps_data + 12);
    serialize_u64(last_seqnum, msg_metadata->rtps_data + 20);
    serialize_u32(cnt, msg_metadata->rtps_data + 28);
}

/* Cyber func=inline */
void deserialize_sedp_heartbeat_metadata(
    uint8_t reader_guid_prefix[12], int64_t *first_seqnum, int64_t *last_seqnum,
    uint32_t *cnt, const message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(msg_metadata->rtps_data, reader_guid_prefix, 12);
    *first_seqnum = deserialize_u64(msg_metadata->rtps_data + 12);
    *last_seqnum = deserialize_u64(msg_metadata->rtps_data + 20);
    *cnt = deserialize_u32(msg_metadata->rtps_data + 28);
}

/* Cyber func=inline */
void serialize_sedp_acknack_metadata(const uint8_t reader_guid_prefix[12],
                                     uint8_t snstate_base, bool snstate_empty,
                                     uint32_t            cnt,
                                     message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(reader_guid_prefix, msg_metadata->rtps_data, 12);
    msg_metadata->rtps_data[12] = snstate_base;
    msg_metadata->rtps_data[13] = snstate_empty ? 1 : 0;
    serialize_u32(cnt, msg_metadata->rtps_data + 14);
}

/* Cyber func=inline */
void deserialize_sedp_acknack_metadata(uint8_t  reader_guid_prefix[12],
                                       uint8_t *snstate_base,
                                       bool *snstate_empty, uint32_t *cnt,
                                       const message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(msg_metadata->rtps_data, reader_guid_prefix, 12);
    *snstate_base = msg_metadata->rtps_data[12];
    *snstate_empty = (msg_metadata->rtps_data[13] != 0);
    *cnt = deserialize_u32(msg_metadata->rtps_data + 14);
}

/* Cyber func=inline */
void serialize_app_metadata(const uint8_t reader_guid_prefix[12],
                            const uint8_t reader_entity_id[4],
                            const uint8_t writer_entity_id[4], int64_t seqnum,
                            message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(reader_guid_prefix, msg_metadata->rtps_data, 12);
    copy_bytes(reader_entity_id, msg_metadata->rtps_data + 12, 4);
    copy_bytes(writer_entity_id, msg_metadata->rtps_data + 16, 4);
    serialize_u64(seqnum, msg_metadata->rtps_data + 20);
}

/* Cyber func=inline */
void deserialize_app_metadata(uint8_t reader_guid_prefix[12],
                              uint8_t reader_entity_id[4],
                              uint8_t writer_entity_id[4], int64_t *seqnum,
                              const message_metadata_t *msg_metadata) {
#pragma HLS inline
    copy_bytes(msg_metadata->rtps_data, reader_guid_prefix, 12);
    copy_bytes(msg_metadata->rtps_data + 12, reader_entity_id, 4);
    copy_bytes(msg_metadata->rtps_data + 16, writer_entity_id, 4);
    *seqnum = deserialize_u64(msg_metadata->rtps_data + 20);
}
