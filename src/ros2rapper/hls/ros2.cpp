// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "endpoint.hpp"
#include "hls.hpp"
#include "message_metadata.hpp"
#include "remove_endpoints.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"

/* Cyber func=inline */
static void find_unused_and_matched_sedp_endpoint(
    const uint8_t       guid_prefix[GUID_PREFIX_SIZE],
    const sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX], bool *is_full_out,
    bool *is_matched_out, sedp_reader_id_t *unused_idx_out,
    sedp_reader_id_t *matched_idx_out) {
#pragma HLS inline
    bool             is_full = true;
    bool             is_matched = false;
    sedp_reader_id_t unused_idx = 0;
    sedp_reader_id_t matched_idx = 0;

    /* Cyber unroll_times=all */
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
#pragma HLS unroll
        sedp_endpoint participant = sedp_reader_tbl[j];
#pragma HLS array_partition variable = participant.guid_prefix complete dim = 1
        bool j_matched = participant.alive;
        /* Cyber unroll_times=all */
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
#pragma HLS unroll
            if (participant.guid_prefix[k] != guid_prefix[k]) {
                j_matched = false;
            }
        }

        if (is_full && !participant.alive) {
            is_full = false;
            unused_idx = j;
        } else if (!is_matched && j_matched) {
            is_matched = true;
            matched_idx = j;
        }
    }

    *is_full_out = is_full;
    *is_matched_out = is_matched;
    *unused_idx_out = unused_idx;
    *matched_idx_out = matched_idx;
}

/* Cyber func=inline */
static bool is_app_endpoint_matched(sedp_endpoint participant,
                                    app_endpoint app_reader_tbl[APP_READER_MAX],
                                    const uint8_t entity_id[4]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
        if (participant.children[j]) {
            bool matched = true;
            /* Cyber unroll_times=all */
            for (auto k = 0; k < 4; k++) {
#pragma HLS unroll
                if (app_reader_tbl[j].entity_id[k] != entity_id[k]) {
                    matched = false;
                }
            }
            if (matched) {
                return true;
            }
        }
    }
    return false;
}

/* Cyber func=inline */
void ros2_in(hls_stream<rtps_data_t> &in,
             sedp_endpoint            sedp_reader_tbl[SEDP_READER_MAX],
             app_endpoint             app_reader_tbl[APP_READER_MAX],
             hls_uint<PUB_TOPICS_MAX> pub_enable,
             hls_uint<SUB_TOPICS_MAX> sub_enable, int64_t timestamp_i64) {
#pragma HLS inline

    hls_uint<1> enable = (pub_enable != 0) || (sub_enable != 0);

    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
    if (!in.read_nb(rtps_data)) {
        return;
    }
    if (!enable) {
        return;
    }

    bool             is_sedp_reader_tbl_full;
    bool             is_participant_matched;
    sedp_reader_id_t sedp_unused_idx;
    sedp_reader_id_t sedp_matched_idx;
    find_unused_and_matched_sedp_endpoint(
        rtps_data.guid_prefix, sedp_reader_tbl, &is_sedp_reader_tbl_full,
        &is_participant_matched, &sedp_unused_idx, &sedp_matched_idx);

    bool            is_app_reader_tbl_full = true;
    app_reader_id_t app_unused_idx;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
        app_endpoint reader = app_reader_tbl[j];
        if (is_app_reader_tbl_full && !reader.alive) {
            is_app_reader_tbl_full = false;
            app_unused_idx = j;
        }
    }

    sedp_endpoint participant;
#pragma HLS array_partition variable = participant.guid_prefix complete dim = 1
#pragma HLS array_partition variable = participant.ip_addr complete dim = 1
#pragma HLS array_partition variable = participant.udp_port complete dim = 1
#pragma HLS array_partition variable = participant.children complete dim = 1

    app_endpoint reader;
#pragma HLS array_partition variable = reader.guid_prefix complete dim = 1
#pragma HLS array_partition variable = reader.ip_addr complete dim = 1
#pragma HLS array_partition variable = reader.udp_port complete dim = 1
#pragma HLS array_partition variable = reader.entity_id complete dim = 1
    // Initialize reader in case rtps_data.type is RTPS_TYPE_SEDP_PUB or
    // RTPS_TYPE_SEDP_SUB
    /* Cyber unroll_times=all */
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        reader.guid_prefix[j] = rtps_data.guid_prefix[j];
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        reader.ip_addr[j] = rtps_data.data[j];
    }
    reader.udp_port[0] = rtps_data.data[4];
    reader.udp_port[1] = rtps_data.data[5];
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        reader.entity_id[j] = rtps_data.data[j + 6];
    }
    reader.topic_id = rtps_data.data[10];
    reader.alive = true;

    switch (rtps_data.type) {
    case RTPS_TYPE_SPDP:
        if (is_participant_matched) {
            participant = sedp_reader_tbl[sedp_matched_idx];
        } else {
            // Initialize sedp_endpoint
            participant.builtin_pubrd_rd_seqnum = 1;
            participant.builtin_subrd_rd_seqnum = 1;
            participant.builtin_pubrd_wr_seqnum = 0;
            participant.builtin_subrd_wr_seqnum = 0;
            participant.builtin_pubrd_acknack_req = false;
            participant.builtin_subrd_acknack_req = false;
            participant.builtin_pubwr_lastsn = 0;
            participant.builtin_subwr_lastsn = 0;
            participant.initial_send_counter = 0;
            participant.pub_heartbeat_cnt = 0;
            participant.sub_heartbeat_cnt = 0;
            participant.pub_acknack_cnt = 0;
            participant.sub_acknack_cnt = 0;
            reset_sedp_endpoint_children(participant.children);
        }
        /* Cyber unroll_times=all */
        for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
            participant.guid_prefix[j] = rtps_data.guid_prefix[j];
        }
        /* Cyber unroll_times=all */
        for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
            participant.ip_addr[j] = rtps_data.data[j];
        }
        participant.udp_port[0] = rtps_data.data[4];
        participant.udp_port[1] = rtps_data.data[5];
        participant.lease_duration = 0;
        /* Cyber unroll_times=all */
        for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
            participant.lease_duration
                |= static_cast<int64_t>(rtps_data.data[j + 6]) << (8 * j);
        }
        participant.timestamp = timestamp_i64;
        participant.alive = true;
        if (is_participant_matched) {
            sedp_reader_tbl[sedp_matched_idx] = participant;
        } else if (!is_sedp_reader_tbl_full) {
            sedp_reader_tbl[sedp_unused_idx] = participant;
        }
        break;
    case RTPS_TYPE_SEDP_HEARTBEAT_PUB:
        if (is_participant_matched) {
            uint8_t first_sn = rtps_data.data[0];
            uint8_t last_sn = rtps_data.data[1];
            participant = sedp_reader_tbl[sedp_matched_idx];
            if (participant.builtin_pubrd_rd_seqnum < first_sn
                || participant.builtin_pubrd_wr_seqnum < last_sn)
                participant.builtin_pubrd_acknack_req = true;
            if (participant.builtin_pubrd_rd_seqnum < first_sn)
                participant.builtin_pubrd_rd_seqnum = first_sn;
            if (participant.builtin_pubrd_wr_seqnum < last_sn)
                participant.builtin_pubrd_wr_seqnum = last_sn;
            sedp_reader_tbl[sedp_matched_idx] = participant;
        }
        break;
    case RTPS_TYPE_SEDP_HEARTBEAT_SUB:
        if (is_participant_matched) {
            uint8_t first_sn = rtps_data.data[0];
            uint8_t last_sn = rtps_data.data[1];
            participant = sedp_reader_tbl[sedp_matched_idx];
            if (participant.builtin_subrd_rd_seqnum < first_sn
                || participant.builtin_subrd_wr_seqnum < last_sn)
                participant.builtin_subrd_acknack_req = true;
            if (participant.builtin_subrd_rd_seqnum < first_sn)
                participant.builtin_subrd_rd_seqnum = first_sn;
            if (participant.builtin_subrd_wr_seqnum < last_sn)
                participant.builtin_subrd_wr_seqnum = last_sn;
            sedp_reader_tbl[sedp_matched_idx] = participant;
        }
        break;
    case RTPS_TYPE_SEDP_PUB_SN_ONLY:
        if (is_participant_matched) {
            uint8_t sn = rtps_data.data[0];
            participant = sedp_reader_tbl[sedp_matched_idx];
            if (participant.builtin_pubrd_rd_seqnum == sn) {
                participant.builtin_pubrd_rd_seqnum++;
                sedp_reader_tbl[sedp_matched_idx] = participant;
            }
        }
        break;
    case RTPS_TYPE_SEDP_SUB_SN_ONLY:
        if (is_participant_matched) {
            uint8_t sn = rtps_data.data[0];
            participant = sedp_reader_tbl[sedp_matched_idx];
            if (participant.builtin_subrd_rd_seqnum == sn) {
                participant.builtin_subrd_rd_seqnum++;
                sedp_reader_tbl[sedp_matched_idx] = participant;
            }
        }
        break;
    case RTPS_TYPE_SEDP_PUB:
        if (is_participant_matched) {
            uint8_t sn = rtps_data.data[11];
            participant = sedp_reader_tbl[sedp_matched_idx];
            if (participant.builtin_pubrd_rd_seqnum == sn) {
                participant.builtin_pubrd_rd_seqnum++;
                reader.app_ep_type = APP_EP_SUB;
                if (!is_app_reader_tbl_full
                    && !is_app_endpoint_matched(participant, app_reader_tbl,
                                                reader.entity_id)) {
                    app_reader_tbl[app_unused_idx] = reader;
                    participant.children[app_unused_idx] = true;
                }
                sedp_reader_tbl[sedp_matched_idx] = participant;
            }
        }
        break;
    case RTPS_TYPE_SEDP_SUB:
        if (is_participant_matched) {
            uint8_t sn = rtps_data.data[11];
            participant = sedp_reader_tbl[sedp_matched_idx];
            if (participant.builtin_subrd_rd_seqnum == sn) {
                participant.builtin_subrd_rd_seqnum++;
                reader.app_ep_type = APP_EP_PUB;
                if (!is_app_reader_tbl_full
                    && !is_app_endpoint_matched(participant, app_reader_tbl,
                                                reader.entity_id)) {
                    app_reader_tbl[app_unused_idx] = reader;
                    participant.children[app_unused_idx] = true;
                }
                sedp_reader_tbl[sedp_matched_idx] = participant;
            }
        }
        break;
    case RTPS_TYPE_RM_ENDPOINT:
        if (is_participant_matched) {
            participant = sedp_reader_tbl[sedp_matched_idx];
            participant.alive = false;
            /* Cyber unroll_times=all */
            for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
                if (participant.children[j]) {
                    app_reader_tbl[j].alive = false;
                }
            }
            sedp_reader_tbl[sedp_matched_idx] = participant;
        }
        break;
    }
}

/* Cyber func=inline */
static void spdp_writer_out(const uint8_t metatraffic_port[2],
                            const uint8_t default_port[2], const config_t *conf,
                            message_metadata_t *msg_metadata) {
#pragma HLS inline
    static const uint8_t dst_addr[4] /* Cyber array=EXPAND */
        = IP_MULTICAST_ADDR;
    uint8_t dst_port[2] /* Cyber array=EXPAND */;
    dst_port[0] = DISCOVERY_TRAFFIC_MULTICAST_PORT_0(conf->port_num_seed);
    dst_port[1] = DISCOVERY_TRAFFIC_MULTICAST_PORT_1(conf->port_num_seed);
#pragma HLS array_partition variable = dst_addr complete dim = 0
#pragma HLS array_partition variable = dst_port complete dim = 0

    set_common_message_metadata(MSG_TYPE_SPDP, 0, dst_addr, dst_port,
                                msg_metadata);
    serialize_spdp_metadata(metatraffic_port, default_port,
                            conf->participant_lease_duration, msg_metadata);
}

/* Cyber func=inline */
static void sedp_writer_out(message_type_t msg_type, topic_id_t topic_id,
                            const uint8_t dst_addr[4],
                            const uint8_t dst_port[2],
                            const uint8_t reader_guid_prefix[12],
                            int64_t seqnum, const uint8_t usertraffic_port[2],
                            const uint8_t       app_entity_id[4],
                            message_metadata_t *msg_metadata) {
#pragma HLS inline
    set_common_message_metadata(msg_type, topic_id, dst_addr, dst_port,
                                msg_metadata);
    serialize_sedp_metadata(reader_guid_prefix, seqnum, usertraffic_port,
                            app_entity_id, msg_metadata);
}

/* Cyber func=inline */
static void sedp_heartbeat_out(message_type_t msg_type,
                               const uint8_t  dst_addr[4],
                               const uint8_t  dst_port[2],
                               const uint8_t  reader_guid_prefix[12],
                               const int64_t  first_seqnum,
                               const int64_t last_seqnum, uint32_t &cnt,
                               message_metadata_t *msg_metadata) {
#pragma HLS inline
    cnt++;
    set_common_message_metadata(msg_type, 0, dst_addr, dst_port, msg_metadata);
    serialize_sedp_heartbeat_metadata(reader_guid_prefix, first_seqnum,
                                      last_seqnum, cnt, msg_metadata);
}

/* Cyber func=inline */
static void sedp_acknack_out(message_type_t msg_type, const uint8_t dst_addr[4],
                             const uint8_t dst_port[2],
                             const uint8_t reader_guid_prefix[12],
                             uint8_t snstate_base, bool snstate_is_empty,
                             uint32_t &cnt, message_metadata_t *msg_metadata) {
#pragma HLS inline
    cnt++;
    set_common_message_metadata(msg_type, 0, dst_addr, dst_port, msg_metadata);
    serialize_sedp_acknack_metadata(reader_guid_prefix, snstate_base,
                                    snstate_is_empty, cnt, msg_metadata);
}

/* Cyber func=inline */
static void app_writer_out(topic_id_t    topic_id,
                           const uint8_t writer_entity_id[4],
                           const uint8_t dst_addr[4], const uint8_t dst_port[2],
                           const uint8_t reader_guid_prefix[12],
                           const uint8_t reader_entity_id[4], int64_t &seqnum,
                           message_metadata_t *msg_metadata) {
#pragma HLS inline
    seqnum++;
    set_common_message_metadata(MSG_TYPE_APP, topic_id, dst_addr, dst_port,
                                msg_metadata);
    serialize_app_metadata(reader_guid_prefix, reader_entity_id,
                           writer_entity_id, seqnum, msg_metadata);
}

#define SPDP_WRITER_OUT()                                                      \
    do {                                                                       \
        spdp_writer_out(metatraffic_port, default_port, conf, &msg_metadata);  \
    } while (0)

#define SEDP_PUB_WRITER_OUT(topic_id, sedp_reader_id)                          \
    do {                                                                       \
        if (sedp_reader_tbl[(sedp_reader_id)].alive) {                         \
            sedp_reader_tbl[(sedp_reader_id)].builtin_pubwr_lastsn++;          \
            sedp_writer_out(                                                   \
                MSG_TYPE_SEDP_PUB, topic_id,                                   \
                sedp_reader_tbl[(sedp_reader_id)].ip_addr,                     \
                sedp_reader_tbl[(sedp_reader_id)].udp_port,                    \
                sedp_reader_tbl[(sedp_reader_id)].guid_prefix,                 \
                sedp_reader_tbl[(sedp_reader_id)].builtin_pubwr_lastsn,        \
                default_port, app_writer_entity_id_list[(topic_id)],           \
                &msg_metadata);                                                \
        }                                                                      \
    } while (0)

#define SEDP_SUB_WRITER_OUT(topic_id, sedp_reader_id)                          \
    do {                                                                       \
        if (sedp_reader_tbl[(sedp_reader_id)].alive) {                         \
            sedp_reader_tbl[(sedp_reader_id)].builtin_subwr_lastsn++;          \
            sedp_writer_out(                                                   \
                MSG_TYPE_SEDP_SUB, topic_id,                                   \
                sedp_reader_tbl[(sedp_reader_id)].ip_addr,                     \
                sedp_reader_tbl[(sedp_reader_id)].udp_port,                    \
                sedp_reader_tbl[(sedp_reader_id)].guid_prefix,                 \
                sedp_reader_tbl[(sedp_reader_id)].builtin_subwr_lastsn,        \
                default_port, app_reader_entity_id_list[(topic_id)],           \
                &msg_metadata);                                                \
        }                                                                      \
    } while (0)

#define SEDP_PUB_HEARTBEAT_OUT(id)                                             \
    do {                                                                       \
        if (sedp_reader_tbl[(id)].alive) {                                     \
            sedp_heartbeat_out(                                                \
                MSG_TYPE_SEDP_HEARTBEAT_PUB, sedp_reader_tbl[(id)].ip_addr,    \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix,                             \
                sedp_reader_tbl[(id)].builtin_pubwr_lastsn + 1,                \
                sedp_reader_tbl[(id)].builtin_pubwr_lastsn,                    \
                sedp_reader_tbl[(id)].pub_heartbeat_cnt, &msg_metadata);       \
        }                                                                      \
    } while (0)

#define SEDP_SUB_HEARTBEAT_OUT(id)                                             \
    do {                                                                       \
        if (sedp_reader_tbl[(id)].alive) {                                     \
            sedp_heartbeat_out(                                                \
                MSG_TYPE_SEDP_HEARTBEAT_SUB, sedp_reader_tbl[(id)].ip_addr,    \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix,                             \
                sedp_reader_tbl[(id)].builtin_subwr_lastsn + 1,                \
                sedp_reader_tbl[(id)].builtin_subwr_lastsn,                    \
                sedp_reader_tbl[(id)].sub_heartbeat_cnt, &msg_metadata);       \
            if (sedp_reader_tbl[(id)].initial_send_counter != 3)               \
                sedp_reader_tbl[(id)].initial_send_counter++;                  \
        }                                                                      \
    } while (0)

#define SEDP_PUB_ACKNACK_OUT(id)                                               \
    do {                                                                       \
        if (sedp_reader_tbl[(id)].alive) {                                     \
            sedp_acknack_out(                                                  \
                MSG_TYPE_SEDP_ACKNACK_PUB, sedp_reader_tbl[(id)].ip_addr,      \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, snstate_base,               \
                snstate_is_empty, sedp_reader_tbl[(id)].pub_acknack_cnt,       \
                &msg_metadata);                                                \
            sedp_reader_tbl[(id)].builtin_pubrd_acknack_req = false;           \
        }                                                                      \
    } while (0)

#define SEDP_SUB_ACKNACK_OUT(id)                                               \
    do {                                                                       \
        if (sedp_reader_tbl[(id)].alive) {                                     \
            sedp_acknack_out(                                                  \
                MSG_TYPE_SEDP_ACKNACK_SUB, sedp_reader_tbl[(id)].ip_addr,      \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, snstate_base,               \
                snstate_is_empty, sedp_reader_tbl[(id)].sub_acknack_cnt,       \
                &msg_metadata);                                                \
            sedp_reader_tbl[(id)].builtin_subrd_acknack_req = false;           \
        }                                                                      \
    } while (0)

/* Cyber func=inline */
static void APP_WRITER_OUT(topic_id_t topic_id, app_reader_id_t app_reader_id,
                           app_endpoint        app_reader_tbl[APP_READER_MAX],
                           const uint8_t       app_writer_entity_id[4],
                           int64_t            &app_seqnum,
                           message_metadata_t *msg_metadata) {
#pragma HLS inline
    if ((app_reader_id < APP_READER_MAX) && app_reader_tbl[app_reader_id].alive
        && (app_reader_tbl[app_reader_id].app_ep_type & APP_EP_PUB)
        && (app_reader_tbl[app_reader_id].topic_id == topic_id)) {
        app_writer_out(topic_id, app_writer_entity_id,
                       app_reader_tbl[app_reader_id].ip_addr,
                       app_reader_tbl[app_reader_id].udp_port,
                       app_reader_tbl[app_reader_id].guid_prefix,
                       app_reader_tbl[app_reader_id].entity_id, app_seqnum,
                       msg_metadata);
    }
}

/* Cyber func=inline */
static void ros2_out(
    hls_stream<message_metadata_t> &out,
    sedp_endpoint                   sedp_reader_tbl[SEDP_READER_MAX],
    app_endpoint                    app_reader_tbl[APP_READER_MAX],
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    const config_t *conf, VOLATILE uint8_t *rawudp_txbuf_grant,
    hls_uint<1> cnt_interval_elapsed, VOLATILE uint8_t *cnt_interval_set,
    hls_uint<1> cnt_spdp_wr_elapsed, VOLATILE uint8_t *cnt_spdp_wr_set,
    hls_uint<1> cnt_sedp_pub_wr_elapsed, VOLATILE uint8_t *cnt_sedp_pub_wr_set,
    hls_uint<1> cnt_sedp_sub_wr_elapsed, VOLATILE uint8_t *cnt_sedp_sub_wr_set,
    hls_uint<1> cnt_sedp_pub_hb_elapsed, VOLATILE uint8_t *cnt_sedp_pub_hb_set,
    hls_uint<1> cnt_sedp_sub_hb_elapsed, VOLATILE uint8_t *cnt_sedp_sub_hb_set,
    hls_uint<1> cnt_sedp_pub_an_elapsed, VOLATILE uint8_t *cnt_sedp_pub_an_set,
    hls_uint<1> cnt_sedp_sub_an_elapsed, VOLATILE uint8_t *cnt_sedp_sub_an_set,
    hls_uint<1> cnt_app_wr_elapsed, VOLATILE uint8_t *cnt_app_wr_set,
    int64_t timestamp_i64) {
    static const uint8_t
        app_writer_entity_id_list[PUB_TOPICS_MAX]
                                 [4] /* Cyber array=EXPAND, array_index=const */
        = ENTITYID_APP_WRITER_LIST;
#pragma HLS array_partition variable = app_writer_entity_id_list complete dim  \
    = 0
    static const uint8_t
        app_reader_entity_id_list[SUB_TOPICS_MAX]
                                 [4] /* Cyber array=EXPAND, array_index=const */
        = ENTITYID_APP_READER_LIST;
#pragma HLS array_partition variable = app_reader_entity_id_list complete dim  \
    = 0

    uint8_t metatraffic_port[2] /* Cyber array=EXPAND */;
    metatraffic_port[0] = DISCOVERY_TRAFFIC_UNICAST_PORT_0(
        conf->port_num_seed, TARGET_PARTICIPANT_ID);
    metatraffic_port[1] = DISCOVERY_TRAFFIC_UNICAST_PORT_1(
        conf->port_num_seed, TARGET_PARTICIPANT_ID);

    uint8_t default_port[2] /* Cyber array=EXPAND */;
    default_port[0] = USER_TRAFFIC_UNICAST_PORT_0(conf->port_num_seed,
                                                  TARGET_PARTICIPANT_ID);
    default_port[1] = USER_TRAFFIC_UNICAST_PORT_1(conf->port_num_seed,
                                                  TARGET_PARTICIPANT_ID);

#pragma HLS array_partition variable = metatraffic_port complete dim = 0
#pragma HLS array_partition variable = default_port complete dim = 0

    static int64_t app_seqnum;

    static hls_uint<2> tx_progress;
    static_assert(
        SEDP_READER_MAX <= 4,
        "'tx_progress' must be able to represent SEDP_READER_MAX - 1.");
    static_assert(
        APP_READER_MAX <= 4,
        "'tx_progress' must be able to represent APP_READER_MAX - 1.");

    static hls_uint<3> tx_cnt_elapsed;
    static_assert(
        SEDP_READER_MAX <= 7,
        "'tx_cnt_elapsed' should be able to represent SEDP_READER_MAX.");
    static_assert(
        PUB_TOPICS_MAX <= 7,
        "'tx_cnt_elapsed' should be able to represent PUB_TOPICS_MAX.");
    static_assert(
        SUB_TOPICS_MAX <= 7,
        "'tx_cnt_elapsed' should be able to represent SUB_TOPICS_MAX.");

    static hls_uint<3> tx_topic_progress;
    static_assert(
        PUB_TOPICS_MAX <= 7,
        "'tx_topic_progress' must be able to represent PUB_TOPICS_MAX.");
    static_assert(
        SUB_TOPICS_MAX <= 7,
        "'tx_topic_progress' must be able to represent SUB_TOPICS_MAX.");

    static hls_uint<4> next_packet_type = 0;
#define ROTATE_NEXT_PACKET_TYPE                                                \
    do {                                                                       \
        if (next_packet_type < 8) {                                            \
            next_packet_type++;                                                \
        } else {                                                               \
            next_packet_type = 0;                                              \
        }                                                                      \
    } while (0)

    message_metadata_t msg_metadata = {.message_type = MSG_TYPE_NONE};
#pragma HLS array_partition variable = msg_metadata.dst_addr type              \
    = complete                                               dim = 1
#pragma HLS array_partition variable = msg_metadata.dst_port type              \
    = complete                                               dim = 1
#pragma HLS array_partition variable = msg_metadata.rtps_data type             \
    = complete                                                dim = 1

    msg_metadata.now.seconds = static_cast<int32_t>(timestamp_i64 >> 32);
    msg_metadata.now.fraction
        = static_cast<uint32_t>(timestamp_i64 & 0xffffffff);

    if (!out.full()) {
        if (*rawudp_txbuf_grant == 1) {
            msg_metadata.message_type = MSG_TYPE_RAWUDP;
        } else if (cnt_interval_elapsed) {
            if (((pub_enable != 0) || (sub_enable != 0))
                && next_packet_type == 0) {
                // Send a SPDP message if
                //   1. cnt_spdp_wr_elapsed is asserted.
                //   2. there exists a living sedp_endpoint whose
                //      initial_send_counter is less than three.
                bool send = cnt_spdp_wr_elapsed;
                /* Cyber unroll_times=all */
                for (auto j = 0; j < SEDP_READER_MAX; j++) {
#pragma HLS unroll
                    if (sedp_reader_tbl[j].alive
                        && (sedp_reader_tbl[j].initial_send_counter < 3)) {
                        send = true;
                    }
                }
                if (send) {
                    SPDP_WRITER_OUT();
                    /* Cyber scheduling_block = non-transparent */
                cnt_reset_0: {
#pragma HLS protocol fixed
                    *cnt_spdp_wr_set = 1;
                    CLOCK_BOUNDARY;
                    CLOCK_BOUNDARY;
                }
                }
                ROTATE_NEXT_PACKET_TYPE;
            } else if ((pub_enable != 0) && next_packet_type == 1) {
                if (tx_topic_progress >= PUB_TOPICS_MAX) {
                    // Finish sending published topic data
                    if (tx_cnt_elapsed == PUB_TOPICS_MAX) {
                        // If ROS2rapper sends all topic data to all
                        // participants, reset the counter.
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_1: {
#pragma HLS protocol fixed
                        *cnt_sedp_pub_wr_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                    }
                    tx_cnt_elapsed = 0;
                    tx_topic_progress = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                } else if (!pub_enable[tx_topic_progress]) {
                    // Skip disabled topics
                    if (cnt_sedp_pub_wr_elapsed) {
                        tx_cnt_elapsed++;
                    }
                    tx_topic_progress++;
                } else {
                    // Count the number of topics whose data is sent to all
                    // participants.
                    if (cnt_sedp_pub_wr_elapsed && (tx_progress == 0)) {
                        tx_cnt_elapsed++;
                    }
                    // Send published topic data
                    if ((tx_progress < SEDP_READER_MAX)
                        && ((sedp_reader_tbl[tx_progress].initial_send_counter
                             < 3)
                            || cnt_sedp_pub_wr_elapsed)) {
                        SEDP_PUB_WRITER_OUT(tx_topic_progress, tx_progress);
                    }

                    if (tx_progress < (SEDP_READER_MAX - 1)) {
                        tx_progress++;
                    } else {
                        tx_progress = 0;
                        tx_topic_progress++;
                    }
                }
            } else if ((sub_enable != 0) && next_packet_type == 2) {
                if (tx_topic_progress >= SUB_TOPICS_MAX) {
                    // Finish sending subscribed topic data
                    if (tx_cnt_elapsed == SUB_TOPICS_MAX) {
                        // If ROS2rapper sends all topic data to all
                        // participants, reset the counter.
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_2: {
#pragma HLS protocol fixed
                        *cnt_sedp_sub_wr_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                    }
                    tx_cnt_elapsed = 0;
                    tx_topic_progress = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                } else if (!sub_enable[tx_topic_progress]) {
                    // Skip disabled topics
                    if (cnt_sedp_sub_wr_elapsed) {
                        tx_cnt_elapsed++;
                    }
                    tx_topic_progress++;
                } else {
                    // Count the number of topics whose data is sent to all
                    // participants.
                    if (cnt_sedp_sub_wr_elapsed && (tx_progress == 0)) {
                        tx_cnt_elapsed++;
                    }
                    // Send subscribed topic data
                    if ((tx_progress < SEDP_READER_MAX)
                        && ((sedp_reader_tbl[tx_progress].initial_send_counter
                             < 3)
                            || cnt_sedp_sub_wr_elapsed)) {
                        SEDP_SUB_WRITER_OUT(tx_topic_progress, tx_progress);
                    }

                    if (tx_progress < (SEDP_READER_MAX - 1)) {
                        tx_progress++;
                    } else {
                        tx_progress = 0;
                        tx_topic_progress++;
                    }
                }
            } else if (next_packet_type == 3) {
                if (cnt_sedp_pub_hb_elapsed)
                    tx_cnt_elapsed++;

                if ((tx_progress < SEDP_READER_MAX)
                    && ((sedp_reader_tbl[tx_progress].initial_send_counter < 3)
                        || cnt_sedp_pub_hb_elapsed)) {
                    SEDP_PUB_HEARTBEAT_OUT(tx_progress);
                }

                if (tx_progress < (SEDP_READER_MAX - 1)) {
                    tx_progress++;
                } else {
                    if (tx_cnt_elapsed == SEDP_READER_MAX) {
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_3: {
#pragma HLS protocol fixed
                        *cnt_sedp_pub_hb_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                    }
                    tx_progress = 0;
                    tx_cnt_elapsed = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                }
            } else if (next_packet_type == 4) {
                if (cnt_sedp_sub_hb_elapsed)
                    tx_cnt_elapsed++;

                if ((tx_progress < SEDP_READER_MAX)
                    && ((sedp_reader_tbl[tx_progress].initial_send_counter < 3)
                        || cnt_sedp_sub_hb_elapsed)) {
                    SEDP_SUB_HEARTBEAT_OUT(tx_progress);
                }

                if (tx_progress < (SEDP_READER_MAX - 1)) {
                    tx_progress++;
                } else {
                    if (tx_cnt_elapsed == SEDP_READER_MAX) {
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_4: {
#pragma HLS protocol fixed
                        *cnt_sedp_sub_hb_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                    }
                    tx_progress = 0;
                    tx_cnt_elapsed = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                }
            } else if (next_packet_type == 5) {
                if (cnt_sedp_pub_an_elapsed)
                    tx_cnt_elapsed++;

                if (tx_progress < SEDP_READER_MAX) {
                    const sedp_endpoint &reader = sedp_reader_tbl[tx_progress];
                    uint8_t wr_seqnum = reader.builtin_pubrd_wr_seqnum;
                    uint8_t rd_seqnum = reader.builtin_pubrd_rd_seqnum;
                    bool    acknack_req = reader.builtin_pubrd_acknack_req;
                    uint8_t snstate_base = rd_seqnum;
                    bool    snstate_is_empty = (wr_seqnum < rd_seqnum);

                    if (cnt_sedp_pub_an_elapsed
                        || (acknack_req && !snstate_is_empty)) {
                        SEDP_PUB_ACKNACK_OUT(tx_progress);
                    }
                }

                if (tx_progress < (SEDP_READER_MAX - 1)) {
                    tx_progress++;
                } else {
                    if (tx_cnt_elapsed == SEDP_READER_MAX) {
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_5: {
#pragma HLS protocol fixed
                        *cnt_sedp_pub_an_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                    }
                    tx_progress = 0;
                    tx_cnt_elapsed = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                }
            } else if (next_packet_type == 6) {
                if (cnt_sedp_sub_an_elapsed)
                    tx_cnt_elapsed++;

                if (tx_progress < SEDP_READER_MAX) {
                    const sedp_endpoint &reader = sedp_reader_tbl[tx_progress];
                    uint8_t wr_seqnum = reader.builtin_subrd_wr_seqnum;
                    uint8_t rd_seqnum = reader.builtin_subrd_rd_seqnum;
                    bool    acknack_req = reader.builtin_subrd_acknack_req;
                    uint8_t snstate_base = rd_seqnum;
                    bool    snstate_is_empty = (wr_seqnum < rd_seqnum);

                    if (cnt_sedp_sub_an_elapsed
                        || (acknack_req && !snstate_is_empty)) {
                        SEDP_SUB_ACKNACK_OUT(tx_progress);
                    }
                }

                if (tx_progress < (SEDP_READER_MAX - 1)) {
                    tx_progress++;
                } else {
                    if (tx_cnt_elapsed == SEDP_READER_MAX) {
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_6: {
#pragma HLS protocol fixed
                        *cnt_sedp_sub_an_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                    }
                    tx_progress = 0;
                    tx_cnt_elapsed = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                }
            } else if (pub_enable != 0 && cnt_app_wr_elapsed
                       && next_packet_type == 7) {
                APP_WRITER_OUT(tx_topic_progress, tx_progress, app_reader_tbl,
                               app_writer_entity_id_list[tx_topic_progress],
                               app_seqnum, &msg_metadata);
                if (tx_progress < (APP_READER_MAX - 1)) {
                    tx_progress++;
                } else {
                    tx_progress = 0;
                    if (tx_topic_progress < (PUB_TOPICS_MAX - 1)) {
                        tx_topic_progress++;
                    } else {
                        /* Cyber scheduling_block = non-transparent */
                    cnt_reset_7: {
#pragma HLS protocol fixed
                        *cnt_app_wr_set = 1;
                        CLOCK_BOUNDARY;
                        CLOCK_BOUNDARY;
                    }
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_topic_progress = 0;
                    }
                }
            } else if (next_packet_type == 8) {
                // Remove dead endpoints.
                if (tx_progress < SEDP_READER_MAX) {
                    remove_dead_endpoints(tx_progress, sedp_reader_tbl,
                                          app_reader_tbl, timestamp_i64);
                }
                if (tx_progress < (SEDP_READER_MAX - 1)) {
                    tx_progress++;
                } else {
                    tx_progress = 0;
                    ROTATE_NEXT_PACKET_TYPE;
                }
            } else {
                ROTATE_NEXT_PACKET_TYPE;
            }
        }

        if (msg_metadata.message_type != MSG_TYPE_NONE) {
            out.write(msg_metadata);
            /* Cyber scheduling_block = non-transparent */
        cnt_reset_interval: {
#pragma HLS protocol fixed
            *cnt_interval_set = 1;
            CLOCK_BOUNDARY;
            CLOCK_BOUNDARY;
        }
        }
    }
}

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2_main(
    hls_stream<rtps_data_t>        &in /* Cyber port_mode=axi_stream */,
    hls_stream<message_metadata_t> &out /* Cyber port_mode=axi_stream */,
    hls_uint<PUB_TOPICS_MAX>        pub_enable /* Cyber port_mode=in */,
    hls_uint<SUB_TOPICS_MAX>        sub_enable /* Cyber port_mode=in */,
    const config_t                 *conf /* Cyber port_mode=in, stable_input */,
    VOLATILE uint8_t
        *udp_txbuf_grant /* Cyber port_mode=shared, volatile=YES */,

    hls_uint<1> cnt_interval_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_spdp_wr_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_pub_wr_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_sub_wr_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_pub_hb_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_sub_hb_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_pub_an_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_sub_an_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_app_wr_elapsed /* Cyber port_mode=in */,

    VOLATILE uint8_t
        *cnt_interval_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_spdp_wr_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_sedp_pub_wr_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_sedp_sub_wr_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_sedp_pub_hb_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_sedp_sub_hb_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_sedp_pub_an_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *cnt_sedp_sub_an_set /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t *cnt_app_wr_set /* Cyber port_mode=shared, volatile=YES */,

    int64_t timestamp_i64 /* Cyber port_mode=in */) {

#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = ap_none port = pub_enable
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS disaggregate             variable = conf
#pragma HLS disaggregate             variable = conf->participant_lease_duration
#pragma HLS interface mode = ap_none port                                      \
    = conf->participant_lease_duration.seconds
#pragma HLS interface mode = ap_none port                                      \
    = conf->participant_lease_duration.fraction
#pragma HLS interface mode = ap_ack port = udp_txbuf_grant
#pragma HLS interface mode = ap_ctrl_none port = return

#pragma HLS interface mode = ap_ack port = cnt_interval_elapsed
#pragma HLS interface mode = ap_ack port = cnt_spdp_wr_elapsed
#pragma HLS interface mode = ap_ack port = cnt_sedp_pub_wr_elapsed
#pragma HLS interface mode = ap_ack port = cnt_sedp_sub_wr_elapsed
#pragma HLS interface mode = ap_ack port = cnt_sedp_pub_hb_elapsed
#pragma HLS interface mode = ap_ack port = cnt_sedp_sub_hb_elapsed
#pragma HLS interface mode = ap_ack port = cnt_sedp_pub_an_elapsed
#pragma HLS interface mode = ap_ack port = cnt_sedp_sub_an_elapsed
#pragma HLS interface mode = ap_ack port = cnt_app_wr_elapsed

#pragma HLS interface mode = ap_vld port = cnt_interval_set
#pragma HLS interface mode = ap_vld port = cnt_spdp_wr_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_pub_wr_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_sub_wr_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_pub_hb_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_sub_hb_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_pub_an_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_sub_an_set
#pragma HLS interface mode = ap_vld port = cnt_app_wr_set

#pragma HLS interface mode = ap_none port = timestamp_i64

    static sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
    static app_endpoint  app_reader_tbl[APP_READER_MAX];
#pragma HLS array_partition variable = sedp_reader_tbl complete dim = 0
#pragma HLS array_partition variable = app_reader_tbl complete dim = 0

    ros2_in(in, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
            timestamp_i64);

    ros2_out(out, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable, conf,
             udp_txbuf_grant, cnt_interval_elapsed, cnt_interval_set,
             cnt_spdp_wr_elapsed, cnt_spdp_wr_set, cnt_sedp_pub_wr_elapsed,
             cnt_sedp_pub_wr_set, cnt_sedp_sub_wr_elapsed, cnt_sedp_sub_wr_set,
             cnt_sedp_pub_hb_elapsed, cnt_sedp_pub_hb_set,
             cnt_sedp_sub_hb_elapsed, cnt_sedp_sub_hb_set,
             cnt_sedp_pub_an_elapsed, cnt_sedp_pub_an_set,
             cnt_sedp_sub_an_elapsed, cnt_sedp_sub_an_set, cnt_app_wr_elapsed,
             cnt_app_wr_set, timestamp_i64);
}
