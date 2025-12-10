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
    const uint8_t            guid_prefix[GUID_PREFIX_SIZE],
    const sedp_reader_tbl_t *sedp_reader_tbl, bool *is_full_out,
    bool *is_matched_out, sedp_reader_id_t *unused_idx_out,
    sedp_reader_id_t *matched_idx_out) {
#pragma HLS inline
    bool             is_full = true;
    bool             is_matched = false;
    sedp_reader_id_t unused_idx = 0;
    sedp_reader_id_t matched_idx = 0;

#ifdef SEDP_READER_TBL_FF
    /* Cyber unroll_times=all */
#else  // !SEDP_READER_TBL_FF
    /* Cyber folding=2 */
#endif // SEDP_READER_TBL_FF
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
#ifdef SEDP_READER_TBL_FF
#pragma HLS unroll
#else // !SEDP_READER_TBL_FF
#pragma HLS pipeline II = 2
#endif // SEDP_READER_TBL_FF

        uint8_t j_guid_prefix[12];
#pragma HLS array_partition variable = j_guid_prefix complete dim = 1
        bool j_alive = get_sedp_reader_tbl_guid_prefix(j_guid_prefix,
                                                       sedp_reader_tbl, j);

        bool j_matched = j_alive;
        /* Cyber unroll_times=all */
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
#pragma HLS unroll
            if (j_guid_prefix[k] != guid_prefix[k]) {
                j_matched = false;
            }
        }

        if (is_full && !j_alive) {
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
static void
find_unused_app_endpoint(const app_endpoint app_reader_tbl[APP_READER_MAX],
                         bool *is_full_out, app_reader_id_t *unused_idx_out) {
#pragma HLS inline
    bool            is_full = true;
    app_reader_id_t unused_idx = 0;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
        app_endpoint reader = app_reader_tbl[j];
        if (is_full && !reader.alive) {
            is_full = false;
            unused_idx = j;
        }
    }
    *is_full_out = is_full;
    *unused_idx_out = unused_idx;
}

/* Cyber func=inline */
static void copy_sedp_endpoint_params(const rtps_data_t &rtps_data,
                                      uint8_t ip_addr[4], uint8_t udp_port[2],
                                      int64_t *lease_duration_out) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        ip_addr[j] = rtps_data.data[j];
    }

    udp_port[0] = rtps_data.data[4];
    udp_port[1] = rtps_data.data[5];

    int64_t lease_duration = 0;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
        lease_duration |= static_cast<int64_t>(rtps_data.data[j + 6])
                          << (8 * j);
    }
    *lease_duration_out = lease_duration;
}

/* Cyber func=inline */
static void copy_app_endpoint_params(const rtps_data_t &rtps_data,
                                     app_endpoint      *reader) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        reader->guid_prefix[j] = rtps_data.guid_prefix[j];
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        reader->ip_addr[j] = rtps_data.data[j];
    }
    reader->udp_port[0] = rtps_data.data[4];
    reader->udp_port[1] = rtps_data.data[5];
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        reader->entity_id[j] = rtps_data.data[j + 6];
    }
    reader->topic_id = rtps_data.data[10];
}

/* Cyber func=inline */
static void ros2_in_spdp_update(const uint8_t ip_addr[4],
                                const uint8_t udp_port[2],
                                int64_t lease_duration, int64_t timestamp_i64,
                                sedp_reader_tbl_t *tbl, sedp_reader_id_t idx) {
#pragma HLS inline
    // Update the UDP port
    uint64_t rdata_0;
    get_sedp_reader_tbl(&rdata_0, tbl, idx, 0);
    uint64_t wdata_0 = (rdata_0 & 0xffffffff0000ffff)
                       | (static_cast<uint64_t>(udp_port[0]) << 16)
                       | (static_cast<uint64_t>(udp_port[1]) << 24);
    set_sedp_reader_tbl(wdata_0, tbl, idx, 0);

    // Update the IP address
    uint8_t old_ip_addr[4];
    uint8_t sn_0, sn_1, sn_2, sn_3;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(old_ip_addr, &sn_0, &sn_1, &sn_2,
                                               &sn_3, tbl, idx);
    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, sn_0, sn_1, sn_2, sn_3,
                                               tbl, idx);

    set_sedp_reader_tbl_lease_duration(lease_duration, tbl, idx);
    set_sedp_reader_tbl_timestamp(timestamp_i64, tbl, idx);
}

/* Cyber func=inline */
static void ros2_in_spdp_new(const uint8_t ip_addr[4],
                             const uint8_t udp_port[2],
                             const uint8_t guid_prefix[12],
                             int64_t lease_duration, int64_t timestamp_i64,
                             sedp_reader_tbl_t *tbl, sedp_reader_id_t idx) {
#pragma HLS inline
    // Set flags, initial_send_counter, UDP port and GUID prefix
    uint64_t wdata_0 = SEDP_ENDPOINT_ALIVE
                       | (static_cast<uint64_t>(udp_port[0]) << 16)
                       | (static_cast<uint64_t>(udp_port[1]) << 24);
    uint64_t wdata_1 = 0;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        wdata_0 |= static_cast<uint64_t>(guid_prefix[j]) << (8 * (j + 4));
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
        wdata_1 |= static_cast<uint64_t>(guid_prefix[j + 4]) << (8 * j);
    }
    set_sedp_reader_tbl(wdata_0, tbl, idx, 0);
    set_sedp_reader_tbl(wdata_1, tbl, idx, 1);

    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, 0, 1, 0, 1, tbl, idx);
    set_sedp_reader_tbl_pubwr_lastsn(0, tbl, idx);
    set_sedp_reader_tbl_subwr_lastsn(0, tbl, idx);
    set_sedp_reader_tbl_heartbeat_cnt(0, 0, tbl, idx);
    set_sedp_reader_tbl_acknack_cnt(0, 0, tbl, idx);
    set_sedp_reader_tbl_lease_duration(lease_duration, tbl, idx);
    set_sedp_reader_tbl_timestamp(timestamp_i64, tbl, idx);
    clear_sedp_reader_tbl_children(tbl, idx);
}

/* Cyber func=inline */
static void ros2_in_spdp(const rtps_data_t &rtps_data, int64_t timestamp_i64,
                         sedp_reader_tbl_t *tbl, bool is_matched,
                         sedp_reader_id_t matched_idx, bool is_full,
                         sedp_reader_id_t unused_idx) {
#pragma HLS inline
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = udp_port complete dim = 1
    int64_t lease_duration;
    copy_sedp_endpoint_params(rtps_data, ip_addr, udp_port, &lease_duration);

    if (is_matched) {
        ros2_in_spdp_update(ip_addr, udp_port, lease_duration, timestamp_i64,
                            tbl, matched_idx);
    } else if (!is_full) {
        ros2_in_spdp_new(ip_addr, udp_port, rtps_data.guid_prefix,
                         lease_duration, timestamp_i64, tbl, unused_idx);
    }
}

/* Cyber func=inline */
static void ros2_in_sedp_heartbeat_pub(uint8_t first_sn, uint8_t last_sn,
                                       sedp_reader_tbl_t *tbl,
                                       sedp_reader_id_t   idx) {
#pragma HLS inline
    uint8_t ip_addr[4];
#pragma HLS array_partition variable = ip_addr complete dim = 1
    uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum, subrd_rd_seqnum;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum, &subrd_wr_seqnum,
        &subrd_rd_seqnum, tbl, idx);

    if (pubrd_rd_seqnum < first_sn || pubrd_wr_seqnum < last_sn) {
        enable_sedp_reader_tbl_flags(SEDP_ENDPOINT_PUBRD_ACKNACK_REQ, tbl, idx);
    }
    if (pubrd_rd_seqnum < first_sn) {
        pubrd_rd_seqnum = first_sn;
    }
    if (pubrd_wr_seqnum < last_sn) {
        pubrd_wr_seqnum = last_sn;
    }
    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, pubrd_wr_seqnum,
                                               pubrd_rd_seqnum, subrd_wr_seqnum,
                                               subrd_rd_seqnum, tbl, idx);
}

/* Cyber func=inline */
static void ros2_in_sedp_heartbeat_sub(uint8_t first_sn, uint8_t last_sn,
                                       sedp_reader_tbl_t *tbl,
                                       sedp_reader_id_t   idx) {
#pragma HLS inline
    uint8_t ip_addr[4];
#pragma HLS array_partition variable = ip_addr complete dim = 1
    uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum, subrd_rd_seqnum;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum, &subrd_wr_seqnum,
        &subrd_rd_seqnum, tbl, idx);

    if (subrd_rd_seqnum < first_sn || subrd_wr_seqnum < last_sn) {
        enable_sedp_reader_tbl_flags(SEDP_ENDPOINT_SUBRD_ACKNACK_REQ, tbl, idx);
    }
    if (subrd_rd_seqnum < first_sn) {
        subrd_rd_seqnum = first_sn;
    }
    if (subrd_wr_seqnum < last_sn) {
        subrd_wr_seqnum = last_sn;
    }
    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, pubrd_wr_seqnum,
                                               pubrd_rd_seqnum, subrd_wr_seqnum,
                                               subrd_rd_seqnum, tbl, idx);
}

/* Cyber func=inline */
static bool is_same_entity_id(const uint8_t lhs[4], const uint8_t rhs[4]) {
#pragma HLS inline
    bool is_same = true;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        if (lhs[j] != rhs[j]) {
            is_same = false;
        }
    }
    return is_same;
}

/* Cyber func=inline */
static void ros2_in_add_new_app_endpoint(
    const app_endpoint &reader, sedp_reader_tbl_t *sedp_reader_tbl,
    app_endpoint app_reader_tbl[APP_READER_MAX], sedp_reader_id_t sedp_idx,
    app_reader_id_t app_idx) {
#pragma HLS inline
    // Read children of sedp_endpoint.
    uint64_t children_0, children_1;
    get_sedp_reader_tbl(&children_0, sedp_reader_tbl, sedp_idx, 9);
    get_sedp_reader_tbl(&children_1, sedp_reader_tbl, sedp_idx, 10);

    // Check whether the found endpoint is known
    bool is_known_app_endpoint = false;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 64; j++) {
#pragma HLS unroll
        uint64_t flag = static_cast<uint64_t>(1) << j;
        if (((children_0 & flag) != 0)
            && is_same_entity_id(app_reader_tbl[j].entity_id,
                                 reader.entity_id)) {
            is_known_app_endpoint = true;
        }
        if (((children_1 & flag) != 0)
            && is_same_entity_id(app_reader_tbl[j + 64].entity_id,
                                 reader.entity_id)) {
            is_known_app_endpoint = true;
        }
    }

    // Update the tables
    if (!is_known_app_endpoint) {
        app_reader_tbl[app_idx] = reader;
        if (app_idx < 64) {
            uint64_t data = children_0 | (static_cast<uint64_t>(1) << app_idx);
            set_sedp_reader_tbl(data, sedp_reader_tbl, sedp_idx, 9);
        } else {
            uint64_t data
                = children_1 | (static_cast<uint64_t>(1) << (app_idx - 64));
            set_sedp_reader_tbl(data, sedp_reader_tbl, sedp_idx, 10);
        }
    }
}

/* Cyber func=inline */
static void ros2_in_sedp_pub(bool is_valid_topic, uint8_t seqnum,
                             const app_endpoint &reader,
                             sedp_reader_tbl_t  *sedp_reader_tbl,
                             app_endpoint        app_reader_tbl[APP_READER_MAX],
                             sedp_reader_id_t    sedp_idx,
                             app_reader_id_t     app_idx) {
#pragma HLS inline
    uint8_t ip_addr[4];
#pragma HLS array_partition variable = ip_addr complete dim = 1
    uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum, subrd_rd_seqnum;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum, &subrd_wr_seqnum,
        &subrd_rd_seqnum, sedp_reader_tbl, sedp_idx);

    if (pubrd_rd_seqnum != seqnum) {
        return;
    }

    pubrd_rd_seqnum++;
    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum,
        subrd_rd_seqnum, sedp_reader_tbl, sedp_idx);
    enable_sedp_reader_tbl_flags(SEDP_ENDPOINT_PUBRD_ACKNACK_REQ,
                                 sedp_reader_tbl, sedp_idx);

    if (is_valid_topic) {
        ros2_in_add_new_app_endpoint(reader, sedp_reader_tbl, app_reader_tbl,
                                     sedp_idx, app_idx);
    }
}

/* Cyber func=inline */
static void ros2_in_sedp_sub(bool is_valid_topic, uint8_t seqnum,
                             const app_endpoint &reader,
                             sedp_reader_tbl_t  *sedp_reader_tbl,
                             app_endpoint        app_reader_tbl[APP_READER_MAX],
                             sedp_reader_id_t    sedp_idx,
                             app_reader_id_t     app_idx) {
#pragma HLS inline
    uint8_t ip_addr[4];
#pragma HLS array_partition variable = ip_addr complete dim = 1
    uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum, subrd_rd_seqnum;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum, &subrd_wr_seqnum,
        &subrd_rd_seqnum, sedp_reader_tbl, sedp_idx);

    if (subrd_rd_seqnum != seqnum) {
        return;
    }

    subrd_rd_seqnum++;
    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum,
        subrd_rd_seqnum, sedp_reader_tbl, sedp_idx);
    enable_sedp_reader_tbl_flags(SEDP_ENDPOINT_SUBRD_ACKNACK_REQ,
                                 sedp_reader_tbl, sedp_idx);

    if (is_valid_topic) {
        ros2_in_add_new_app_endpoint(reader, sedp_reader_tbl, app_reader_tbl,
                                     sedp_idx, app_idx);
    }
}

/* Cyber func=inline */
void ros2_in(hls_stream<rtps_data_t> &in, sedp_reader_tbl_t *sedp_reader_tbl,
             app_endpoint             app_reader_tbl[APP_READER_MAX],
             hls_uint<PUB_TOPICS_MAX> pub_enable,
             hls_uint<SUB_TOPICS_MAX> sub_enable, int64_t timestamp_i64) {
#pragma HLS inline

    hls_uint<1> enable = (pub_enable != 0) || (sub_enable != 0);

    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
    if (in.empty()) {
        return;
    }
    in.read_nb(rtps_data);
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

    bool            is_app_reader_tbl_full;
    app_reader_id_t app_unused_idx;
    find_unused_app_endpoint(app_reader_tbl, &is_app_reader_tbl_full,
                             &app_unused_idx);

    app_endpoint reader;
#pragma HLS array_partition variable = reader.guid_prefix complete dim = 1
#pragma HLS array_partition variable = reader.ip_addr complete dim = 1
#pragma HLS array_partition variable = reader.udp_port complete dim = 1
#pragma HLS array_partition variable = reader.entity_id complete dim = 1
    // Initialize reader in case rtps_data.type is RTPS_TYPE_SEDP_PUB or
    // RTPS_TYPE_SEDP_SUB
    copy_app_endpoint_params(rtps_data, &reader);
    reader.alive = true;

    switch (rtps_data.type) {
    case RTPS_TYPE_SPDP:
        ros2_in_spdp(rtps_data, timestamp_i64, sedp_reader_tbl,
                     is_participant_matched, sedp_matched_idx,
                     is_sedp_reader_tbl_full, sedp_unused_idx);
        break;
    case RTPS_TYPE_SEDP_HEARTBEAT_PUB:
        if (is_participant_matched) {
            uint8_t first_sn = rtps_data.data[0];
            uint8_t last_sn = rtps_data.data[1];
            ros2_in_sedp_heartbeat_pub(first_sn, last_sn, sedp_reader_tbl,
                                       sedp_matched_idx);
        }
        break;
    case RTPS_TYPE_SEDP_HEARTBEAT_SUB:
        if (is_participant_matched) {
            uint8_t first_sn = rtps_data.data[0];
            uint8_t last_sn = rtps_data.data[1];
            ros2_in_sedp_heartbeat_sub(first_sn, last_sn, sedp_reader_tbl,
                                       sedp_matched_idx);
        }
        break;
    case RTPS_TYPE_SEDP_PUB_SN_ONLY:
    case RTPS_TYPE_SEDP_PUB:
        reader.app_ep_type = APP_EP_SUB;
        if (is_participant_matched && !is_app_reader_tbl_full) {
            ros2_in_sedp_pub(rtps_data.type == RTPS_TYPE_SEDP_PUB,
                             rtps_data.data[11], reader, sedp_reader_tbl,
                             app_reader_tbl, sedp_matched_idx, app_unused_idx);
        }
        break;
    case RTPS_TYPE_SEDP_SUB_SN_ONLY:
    case RTPS_TYPE_SEDP_SUB:
        reader.app_ep_type = APP_EP_PUB;
        if (is_participant_matched && !is_app_reader_tbl_full) {
            ros2_in_sedp_sub(rtps_data.type == RTPS_TYPE_SEDP_SUB,
                             rtps_data.data[11], reader, sedp_reader_tbl,
                             app_reader_tbl, sedp_matched_idx, app_unused_idx);
        }
        break;
    case RTPS_TYPE_RM_ENDPOINT:
        if (is_participant_matched) {
            remove_sedp_endpoint(sedp_matched_idx, sedp_reader_tbl,
                                 app_reader_tbl);
        }
        break;
    }
}

/* Cyber func=inline */
static bool get_ros2_out_spdp_info(const sedp_reader_tbl_t *tbl,
                                   unsigned int             idx) {
#pragma HLS inline
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, idx, 0);
    bool        alive = ((data & SEDP_ENDPOINT_ALIVE) != 0);
    hls_uint<2> initial_send_counter = ((data >> 8) & 3);
    return alive && (initial_send_counter < 3);
}

/* Cyber func=inline */
static bool get_ros2_out_sedp_info(bool    cnt_elapsed,
                                   bool    increment_initial_send_counter,
                                   uint8_t ip_addr[4], uint8_t udp_port[2],
                                   uint8_t            guid_prefix[12],
                                   sedp_reader_tbl_t *tbl, unsigned int idx) {
#pragma HLS inline
    uint64_t data_0, data_1;
    uint8_t  sn_0, sn_1, sn_2, sn_3;

    get_sedp_reader_tbl(&data_0, tbl, idx, 0);
    bool        alive = ((data_0 & SEDP_ENDPOINT_ALIVE) != 0);
    hls_uint<2> initial_send_counter = ((data_0 >> 8) & 3);
    if (!alive || ((initial_send_counter == 3) && !cnt_elapsed)) {
        return false;
    }

    udp_port[0] = (data_0 >> 16) & 0xff;
    udp_port[1] = (data_0 >> 24) & 0xff;

    get_sedp_reader_tbl(&data_1, tbl, idx, 1);
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        guid_prefix[j] = (data_0 >> (8 * (j + 4))) & 0xff;
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
        guid_prefix[j + 4] = (data_1 >> (8 * j)) & 0xff;
    }

    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(ip_addr, &sn_0, &sn_1, &sn_2,
                                               &sn_3, tbl, idx);

    if (increment_initial_send_counter && (initial_send_counter < 3)) {
        initial_send_counter++;
        uint64_t wdata = (data_0 & 0xffffffffffff00ff)
                         | (static_cast<uint64_t>(initial_send_counter) << 8);
        set_sedp_reader_tbl(wdata, tbl, idx, 0);
    }

    return true;
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
                               const int64_t last_seqnum, uint32_t cnt,
                               message_metadata_t *msg_metadata) {
#pragma HLS inline
    set_common_message_metadata(msg_type, 0, dst_addr, dst_port, msg_metadata);
    serialize_sedp_heartbeat_metadata(reader_guid_prefix, first_seqnum,
                                      last_seqnum, cnt, msg_metadata);
}

/* Cyber func=inline */
static void sedp_acknack_out(message_type_t msg_type, const uint8_t dst_addr[4],
                             const uint8_t dst_port[2],
                             const uint8_t reader_guid_prefix[12],
                             uint8_t snstate_base, bool snstate_is_empty,
                             uint32_t cnt, message_metadata_t *msg_metadata) {
#pragma HLS inline
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

/* Cyber func=inline */
static void SEDP_PUB_WRITER_OUT(hls_uint<1> cnt_elapsed, topic_id_t topic_id,
                                const uint8_t      default_port[2],
                                const uint8_t      entity_id[4],
                                sedp_reader_tbl_t *tbl, sedp_reader_id_t idx,
                                message_metadata_t *msg_metadata) {
#pragma HLS inline
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
#pragma HLS array_partition variable = udp_port complete dim = 1
#pragma HLS array_partition variable = guid_prefix complete dim = 1
    if (!get_ros2_out_sedp_info(cnt_elapsed, false, ip_addr, udp_port,
                                guid_prefix, tbl, idx)) {
        return;
    }

    int64_t pubwr_lastsn;
    get_sedp_reader_tbl_pubwr_lastsn(&pubwr_lastsn, tbl, idx);
    pubwr_lastsn++;
    set_sedp_reader_tbl_pubwr_lastsn(pubwr_lastsn, tbl, idx);

    sedp_writer_out(MSG_TYPE_SEDP_PUB, topic_id, ip_addr, udp_port, guid_prefix,
                    pubwr_lastsn, default_port, entity_id, msg_metadata);
}

/* Cyber func=inline */
static void SEDP_SUB_WRITER_OUT(hls_uint<1> cnt_elapsed, topic_id_t topic_id,
                                const uint8_t      default_port[2],
                                const uint8_t      entity_id[4],
                                sedp_reader_tbl_t *tbl, sedp_reader_id_t idx,
                                message_metadata_t *msg_metadata) {
#pragma HLS inline
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
#pragma HLS array_partition variable = udp_port complete dim = 1
#pragma HLS array_partition variable = guid_prefix complete dim = 1
    if (!get_ros2_out_sedp_info(cnt_elapsed, false, ip_addr, udp_port,
                                guid_prefix, tbl, idx)) {
        return;
    }

    int64_t subwr_lastsn;
    get_sedp_reader_tbl_subwr_lastsn(&subwr_lastsn, tbl, idx);
    subwr_lastsn++;
    set_sedp_reader_tbl_subwr_lastsn(subwr_lastsn, tbl, idx);

    sedp_writer_out(MSG_TYPE_SEDP_SUB, topic_id, ip_addr, udp_port, guid_prefix,
                    subwr_lastsn, default_port, entity_id, msg_metadata);
}

/* Cyber func=inline */
static void SEDP_PUB_HEARTBEAT_OUT(hls_uint<1>        cnt_elapsed,
                                   sedp_reader_tbl_t *tbl, sedp_reader_id_t idx,
                                   message_metadata_t *msg_metadata) {
#pragma HLS inline
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
#pragma HLS array_partition variable = udp_port complete dim = 1
#pragma HLS array_partition variable = guid_prefix complete dim = 1
    if (!get_ros2_out_sedp_info(cnt_elapsed, false, ip_addr, udp_port,
                                guid_prefix, tbl, idx)) {
        return;
    }

    int64_t pubwr_lastsn;
    get_sedp_reader_tbl_pubwr_lastsn(&pubwr_lastsn, tbl, idx);

    uint32_t pub_heartbeat_cnt, sub_heartbeat_cnt;
    get_sedp_reader_tbl_heartbeat_cnt(&pub_heartbeat_cnt, &sub_heartbeat_cnt,
                                      tbl, idx);
    pub_heartbeat_cnt++;
    set_sedp_reader_tbl_heartbeat_cnt(pub_heartbeat_cnt, sub_heartbeat_cnt, tbl,
                                      idx);

    sedp_heartbeat_out(MSG_TYPE_SEDP_HEARTBEAT_PUB, ip_addr, udp_port,
                       guid_prefix, pubwr_lastsn + 1, pubwr_lastsn,
                       pub_heartbeat_cnt, msg_metadata);
}

/* Cyber func=inline */
static void SEDP_SUB_HEARTBEAT_OUT(hls_uint<1>        cnt_elapsed,
                                   sedp_reader_tbl_t *tbl, sedp_reader_id_t idx,
                                   message_metadata_t *msg_metadata) {
#pragma HLS inline
    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
#pragma HLS array_partition variable = udp_port complete dim = 1
#pragma HLS array_partition variable = guid_prefix complete dim = 1
    if (!get_ros2_out_sedp_info(cnt_elapsed, true, ip_addr, udp_port,
                                guid_prefix, tbl, idx)) {
        return;
    }

    int64_t subwr_lastsn;
    get_sedp_reader_tbl_subwr_lastsn(&subwr_lastsn, tbl, idx);

    uint32_t pub_heartbeat_cnt, sub_heartbeat_cnt;
    get_sedp_reader_tbl_heartbeat_cnt(&pub_heartbeat_cnt, &sub_heartbeat_cnt,
                                      tbl, idx);
    sub_heartbeat_cnt++;
    set_sedp_reader_tbl_heartbeat_cnt(pub_heartbeat_cnt, sub_heartbeat_cnt, tbl,
                                      idx);

    sedp_heartbeat_out(MSG_TYPE_SEDP_HEARTBEAT_SUB, ip_addr, udp_port,
                       guid_prefix, subwr_lastsn + 1, subwr_lastsn,
                       sub_heartbeat_cnt, msg_metadata);
}

/* Cyber func=inline */
static void SEDP_PUB_ACKNACK_OUT(hls_uint<1>        cnt_elapsed,
                                 sedp_reader_tbl_t *tbl, sedp_reader_id_t idx,
                                 message_metadata_t *msg_metadata) {
#pragma HLS inline
    static const uint64_t req_flag_mask = SEDP_ENDPOINT_PUBRD_ACKNACK_REQ;

    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
#pragma HLS array_partition variable = udp_port complete dim = 1
#pragma HLS array_partition variable = guid_prefix complete dim = 1

    uint64_t data_0, data_1;
    get_sedp_reader_tbl(&data_0, tbl, idx, 0);
    bool alive = ((data_0 & SEDP_ENDPOINT_ALIVE) != 0);
    bool acknack_req = ((data_0 & req_flag_mask) != 0);
    if (!alive || (!cnt_elapsed && !acknack_req)) {
        return;
    }

    uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum, subrd_rd_seqnum;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum, &subrd_wr_seqnum,
        &subrd_rd_seqnum, tbl, idx);
    uint8_t snstate_base = pubrd_rd_seqnum;
    bool    snstate_is_empty = (pubrd_wr_seqnum < pubrd_rd_seqnum);
    if (!cnt_elapsed && snstate_is_empty) {
        return;
    }

    udp_port[0] = (data_0 >> 16) & 0xff;
    udp_port[1] = (data_0 >> 24) & 0xff;

    get_sedp_reader_tbl(&data_1, tbl, idx, 1);
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        guid_prefix[j] = (data_0 >> (8 * (j + 4))) & 0xff;
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
        guid_prefix[j + 4] = (data_1 >> (8 * j)) & 0xff;
    }

    uint32_t pub_acknack_cnt, sub_acknack_cnt;
    get_sedp_reader_tbl_acknack_cnt(&pub_acknack_cnt, &sub_acknack_cnt, tbl,
                                    idx);
    pub_acknack_cnt++;
    set_sedp_reader_tbl_acknack_cnt(pub_acknack_cnt, sub_acknack_cnt, tbl, idx);

    sedp_acknack_out(MSG_TYPE_SEDP_ACKNACK_PUB, ip_addr, udp_port, guid_prefix,
                     snstate_base, snstate_is_empty, pub_acknack_cnt,
                     msg_metadata);

    if (acknack_req) {
        set_sedp_reader_tbl(data_0 & ~req_flag_mask, tbl, idx, 0);
    }
}

/* Cyber func=inline */
static void SEDP_SUB_ACKNACK_OUT(hls_uint<1>        cnt_elapsed,
                                 sedp_reader_tbl_t *tbl, sedp_reader_id_t idx,
                                 message_metadata_t *msg_metadata) {
#pragma HLS inline
    static const uint64_t req_flag_mask = SEDP_ENDPOINT_SUBRD_ACKNACK_REQ;

    uint8_t ip_addr[4] /* Cyber array=EXPAND */;
    uint8_t udp_port[2] /* Cyber array=EXPAND */;
    uint8_t guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
#pragma HLS array_partition variable = udp_port complete dim = 1
#pragma HLS array_partition variable = guid_prefix complete dim = 1

    uint64_t data_0, data_1;
    get_sedp_reader_tbl(&data_0, tbl, idx, 0);
    bool alive = ((data_0 & SEDP_ENDPOINT_ALIVE) != 0);
    bool acknack_req = ((data_0 & req_flag_mask) != 0);
    if (!alive || (!cnt_elapsed && !acknack_req)) {
        return;
    }

    uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum, subrd_rd_seqnum;
    get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
        ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum, &subrd_wr_seqnum,
        &subrd_rd_seqnum, tbl, idx);
    uint8_t snstate_base = subrd_rd_seqnum;
    bool    snstate_is_empty = (subrd_wr_seqnum < subrd_rd_seqnum);
    if (!cnt_elapsed && snstate_is_empty) {
        return;
    }

    udp_port[0] = (data_0 >> 16) & 0xff;
    udp_port[1] = (data_0 >> 24) & 0xff;

    get_sedp_reader_tbl(&data_1, tbl, idx, 1);
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        guid_prefix[j] = (data_0 >> (8 * (j + 4))) & 0xff;
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
        guid_prefix[j + 4] = (data_1 >> (8 * j)) & 0xff;
    }

    uint32_t pub_acknack_cnt, sub_acknack_cnt;
    get_sedp_reader_tbl_acknack_cnt(&pub_acknack_cnt, &sub_acknack_cnt, tbl,
                                    idx);
    sub_acknack_cnt++;
    set_sedp_reader_tbl_acknack_cnt(pub_acknack_cnt, sub_acknack_cnt, tbl, idx);

    sedp_acknack_out(MSG_TYPE_SEDP_ACKNACK_SUB, ip_addr, udp_port, guid_prefix,
                     snstate_base, snstate_is_empty, sub_acknack_cnt,
                     msg_metadata);

    if (acknack_req) {
        set_sedp_reader_tbl(data_0 & ~req_flag_mask, tbl, idx, 0);
    }
}

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
    hls_stream<message_metadata_t> &out, sedp_reader_tbl_t *sedp_reader_tbl,
    app_endpoint             app_reader_tbl[APP_READER_MAX],
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    const config_t *conf, hls_uint<1> cnt_interval_elapsed,
    VOLATILE uint8_t *cnt_interval_set, hls_uint<1> cnt_spdp_wr_elapsed,
    VOLATILE uint8_t *cnt_spdp_wr_set, hls_uint<1> cnt_sedp_pub_wr_elapsed,
    VOLATILE uint8_t *cnt_sedp_pub_wr_set, hls_uint<1> cnt_sedp_sub_wr_elapsed,
    VOLATILE uint8_t *cnt_sedp_sub_wr_set, hls_uint<1> cnt_sedp_pub_hb_elapsed,
    VOLATILE uint8_t *cnt_sedp_pub_hb_set, hls_uint<1> cnt_sedp_sub_hb_elapsed,
    VOLATILE uint8_t *cnt_sedp_sub_hb_set, hls_uint<1> cnt_sedp_pub_an_elapsed,
    VOLATILE uint8_t *cnt_sedp_pub_an_set, hls_uint<1> cnt_sedp_sub_an_elapsed,
    VOLATILE uint8_t *cnt_sedp_sub_an_set, hls_uint<1> cnt_app_wr_elapsed,
    VOLATILE uint8_t *cnt_app_wr_set, int64_t timestamp_i64) {
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

    static hls_uint<7> tx_progress;
    static_assert(
        SEDP_READER_MAX <= 128,
        "'tx_progress' must be able to represent SEDP_READER_MAX - 1.");
    static_assert(
        APP_READER_MAX <= 128,
        "'tx_progress' must be able to represent APP_READER_MAX - 1.");

    static hls_uint<8> tx_cnt_elapsed;
    static_assert(
        SEDP_READER_MAX <= 255,
        "'tx_cnt_elapsed' should be able to represent SEDP_READER_MAX.");
    static_assert(
        PUB_TOPICS_MAX <= 255,
        "'tx_cnt_elapsed' should be able to represent PUB_TOPICS_MAX.");
    static_assert(
        SUB_TOPICS_MAX <= 255,
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

    if (!out.full() && cnt_interval_elapsed) {
        if (((pub_enable != 0) || (sub_enable != 0)) && next_packet_type == 0) {
            // Send a SPDP message if
            //   1. cnt_spdp_wr_elapsed is asserted.
            //   2. there exists a living sedp_endpoint whose
            //      initial_send_counter is less than three.
            bool send = cnt_spdp_wr_elapsed;
            if (!send) {
#ifdef SEDP_READER_TBL_FF
                /* Cyber unroll_times=all */
#else  // !SEDP_READER_TBL_FF
                /* Cyber folding=1 */
#endif // SEDP_READER_TBL_FF
                for (auto j = 0; j < SEDP_READER_MAX; j++) {
#ifdef SEDP_READER_TBL_FF
#pragma HLS unroll
#endif // SEDP_READER_TBL_FF
                    if (get_ros2_out_spdp_info(sedp_reader_tbl, j)) {
                        send = true;
                    }
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
                if (tx_progress < SEDP_READER_MAX) {
                    SEDP_PUB_WRITER_OUT(
                        cnt_sedp_pub_wr_elapsed, tx_topic_progress,
                        default_port,
                        app_writer_entity_id_list[tx_topic_progress],
                        sedp_reader_tbl, tx_progress, &msg_metadata);
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
                if (tx_progress < SEDP_READER_MAX) {
                    SEDP_SUB_WRITER_OUT(
                        cnt_sedp_sub_wr_elapsed, tx_topic_progress,
                        default_port,
                        app_reader_entity_id_list[tx_topic_progress],
                        sedp_reader_tbl, tx_progress, &msg_metadata);
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

            if (tx_progress < SEDP_READER_MAX) {
                SEDP_PUB_HEARTBEAT_OUT(cnt_sedp_pub_hb_elapsed, sedp_reader_tbl,
                                       tx_progress, &msg_metadata);
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

            if (tx_progress < SEDP_READER_MAX) {
                SEDP_SUB_HEARTBEAT_OUT(cnt_sedp_sub_hb_elapsed, sedp_reader_tbl,
                                       tx_progress, &msg_metadata);
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
                SEDP_PUB_ACKNACK_OUT(cnt_sedp_pub_an_elapsed, sedp_reader_tbl,
                                     tx_progress, &msg_metadata);
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
                SEDP_SUB_ACKNACK_OUT(cnt_sedp_sub_an_elapsed, sedp_reader_tbl,
                                     tx_progress, &msg_metadata);
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

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2_main(
    hls_stream<rtps_data_t> &in /* Cyber port_mode=axi_stream */,
    hls_stream<message_metadata_t>
        &out /* Cyber port_mode=axi_stream:reg_both */,
#ifdef SEDP_READER_TBL_RAM
    sedp_reader_tbl_t *sedp_reader_tbl,
#endif // SEDP_READER_TBL_RAM
    hls_uint<PUB_TOPICS_MAX> pub_enable /* Cyber port_mode=in */,
    hls_uint<SUB_TOPICS_MAX> sub_enable /* Cyber port_mode=in */,
    const config_t          *conf /* Cyber port_mode=in, stable_input */,

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

#ifdef SEDP_READER_TBL_RAM
#pragma HLS disaggregate variable = sedp_reader_tbl
#pragma HLS interface mode = ap_memory port                                    \
    = sedp_reader_tbl->ram storage_type = ram_1p latency = 1
#endif // SEDP_READER_TBL_RAM

#pragma HLS interface mode = ap_none port = pub_enable
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS disaggregate             variable = conf
#pragma HLS disaggregate             variable = conf->participant_lease_duration
#pragma HLS interface mode = ap_none port                                      \
    = conf->participant_lease_duration.seconds
#pragma HLS interface mode = ap_none port                                      \
    = conf->participant_lease_duration.fraction
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

#ifdef SEDP_READER_TBL_FF
    static sedp_reader_tbl_t sedp_reader_tbl;
#pragma HLS array_partition variable = sedp_reader_tbl.ram complete dim = 0
#endif

    static app_endpoint app_reader_tbl[APP_READER_MAX];
#pragma HLS array_partition variable = app_reader_tbl complete dim = 0

    ros2_in(in,
#ifdef SEDP_READER_TBL_FF
            &sedp_reader_tbl,
#else  // !SEDP_READR_TBL_FF
            sedp_reader_tbl,
#endif // SEDP_READER_TBL_FF
            app_reader_tbl, pub_enable, sub_enable, timestamp_i64);

    ros2_out(
        out,
#ifdef SEDP_READER_TBL_FF
        &sedp_reader_tbl,
#else  // !SEDP_READR_TBL_FF
        sedp_reader_tbl,
#endif // SEDP_READER_TBL_FF
        app_reader_tbl, pub_enable, sub_enable, conf, cnt_interval_elapsed,
        cnt_interval_set, cnt_spdp_wr_elapsed, cnt_spdp_wr_set,
        cnt_sedp_pub_wr_elapsed, cnt_sedp_pub_wr_set, cnt_sedp_sub_wr_elapsed,
        cnt_sedp_sub_wr_set, cnt_sedp_pub_hb_elapsed, cnt_sedp_pub_hb_set,
        cnt_sedp_sub_hb_elapsed, cnt_sedp_sub_hb_set, cnt_sedp_pub_an_elapsed,
        cnt_sedp_pub_an_set, cnt_sedp_sub_an_elapsed, cnt_sedp_sub_an_set,
        cnt_app_wr_elapsed, cnt_app_wr_set, timestamp_i64);
}
