// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "app.hpp"
#include "common.hpp"
#include "duration.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "message_metadata.hpp"
#include "ros2.hpp"
#include "slip.hpp"
#include "spdp.hpp"
#include "udp.hpp"
#include <cstdint>

#define USE_FIFOIF_ETHERNET

/* Cyber func=inline */
static void spdp_writer_out(const sender_config_t    *conf,
                            const message_metadata_t *msg_metadata,
                            hls_stream<uint8_t>      &out) {
    uint8_t metatraffic_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = metatraffic_port type = complete dim = 1
    uint8_t default_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = default_port type = complete dim = 1
    duration lease_duration;

    deserialize_spdp_metadata(metatraffic_port, default_port, &lease_duration,
                              msg_metadata);

    ip_set_header(conf->ip_addr, msg_metadata->dst_addr, IP_HDR_TTL_MULTICAST,
                  SPDP_WRITER_UDP_PKT_LEN, out);

    udp_set_header(conf->node_udp_port, msg_metadata->dst_port,
                   SPDP_WRITER_RTPS_PKT_LEN, out);

    spdp_writer(conf->guid_prefix, conf->ip_addr, metatraffic_port,
                conf->ip_addr, default_port, lease_duration, out,
                conf->node_name, conf->node_name_len, msg_metadata->now);
}

/* Cyber func=inline */
static void sedp_writer_out(const uint8_t writer_entity_id[4],
                            const uint8_t reader_entity_id[4],
                            const uint8_t topic_name[], uint8_t topic_name_len,
                            const uint8_t             topic_type_name[],
                            uint8_t                   topic_type_name_len,
                            const sender_config_t    *conf,
                            const message_metadata_t *msg_metadata,
                            hls_stream<uint8_t>      &out) {
    uint8_t reader_guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = reader_guid_prefix type = complete dim  \
    = 1
    int64_t seqnum;
    uint8_t usertraffic_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = usertraffic_port type = complete dim = 1
    uint8_t app_entity_id[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = app_entity_id type = complete dim = 1

    deserialize_sedp_metadata(reader_guid_prefix, &seqnum, usertraffic_port,
                              app_entity_id, msg_metadata);

    ip_set_header(conf->ip_addr, msg_metadata->dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_WRITER_UDP_PKT_LEN, out);

    udp_set_header(conf->node_udp_port, msg_metadata->dst_port,
                   SEDP_WRITER_RTPS_PKT_LEN, out);

    sedp_writer(conf->guid_prefix, writer_entity_id, reader_guid_prefix,
                reader_entity_id, seqnum, conf->ip_addr, usertraffic_port,
                app_entity_id, out, topic_name, topic_name_len, topic_type_name,
                topic_type_name_len, msg_metadata->now);
}

/* Cyber func=inline */
static void sedp_heartbeat_out(const uint8_t             writer_entity_id[4],
                               const uint8_t             reader_entity_id[4],
                               const sender_config_t    *conf,
                               const message_metadata_t *msg_metadata,
                               hls_stream<uint8_t>      &out) {
    uint8_t reader_guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = reader_guid_prefix type = complete dim  \
    = 1
    int64_t  first_seqnum;
    int64_t  last_seqnum;
    uint32_t cnt;

    deserialize_sedp_heartbeat_metadata(reader_guid_prefix, &first_seqnum,
                                        &last_seqnum, &cnt, msg_metadata);

    ip_set_header(conf->ip_addr, msg_metadata->dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_HEARTBEAT_UDP_PKT_LEN, out);

    udp_set_header(conf->node_udp_port, msg_metadata->dst_port,
                   SEDP_HEARTBEAT_RTPS_PKT_LEN, out);

    sedp_heartbeat(conf->guid_prefix, writer_entity_id, reader_guid_prefix,
                   reader_entity_id, first_seqnum, last_seqnum, cnt, out);
}

/* Cyber func=inline */
static void sedp_acknack_out(const uint8_t             writer_entity_id[4],
                             const uint8_t             reader_entity_id[4],
                             const sender_config_t    *conf,
                             const message_metadata_t *msg_metadata,
                             hls_stream<uint8_t>      &out) {
    uint8_t reader_guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = reader_guid_prefix type = complete dim  \
    = 1
    uint8_t  snstate_base;
    bool     snstate_empty;
    uint32_t cnt;

    deserialize_sedp_acknack_metadata(reader_guid_prefix, &snstate_base,
                                      &snstate_empty, &cnt, msg_metadata);

    ip_set_header(conf->ip_addr, msg_metadata->dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_ACKNACK_UDP_PKT_LEN, out);

    udp_set_header(conf->node_udp_port, msg_metadata->dst_port,
                   SEDP_ACKNACK_RTPS_PKT_LEN, out);

    sedp_acknack(conf->guid_prefix, writer_entity_id, reader_guid_prefix,
                 reader_entity_id, snstate_base, snstate_empty, cnt, out);
}

/* Cyber func=inline */
static void app_writer_out(
#ifdef PUB_DATA_FF
    VOLATILE
#endif // PUB_DATA_FF
    const uint32_t pub_app_data[MAX_APP_DATA_LEN / 4],
    app_data_len_t pub_app_data_len, const sender_config_t *conf,
    const message_metadata_t *msg_metadata, hls_stream<uint8_t> &out) {
    uint8_t reader_guid_prefix[12] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = reader_guid_prefix type = complete dim  \
    = 1
    uint8_t reader_entity_id[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = reader_entity_id type = complete dim = 1
    uint8_t writer_entity_id[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = writer_entity_id type = complete dim = 1
    int64_t seqnum;

    deserialize_app_metadata(reader_guid_prefix, reader_entity_id,
                             writer_entity_id, &seqnum, msg_metadata);

    ip_set_header(conf->ip_addr, msg_metadata->dst_addr, IP_HDR_TTL_UNICAST,
                  APP_WRITER_UDP_PKT_LEN(pub_app_data_len), out);

    udp_set_header(conf->node_udp_port, msg_metadata->dst_port,
                   APP_WRITER_RTPS_PKT_LEN(pub_app_data_len), out);

    app_writer(conf->guid_prefix, writer_entity_id, reader_guid_prefix,
               reader_entity_id, seqnum, pub_app_data, pub_app_data_len, out,
               msg_metadata->now);
}

/* Cyber func=inline */
static void APP_WRITER_OUT(
#ifdef PUB_DATA_FF
    VOLATILE
#endif // PUB_DATA_FF
    const uint32_t                 pub_app_data[MAX_APP_DATA_LEN / 4],
    VOLATILE const app_data_len_t *pub_app_data_len,
    VOLATILE uint8_t *pub_app_data_req, VOLATILE uint8_t *pub_app_data_rel,
    VOLATILE uint8_t *pub_app_data_grant, const sender_config_t *conf,
    const message_metadata_t *msg_metadata, hls_stream<uint8_t> &out) {
#pragma HLS inline
    uint8_t grant;
    /* Cyber scheduling_block = non-transparent */
app_data_request_section: {
#pragma HLS protocol fixed
    *pub_app_data_req = 0 /* write dummy value to assert valid signal */;
    CLOCK_BOUNDARY;
    CLOCK_BOUNDARY;
    grant = *pub_app_data_grant;
}

    if (grant != 0) {
        app_writer_out(pub_app_data, *pub_app_data_len, conf, msg_metadata,
                       out);

        /* Cyber scheduling_block = non-transparent */
    app_data_release_section: {
#pragma HLS protocol fixed
        CLOCK_BOUNDARY;
        *pub_app_data_rel = 0 /* write dummy value to assert valid signal */;
        CLOCK_BOUNDARY;
        CLOCK_BOUNDARY;
    }
    }
}

/* Cyber func=process, bdltran_option=-s, process_valid=NO,
   async_reset_port=rst_n- */
void ros2_sender(
    hls_stream<message_metadata_t> &in /* Cyber port_mode=axi_stream */,
    hls_stream<uint8_t>            &out /* Cyber port_mode=cw_fifo */,
    const sender_config_t          *conf /* Cyber port_mode=in, stable_input,
                                            port_synchronizer=INPUT_PORT_SYNC_REGS */
    ,

#ifdef PUB_DATA_FF
    VOLATILE
#endif // PUB_DATA_FF
    const uint32_t pub_app_data_0[MAX_APP_DATA_LEN / 4]
#ifdef PUB_DATA_FF
/* Cyber array=EXPAND, port_mode=shared, volatile=YES,
 * port_synchronizer=INPUT_PORT_SYNC_REGS */
#endif // PUB_DATA_FF
    ,
    VOLATILE const app_data_len_t
        *pub_app_data_len_0 /* Cyber port_mode=cw_fifo, volatile=YES,
                               port_synchronizer=INPUT_PORT_SYNC_REGS */
    ,
    VOLATILE uint8_t
        *pub_app_data_req_0 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_rel_0 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_grant_0 /* Cyber port_mode=shared, volatile=YES */,

#ifdef PUB_DATA_FF
    VOLATILE
#endif // PUB_DATA_FF
    const uint32_t pub_app_data_1[MAX_APP_DATA_LEN / 4]
#ifdef PUB_DATA_FF
/* Cyber array=EXPAND, port_mode=shared, volatile=YES,
 * port_synchronizer=INPUT_PORT_SYNC_REGS */
#endif // PUB_DATA_FF
    ,
    VOLATILE const app_data_len_t
        *pub_app_data_len_1 /* Cyber port_mode=cw_fifo, volatile=YES,
                               port_synchronizer=INPUT_PORT_SYNC_REGS */
    ,
    VOLATILE uint8_t
        *pub_app_data_req_1 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_rel_1 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_grant_1 /* Cyber port_mode=shared, volatile=YES */,

#ifdef PUB_DATA_FF
    VOLATILE
#endif // PUB_DATA_FF
    const uint32_t pub_app_data_2[MAX_APP_DATA_LEN / 4]
#ifdef PUB_DATA_FF
/* Cyber array=EXPAND, port_mode=shared, volatile=YES,
 * port_synchronizer=INPUT_PORT_SYNC_REGS */
#endif // PUB_DATA_FF
    ,
    VOLATILE const app_data_len_t
        *pub_app_data_len_2 /* Cyber port_mode=cw_fifo, volatile=YES,
                               port_synchronizer=INPUT_PORT_SYNC_REGS */
    ,
    VOLATILE uint8_t
        *pub_app_data_req_2 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_rel_2 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_grant_2 /* Cyber port_mode=shared, volatile=YES */,

#ifdef PUB_DATA_FF
    VOLATILE
#endif // PUB_DATA_FF
    const uint32_t pub_app_data_3[MAX_APP_DATA_LEN / 4]
#ifdef PUB_DATA_FF
/* Cyber array=EXPAND, port_mode=shared, volatile=YES,
 * port_synchronizer=INPUT_PORT_SYNC_REGS */
#endif // PUB_DATA_FF
    ,
    VOLATILE const app_data_len_t
        *pub_app_data_len_3 /* Cyber port_mode=cw_fifo, volatile=YES,
                               port_synchronizer=INPUT_PORT_SYNC_REGS */
    ,
    VOLATILE uint8_t
        *pub_app_data_req_3 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_rel_3 /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
        *pub_app_data_grant_3 /* Cyber port_mode=shared, volatile=YES */) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = ap_fifo port = out
#pragma HLS disaggregate             variable = conf
#pragma HLS array_reshape variable = conf->ip_addr type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->ip_addr
#pragma HLS array_reshape variable = conf->node_name type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->node_name
#pragma HLS interface mode = ap_none port = conf->node_name_len
#pragma HLS array_reshape variable = conf->node_udp_port type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->node_udp_port
#pragma HLS array_reshape variable = conf->guid_prefix type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->guid_prefix
#pragma HLS array_reshape variable = conf->pub_topic_name type = complete dim  \
    = 2
#pragma HLS array_partition variable = conf->pub_topic_name type               \
    = complete                                              dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_name
#pragma HLS array_partition variable = conf->pub_topic_name_len type           \
    = complete                                                  dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_name_len
#pragma HLS array_reshape variable = conf->pub_topic_type_name type            \
    = complete                                                 dim = 2
#pragma HLS array_partition variable = conf->pub_topic_type_name type          \
    = complete                                                   dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_type_name
#pragma HLS array_partition variable = conf->pub_topic_type_name_len type      \
    = complete                                                       dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_type_name_len
#pragma HLS array_reshape variable = conf->sub_topic_name type = complete dim  \
    = 2
#pragma HLS array_partition variable = conf->sub_topic_name type               \
    = complete                                              dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_name
#pragma HLS array_partition variable = conf->sub_topic_name_len type           \
    = complete                                                  dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_name_len
#pragma HLS array_reshape variable = conf->sub_topic_type_name type            \
    = complete                                                 dim = 2
#pragma HLS array_partition variable = conf->sub_topic_type_name type          \
    = complete                                                   dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_type_name
#pragma HLS array_partition variable = conf->sub_topic_type_name_len type      \
    = complete                                                       dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_type_name_len

#ifdef PUB_DATA_FF
#pragma HLS interface mode = ap_fifo port = pub_app_data_0
#pragma HLS interface mode = ap_fifo port = pub_app_data_1
#pragma HLS interface mode = ap_fifo port = pub_app_data_2
#pragma HLS interface mode = ap_fifo port = pub_app_data_3
#pragma HLS array_reshape variable = pub_app_data_0 type = complete dim = 0
#pragma HLS array_reshape variable = pub_app_data_1 type = complete dim = 0
#pragma HLS array_reshape variable = pub_app_data_2 type = complete dim = 0
#pragma HLS array_reshape variable = pub_app_data_3 type = complete dim = 0
#endif // PUB_DATA_FF
#ifdef PUB_DATA_RAM
#pragma HLS interface mode = ap_memory port = pub_app_data_0 storage_type      \
    = rom_1p                                                 latency = 1
#pragma HLS interface mode = ap_memory port = pub_app_data_1 storage_type      \
    = rom_1p                                                 latency = 1
#pragma HLS interface mode = ap_memory port = pub_app_data_2 storage_type      \
    = rom_1p                                                 latency = 1
#pragma HLS interface mode = ap_memory port = pub_app_data_3 storage_type      \
    = rom_1p                                                 latency = 1
#endif // PUB_DATA_RAM

#pragma HLS interface mode = ap_fifo port = pub_app_data_len_0
#pragma HLS interface mode = ap_fifo port = pub_app_data_len_1
#pragma HLS interface mode = ap_fifo port = pub_app_data_len_2
#pragma HLS interface mode = ap_fifo port = pub_app_data_len_3
#pragma HLS interface mode = ap_vld port = pub_app_data_req_0
#pragma HLS interface mode = ap_vld port = pub_app_data_req_1
#pragma HLS interface mode = ap_vld port = pub_app_data_req_2
#pragma HLS interface mode = ap_vld port = pub_app_data_req_3
#pragma HLS interface mode = ap_vld port = pub_app_data_rel_0
#pragma HLS interface mode = ap_vld port = pub_app_data_rel_1
#pragma HLS interface mode = ap_vld port = pub_app_data_rel_2
#pragma HLS interface mode = ap_vld port = pub_app_data_rel_3
#pragma HLS interface mode = ap_ack port = pub_app_data_grant_0
#pragma HLS interface mode = ap_ack port = pub_app_data_grant_1
#pragma HLS interface mode = ap_ack port = pub_app_data_grant_2
#pragma HLS interface mode = ap_ack port = pub_app_data_grant_3

    static const uint8_t pub_writer_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_WRITER;
    static const uint8_t sub_writer_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_WRITER;
#pragma HLS array_partition variable = pub_writer_entity_id complete dim = 0
#pragma HLS array_partition variable = sub_writer_entity_id complete dim = 0

    static const uint8_t pub_reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
    static const uint8_t sub_reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = pub_reader_entity_id complete dim = 0
#pragma HLS array_partition variable = sub_reader_entity_id complete dim = 0

    const message_metadata_t msg_metadata = in.read();
    const topic_id_t         topic_id = msg_metadata.topic_id;
#pragma HLS array_partition variable = msg_metadata.dst_addr type              \
    = complete                                               dim = 1
#pragma HLS array_partition variable = msg_metadata.dst_port type              \
    = complete                                               dim = 1
#pragma HLS array_partition variable = msg_metadata.rtps_data type             \
    = complete                                                dim = 1

    if (msg_metadata.message_type == MSG_TYPE_SPDP) {
        spdp_writer_out(conf, &msg_metadata, out);
    } else if ((msg_metadata.message_type == MSG_TYPE_SEDP_PUB)
               && (topic_id < PUB_TOPICS_MAX)) {
        sedp_writer_out(
            pub_writer_entity_id, pub_reader_entity_id,
            conf->pub_topic_name[topic_id], conf->pub_topic_name_len[topic_id],
            conf->pub_topic_type_name[topic_id],
            conf->pub_topic_type_name_len[topic_id], conf, &msg_metadata, out);
    } else if ((msg_metadata.message_type == MSG_TYPE_SEDP_SUB)
               && (topic_id < SUB_TOPICS_MAX)) {
        sedp_writer_out(
            sub_writer_entity_id, sub_reader_entity_id,
            conf->sub_topic_name[topic_id], conf->sub_topic_name_len[topic_id],
            conf->sub_topic_type_name[topic_id],
            conf->sub_topic_type_name_len[topic_id], conf, &msg_metadata, out);
    } else if (msg_metadata.message_type == MSG_TYPE_SEDP_HEARTBEAT_PUB) {
        sedp_heartbeat_out(pub_writer_entity_id, pub_reader_entity_id, conf,
                           &msg_metadata, out);
    } else if (msg_metadata.message_type == MSG_TYPE_SEDP_HEARTBEAT_SUB) {
        sedp_heartbeat_out(sub_writer_entity_id, sub_reader_entity_id, conf,
                           &msg_metadata, out);
    } else if (msg_metadata.message_type == MSG_TYPE_SEDP_ACKNACK_PUB) {
        sedp_acknack_out(pub_writer_entity_id, pub_reader_entity_id, conf,
                         &msg_metadata, out);
    } else if (msg_metadata.message_type == MSG_TYPE_SEDP_ACKNACK_SUB) {
        sedp_acknack_out(sub_writer_entity_id, sub_reader_entity_id, conf,
                         &msg_metadata, out);
    } else if (msg_metadata.message_type == MSG_TYPE_APP) {
        switch (topic_id) {
        case 0:
            APP_WRITER_OUT(pub_app_data_0, pub_app_data_len_0,
                           pub_app_data_req_0, pub_app_data_rel_0,
                           pub_app_data_grant_0, conf, &msg_metadata, out);
            break;
        case 1:
            APP_WRITER_OUT(pub_app_data_1, pub_app_data_len_1,
                           pub_app_data_req_1, pub_app_data_rel_1,
                           pub_app_data_grant_1, conf, &msg_metadata, out);
            break;
        case 2:
            APP_WRITER_OUT(pub_app_data_2, pub_app_data_len_2,
                           pub_app_data_req_2, pub_app_data_rel_2,
                           pub_app_data_grant_2, conf, &msg_metadata, out);
            break;
        case 3:
            APP_WRITER_OUT(pub_app_data_3, pub_app_data_len_3,
                           pub_app_data_req_3, pub_app_data_rel_3,
                           pub_app_data_grant_3, conf, &msg_metadata, out);
            break;
        }
    }
}
