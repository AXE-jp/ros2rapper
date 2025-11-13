// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "duration.hpp"
#include "endpoint.hpp"
#include "ip.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include "sedp.hpp"
#include "spdp.hpp"
#include "util.hpp"

/* Cyber func=inline */
static topic_id_t get_matched_pub_topic_id(hls_uint<PUB_TOPICS_MAX> matched) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < PUB_TOPICS_MAX; j++) {
#pragma HLS unroll
        if (matched[j]) {
            return static_cast<topic_id_t>(j);
        }
    }
    return 0;
}

#define FLAGS_FOUND_GUID     0x01
#define FLAGS_FOUND_LOCATOR  0x02
#define FLAGS_UNMATCH_DOMAIN 0x04
#define FLAGS_UNMATCH_TOPIC  0x08
#define FLAGS_UNMATCH_TYPE   0x10

enum {
    SEDP_READ_HDR_PROTOCOL,
    SEDP_READ_HDR_GUID_PREFIX,
    SEDP_READ_SBM_HDR,
    SEDP_READ_INFO_DST,
    SEDP_READ_HEARTBEAT,
    SEDP_READ_SBM_DATA_HDR,
    SEDP_READ_SP_HDR,
    SEDP_READ_PARAM_ID,
    SEDP_READ_PARAM_LEN,
    SEDP_READ_PARAM,
    SEDP_READ_SKIP_SBM,
    SEDP_READ_DO_NOTHING,
};

/* Cyber func=inline */
void sedp_reader(
    hls_uint<9> in, hls_stream<rtps_data_t> &out,
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    const uint8_t ip_addr[4], const uint8_t subnet_mask[4],
    uint16_t port_num_seed, const uint8_t guid_prefix[12],
    const uint8_t pub_topic_name[PUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t pub_topic_name_len[PUB_TOPICS_MAX],
    const uint8_t pub_type_name[PUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t pub_type_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_topic_name[SUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t sub_topic_name_len[SUB_TOPICS_MAX],
    const uint8_t sub_type_name[SUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t sub_type_name_len[SUB_TOPICS_MAX]) {
#pragma HLS inline
    static const uint8_t pub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
#pragma HLS array_partition variable = pub_reader_id complete dim = 0
    static const uint8_t sub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = sub_reader_id complete dim = 0

    static hls_uint<4>       state;
    static uint16_t          offset;
    static hls_uint<5>       flags;
    static builtin_ep_type_t ep_type;

    static hls_uint<PUB_TOPICS_MAX> pub_topics_unmatched;
    static hls_uint<PUB_TOPICS_MAX> pub_types_unmatched;

    static hls_uint<SUB_TOPICS_MAX> sub_topics_unmatched;
    static hls_uint<SUB_TOPICS_MAX> sub_types_unmatched;

    static uint8_t  sbm_id;
    static bool     sbm_le;
    static uint16_t sbm_len;
    static uint8_t  sbm_sn_0;
    static uint8_t  sbm_sn_1;
    static uint16_t rep_id;
    static uint16_t param_id;
    static uint16_t param_len;
    static uint32_t sp_len;

    static uint8_t
        sedp_guid_prefix[12] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = sedp_guid_prefix complete dim = 1
    static uint8_t sedp_ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = sedp_ip_addr complete dim = 1
    static uint8_t sedp_udp_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = sedp_udp_port complete dim = 1
    static uint8_t
        sedp_entity_id[4] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = sedp_entity_id complete dim = 1

    if ((pub_enable == 0) && (sub_enable == 0)) {
        return;
    }

    uint8_t data = in & 0xff;
    bool    end = in & 0x100;

    switch (state) {
    case SEDP_READ_HDR_PROTOCOL:
        if (!rtps_compare_protocol(offset, data)) {
            state = SEDP_READ_DO_NOTHING;
            break;
        }
        offset++;
        if (offset == RTPS_HDR_OFFSET_GUID_PREFIX) {
            offset = 0;
            state = SEDP_READ_HDR_GUID_PREFIX;
        }
        break;
    case SEDP_READ_HDR_GUID_PREFIX:
        if (offset < 12) {
            sedp_guid_prefix[offset] = data;
        }
        offset++;
        if (offset == RTPS_HDR_SIZE - RTPS_HDR_OFFSET_GUID_PREFIX) {
            offset = 0;
            state = SEDP_READ_SBM_HDR;
        }
        break;
    case SEDP_READ_SBM_HDR:
        switch (offset) {
        case SBM_HDR_OFFSET_SUBMESSAGE_ID:
            sbm_id = data;
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_le = data & SBM_FLAGS_ENDIANNESS ? true : false;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER:
            sbm_len = sbm_le ? data : data << 8;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER + 1:
            sbm_len |= sbm_le ? data << 8 : data;
        }
        offset++;
        if (offset == SBM_HDR_SIZE) {
            offset = 0;
            if (sbm_id == SBM_ID_INFO_DST) {
                state = SEDP_READ_INFO_DST;
            } else if (sbm_id == SBM_ID_DATA) {
                ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
                state = SEDP_READ_SBM_DATA_HDR;
            } else if (sbm_id == SBM_ID_HEARTBEAT) {
                ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
                state = SEDP_READ_HEARTBEAT;
            } else
                state = SEDP_READ_SKIP_SBM;
        }
        break;
    case SEDP_READ_INFO_DST:
        if (offset < 12) {
            if (guid_prefix[offset] != data) {
                state = SEDP_READ_DO_NOTHING;
                break;
            }
        }
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = SEDP_READ_SBM_HDR;
        }
        break;
    case SEDP_READ_HEARTBEAT:
        if (!rtps_compare_heartbeat_hdr_reader_id(offset, data,
                                                  pub_reader_id)) {
            ep_type &= ~(BUILTIN_EP_PUB);
        }
        if (!rtps_compare_heartbeat_hdr_reader_id(offset, data,
                                                  sub_reader_id)) {
            ep_type &= ~(BUILTIN_EP_SUB);
        }
        // Ignore other than lower 8-bit of Sequence Number to reduce resources
        if (sbm_le && offset == SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 4) {
            sbm_sn_0 = data;
        } else if (!sbm_le
                   && offset == SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 7) {
            sbm_sn_0 = data;
        } else if (sbm_le && offset == SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 4) {
            sbm_sn_1 = data;
        } else if (!sbm_le && offset == SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 7) {
            sbm_sn_1 = data;
        }

        offset++;

        if (offset == sbm_len) {
            rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
            /* Cyber unroll_times=all */
            for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
                rtps_data.guid_prefix[j] = sedp_guid_prefix[j];
            }
            rtps_data.data[0] = sbm_sn_0;
            rtps_data.data[1] = sbm_sn_1;
            if (ep_type & BUILTIN_EP_PUB) {
                rtps_data.type = RTPS_TYPE_SEDP_HEARTBEAT_PUB;
                out.write(rtps_data);
            } else if (ep_type & BUILTIN_EP_SUB) {
                rtps_data.type = RTPS_TYPE_SEDP_HEARTBEAT_SUB;
                out.write(rtps_data);
            }
            offset = 0;
            state = SEDP_READ_SBM_HDR;
        }
        break;
    case SEDP_READ_SBM_DATA_HDR: // parse sub-message header
        if (!rtps_compare_data_hdr_reader_id(offset, data, pub_reader_id)) {
            ep_type &= ~(BUILTIN_EP_PUB);
        }
        if (!rtps_compare_data_hdr_reader_id(offset, data, sub_reader_id)) {
            ep_type &= ~(BUILTIN_EP_SUB);
        }

        // Ignore other than lower 8-bit of Sequence Number to reduce resources
        if (sbm_le && offset == SBM_DATA_HDR_OFFSET_WRITER_SN + 4) {
            sbm_sn_0 = data;
        } else if (!sbm_le && offset == SBM_DATA_HDR_OFFSET_WRITER_SN + 7) {
            sbm_sn_0 = data;
        }

        offset++;

        if (ep_type == 0) {
            state = SEDP_READ_SKIP_SBM;
            break;
        }
        if (offset == SBM_DATA_HDR_SIZE) {
            sbm_len -= SBM_DATA_HDR_SIZE;
            offset = 0;
            state = SEDP_READ_SP_HDR;
        }
        break;
    case SEDP_READ_SP_HDR:
        switch (offset) {
        case SP_HDR_OFFSET_REPRESENTATION_ID:
            rep_id = data << 8;
            break;
        case SP_HDR_OFFSET_REPRESENTATION_ID + 1:
            rep_id |= data;
        }
        offset++;
        if (offset == SP_HDR_SIZE) {
            sbm_len -= SP_HDR_SIZE;
            offset = 0;
            state = SEDP_READ_PARAM_ID;
        }
        break;
    case SEDP_READ_PARAM_ID:
        if (offset == 0)
            param_id = rep_id & SP_ID_CDR_LE ? data : data << 8;
        else
            param_id |= rep_id & SP_ID_CDR_LE ? data << 8 : data;
        offset++;
        if (offset == sizeof(param_id)) {
            sbm_len -= sizeof(param_id);
            offset = 0;
            state = SEDP_READ_PARAM_LEN;
        }
        break;
    case SEDP_READ_PARAM_LEN:
        if (offset == 0)
            param_len = rep_id & SP_ID_CDR_LE ? data : data << 8;
        else
            param_len |= rep_id & SP_ID_CDR_LE ? data << 8 : data;
        offset++;
        if (offset == sizeof(param_len)) {
            if (param_id == PID_SENTINEL) {
                rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
                /* Cyber unroll_times=all */
                for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
                    rtps_data.guid_prefix[j] = sedp_guid_prefix[j];
                }
                /* Cyber unroll_times=all */
                for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
                    rtps_data.data[j] = sedp_ip_addr[j];
                }
                rtps_data.data[4] = sedp_udp_port[0];
                rtps_data.data[5] = sedp_udp_port[1];
                /* Cyber unroll_times=all */
                for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
                    rtps_data.data[j + 6] = sedp_entity_id[j];
                }
                rtps_data.data[11] = sbm_sn_0;

                hls_uint<5> found = FLAGS_FOUND_GUID | FLAGS_FOUND_LOCATOR;
                if (flags == found) {
                    if (ep_type & BUILTIN_EP_SUB) {
                        rtps_data.data[10] = get_matched_pub_topic_id(
                            ~pub_topics_unmatched & ~pub_types_unmatched);
                        rtps_data.type = RTPS_TYPE_SEDP_SUB;
                    } else {
                        rtps_data.type = RTPS_TYPE_SEDP_PUB;
                    }
                } else {
                    if (ep_type & BUILTIN_EP_SUB) {
                        rtps_data.type = RTPS_TYPE_SEDP_SUB_SN_ONLY;
                    } else {
                        rtps_data.type = RTPS_TYPE_SEDP_PUB_SN_ONLY;
                    }
                }
                out.write(rtps_data);
                flags = 0;
                pub_topics_unmatched = 0;
                pub_types_unmatched = 0;
                sub_topics_unmatched = 0;
                sub_types_unmatched = 0;
                offset = 0;
                state = SEDP_READ_SBM_HDR;
            } else {
                sbm_len -= sizeof(param_len);
                param_len = ROUND_UP(param_len, 4);
                offset = 0;
                state = (param_len == 0) ? SEDP_READ_PARAM_ID : SEDP_READ_PARAM;
            }
        }
        break;
    case SEDP_READ_PARAM:
        switch (param_id) {
        case PID_UNICAST_LOCATOR:
            if (flags & FLAGS_FOUND_LOCATOR)
                break;
            if (offset < 4) {
                ; // do nothing
            } else if (offset < 8) {
                if (rep_id & SP_ID_CDR_LE) {
                    if (offset == 4) {
                        sedp_udp_port[1] = data;
                    } else if (offset == 5) {
                        sedp_udp_port[0] = data;
                    }
                } else {
                    if (offset == 6) {
                        sedp_udp_port[0] = data;
                    } else if (offset == 7) {
                        sedp_udp_port[1] = data;
                    }
                }
            } else if (offset < 20) {
                ; // do nothing
            } else if (offset == 20) {
                sedp_ip_addr[0] = data;
            } else if (offset == 21) {
                sedp_ip_addr[1] = data;
            } else if (offset == 22) {
                sedp_ip_addr[2] = data;
            } else if (offset == 23) {
                sedp_ip_addr[3] = data;
            }
            break;
        case PID_TOPIC_NAME:
            if (offset < 4) {
                if (rep_id & SP_ID_CDR_LE) {
                    if (offset == 0)
                        sp_len = data;
                    else if (offset == 1)
                        sp_len |= data << 8;
                    else if (offset == 2)
                        sp_len |= data << 16;
                    else
                        sp_len |= data << 24;
                } else {
                    if (offset == 0)
                        sp_len = data << 24;
                    else if (offset == 1)
                        sp_len |= data << 16;
                    else if (offset == 2)
                        sp_len |= data << 8;
                    else
                        sp_len |= data;
                }
            } else {
                if (ep_type & BUILTIN_EP_SUB) {
                    /* Cyber unroll_times=all */
                    for (auto j = 0; j < PUB_TOPICS_MAX; j++) {
#pragma HLS unroll
                        if ( // This topic is disabled,
                            !pub_enable[j]
                            // or the topic name length is different,
                            || (sp_len != pub_topic_name_len[j])
                            // or the topic name has a different letter
                            || ((offset < sp_len + 4)
                                && (pub_topic_name[j][offset - 4] != data))) {
                            pub_topics_unmatched
                                |= hls_uint<PUB_TOPICS_MAX>(1 << j);
                        }
                    }
                    if ((~pub_topics_unmatched & ~pub_types_unmatched) == 0) {
                        // Every topic has an unmatched topic name or an
                        // unmached type name.
                        flags |= hls_uint<5>(FLAGS_UNMATCH_TOPIC);
                    }
                } else {
                    /* Cyber unroll_times=all */
                    for (auto j = 0; j < SUB_TOPICS_MAX; j++) {
#pragma HLS unroll
                        if (!sub_enable[j] || (sp_len != sub_topic_name_len[j])
                            || ((offset < sp_len + 4)
                                && (sub_topic_name[j][offset - 4] != data))) {
                            sub_topics_unmatched
                                |= hls_uint<SUB_TOPICS_MAX>(1 << j);
                        }
                    }
                    if ((~sub_topics_unmatched & ~sub_types_unmatched) == 0) {
                        flags |= hls_uint<5>(FLAGS_UNMATCH_TOPIC);
                    }
                }
            }
            break;
        case PID_TYPE_NAME:
            if (offset < 4) {
                if (rep_id & SP_ID_CDR_LE) {
                    if (offset == 0)
                        sp_len = data;
                    else if (offset == 1)
                        sp_len |= data << 8;
                    else if (offset == 2)
                        sp_len |= data << 16;
                    else
                        sp_len |= data << 24;
                } else {
                    if (offset == 0)
                        sp_len = data << 24;
                    else if (offset == 1)
                        sp_len |= data << 16;
                    else if (offset == 2)
                        sp_len |= data << 8;
                    else
                        sp_len |= data;
                }
            } else {
                if (ep_type & BUILTIN_EP_SUB) {
                    /* Cyber unroll_times=all */
                    for (auto j = 0; j < PUB_TOPICS_MAX; j++) {
#pragma HLS unroll
                        if (!pub_enable[j] || (sp_len != pub_type_name_len[j])
                            || ((offset < sp_len + 4)
                                && (pub_type_name[j][offset - 4] != data))) {
                            pub_types_unmatched
                                |= hls_uint<PUB_TOPICS_MAX>(1 << j);
                        }
                    }
                    if ((~pub_topics_unmatched & ~pub_types_unmatched) == 0) {
                        flags |= hls_uint<5>(FLAGS_UNMATCH_TYPE);
                    }
                } else {
                    /* Cyber unroll_times=all */
                    for (auto j = 0; j < SUB_TOPICS_MAX; j++) {
#pragma HLS unroll
                        if (!sub_enable[j] || (sp_len != sub_type_name_len[j])
                            || ((offset < sp_len + 4)
                                && (sub_type_name[j][offset - 4] != data))) {
                            sub_types_unmatched
                                |= hls_uint<SUB_TOPICS_MAX>(1 << j);
                        }
                    }
                    if ((~sub_topics_unmatched & ~sub_types_unmatched) == 0) {
                        flags |= hls_uint<5>(FLAGS_UNMATCH_TYPE);
                    }
                }
            }
            break;
        case PID_ENDPOINT_GUID:
            if (flags & FLAGS_FOUND_GUID)
                break;
            if ((offset >= 12) && (offset < 16)) {
                sedp_entity_id[offset - 12] = data;
            }
        }
        offset++;
        if (offset == param_len) {
            if (param_id == PID_ENDPOINT_GUID) {
                flags |= (hls_uint<5>)FLAGS_FOUND_GUID;
            } else if (param_id == PID_UNICAST_LOCATOR) {
                uint16_t udp_port = (sedp_udp_port[0] << 8) | sedp_udp_port[1];
                if (udp_port >= port_num_seed
                    && udp_port - port_num_seed < DG) {
                    if (is_same_subnet(sedp_ip_addr, ip_addr, subnet_mask)) {
                        flags |= (hls_uint<5>)FLAGS_FOUND_LOCATOR;
                    }
                }
            }
            offset = 0;
            state = SEDP_READ_PARAM_ID;
        }
        break;
    case SEDP_READ_SKIP_SBM:
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = SEDP_READ_SBM_HDR;
        }
        break;
    default:; // do nothing
    }

    if (end) {
        flags = 0;
        pub_topics_unmatched = 0;
        pub_types_unmatched = 0;
        sub_topics_unmatched = 0;
        sub_types_unmatched = 0;
        offset = 0;
        state = SEDP_READ_HDR_PROTOCOL;
    }
}

/* Cyber func=inline */
void sedp_writer(
    const uint8_t writer_guid_prefix[12], const uint8_t writer_entity_id[4],
    const uint8_t reader_guid_prefix[12], const uint8_t reader_entity_id[4],
    int64_t seqnum, const uint8_t usertraffic_addr[4],
    const uint8_t usertraffic_port[2], const uint8_t app_entity_id[4],
    hls_stream<uint8_t> &out, const uint8_t topic_name[MAX_TOPIC_NAME_LEN],
    uint8_t topic_name_len, const uint8_t type_name[MAX_TOPIC_TYPE_NAME_LEN],
    uint8_t type_name_len, timestamp now) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t  sbm_flags = SBM_FLAGS_ENDIANNESS;
    static const uint16_t rep_id = SP_ID_PL_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t  sbm_flags = 0;
    static const uint16_t rep_id = SP_ID_PL_CDR_BE;
#endif // SBM_ENDIAN_BIG

    static const uint16_t ext_flags = 0;
    static const uint16_t rep_opt = 0;

    static const uint16_t octets_to_next_header
        = SEDP_WRITER_OCTETS_TO_NEXT_HEADER;

    static const uint8_t participant_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_PARTICIPANT;
#pragma HLS array_partition variable = participant_entity_id complete dim = 0

    uint16_t pid_topic_name_size = SP_STR_DATA_SIZE(topic_name_len);

    uint16_t pid_type_name_size = SP_STR_DATA_SIZE(type_name_len);

    static const uint32_t type_max_size_serialized = 4 + MAX_APP_DATA_LEN;
    static const uint32_t durability_qos = 0;
    static const duration deadline = DURATION_INFINITE;
    static const duration latency_budget = DURATION_ZERO;
    static const uint32_t liveliness_qos = 0;
    static const duration liveliness_duration = DURATION_INFINITE;
    static const uint32_t reliability_qos = 1;
    static const duration reliability_duration
        = {0, (uint64_t)((0.1 * (1ULL << 32)) + 0.5)}; // 100 ms
    static const duration lifespan = DURATION_INFINITE;
    static const uint32_t ownership = 0;
    static const uint32_t ownership_strength = 0;
    static const uint32_t destination_order = 0;

    int32_t  seqnum_h = seqnum >> 32;
    uint32_t seqnum_l = seqnum & 0xffffffff;

    out.write('R');
    out.write('T');
    out.write('P');
    out.write('S');
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(RTPS_HDR_VENDOR_ID >> 8);
    out.write(RTPS_HDR_VENDOR_ID & 0xff);
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(SBM_ID_INFO_DST);
    out.write(sbm_flags);
    out.write(S_BYTE0(GUID_PREFIX_SIZE));
    out.write(S_BYTE1(GUID_PREFIX_SIZE));
    out.write(reader_guid_prefix[0]);
    out.write(reader_guid_prefix[1]);
    out.write(reader_guid_prefix[2]);
    out.write(reader_guid_prefix[3]);
    out.write(reader_guid_prefix[4]);
    out.write(reader_guid_prefix[5]);
    out.write(reader_guid_prefix[6]);
    out.write(reader_guid_prefix[7]);
    out.write(reader_guid_prefix[8]);
    out.write(reader_guid_prefix[9]);
    out.write(reader_guid_prefix[10]);
    out.write(reader_guid_prefix[11]);
    out.write(SBM_ID_INFO_TS);
    out.write(sbm_flags);
    out.write(S_BYTE0(TIMESTAMP_SIZE));
    out.write(S_BYTE1(TIMESTAMP_SIZE));
    out.write(L_BYTE0(now.seconds));
    out.write(L_BYTE1(now.seconds));
    out.write(L_BYTE2(now.seconds));
    out.write(L_BYTE3(now.seconds));
    out.write(L_BYTE0(now.fraction));
    out.write(L_BYTE1(now.fraction));
    out.write(L_BYTE2(now.fraction));
    out.write(L_BYTE3(now.fraction));
    out.write(SBM_ID_DATA);
    out.write(sbm_flags | SBM_FLAGS_DATA);
    out.write(S_BYTE0(octets_to_next_header));
    out.write(S_BYTE1(octets_to_next_header));
    out.write(ext_flags >> 8);
    out.write(ext_flags & 0xff);
    out.write(S_BYTE0(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS));
    out.write(S_BYTE1(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS));
    out.write(reader_entity_id[0]);
    out.write(reader_entity_id[1]);
    out.write(reader_entity_id[2]);
    out.write(reader_entity_id[3]);
    out.write(writer_entity_id[0]);
    out.write(writer_entity_id[1]);
    out.write(writer_entity_id[2]);
    out.write(writer_entity_id[3]);
    out.write(L_BYTE0(seqnum_h));
    out.write(L_BYTE1(seqnum_h));
    out.write(L_BYTE2(seqnum_h));
    out.write(L_BYTE3(seqnum_h));
    out.write(L_BYTE0(seqnum_l));
    out.write(L_BYTE1(seqnum_l));
    out.write(L_BYTE2(seqnum_l));
    out.write(L_BYTE3(seqnum_l));
    out.write(rep_id >> 8);
    out.write(rep_id & 0xff);
    out.write(rep_opt >> 8);
    out.write(rep_opt & 0xff);
    out.write(S_BYTE0(PID_UNICAST_LOCATOR));
    out.write(S_BYTE1(PID_UNICAST_LOCATOR));
    out.write(S_BYTE0(PID_UNICAST_LOCATOR_SIZE));
    out.write(S_BYTE1(PID_UNICAST_LOCATOR_SIZE));
    out.write(L_BYTE0(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE1(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE2(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE3(LOCATOR_KIND_UDPv4));
#ifdef SBM_ENDIAN_LITTLE
    out.write(usertraffic_port[1]);
    out.write(usertraffic_port[0]);
    out.write(0);
    out.write(0);
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    out.write(0);
    out.write(0);
    out.write(usertraffic_port[0]);
    out.write(usertraffic_port[1]);
#endif // SBM_ENDIAN_BIG
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
        out.write(0);
    }
    out.write(usertraffic_addr[0]);
    out.write(usertraffic_addr[1]);
    out.write(usertraffic_addr[2]);
    out.write(usertraffic_addr[3]);
    out.write(S_BYTE0(PID_PARTICIPANT_GUID));
    out.write(S_BYTE1(PID_PARTICIPANT_GUID));
    out.write(S_BYTE0(PID_PARTICIPANT_GUID_SIZE));
    out.write(S_BYTE1(PID_PARTICIPANT_GUID_SIZE));
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(participant_entity_id[0]);
    out.write(participant_entity_id[1]);
    out.write(participant_entity_id[2]);
    out.write(participant_entity_id[3]);
    out.write(S_BYTE0(PID_TOPIC_NAME));
    out.write(S_BYTE1(PID_TOPIC_NAME));
    out.write(S_BYTE0(pid_topic_name_size));
    out.write(S_BYTE1(pid_topic_name_size));
    out.write(L_BYTE0(topic_name_len));
    out.write(L_BYTE1(topic_name_len));
    out.write(L_BYTE2(topic_name_len));
    out.write(L_BYTE3(topic_name_len));
    /* Cyber unroll_times=all */
    for (auto j = 0; j < MAX_TOPIC_NAME_LEN; j++) {
#pragma HLS unroll
        out.write(topic_name[j]);
    }
    out.write(S_BYTE0(PID_TYPE_NAME));
    out.write(S_BYTE1(PID_TYPE_NAME));
    out.write(S_BYTE0(pid_type_name_size));
    out.write(S_BYTE1(pid_type_name_size));
    out.write(L_BYTE0(type_name_len));
    out.write(L_BYTE1(type_name_len));
    out.write(L_BYTE2(type_name_len));
    out.write(L_BYTE3(type_name_len));
    /* Cyber unroll_times=all */
    for (auto j = 0; j < MAX_TOPIC_TYPE_NAME_LEN; j++) {
#pragma HLS unroll
        out.write(type_name[j]);
    }
    out.write(S_BYTE0(PID_KEY_HASH));
    out.write(S_BYTE1(PID_KEY_HASH));
    out.write(S_BYTE0(PID_KEY_HASH_SIZE));
    out.write(S_BYTE1(PID_KEY_HASH_SIZE));
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(app_entity_id[0]);
    out.write(app_entity_id[1]);
    out.write(app_entity_id[2]);
    out.write(app_entity_id[3]);
    out.write(S_BYTE0(PID_ENDPOINT_GUID));
    out.write(S_BYTE1(PID_ENDPOINT_GUID));
    out.write(S_BYTE0(PID_ENDPOINT_GUID_SIZE));
    out.write(S_BYTE1(PID_ENDPOINT_GUID_SIZE));
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(app_entity_id[0]);
    out.write(app_entity_id[1]);
    out.write(app_entity_id[2]);
    out.write(app_entity_id[3]);
    out.write(S_BYTE0(PID_TYPE_MAX_SIZE_SERIALIZED));
    out.write(S_BYTE1(PID_TYPE_MAX_SIZE_SERIALIZED));
    out.write(S_BYTE0(PID_TYPE_MAX_SIZE_SERIALIZED_SIZE));
    out.write(S_BYTE1(PID_TYPE_MAX_SIZE_SERIALIZED_SIZE));
    out.write(L_BYTE0(type_max_size_serialized));
    out.write(L_BYTE1(type_max_size_serialized));
    out.write(L_BYTE2(type_max_size_serialized));
    out.write(L_BYTE3(type_max_size_serialized));
    out.write(S_BYTE0(PID_PROTOCOL_VERSION));
    out.write(S_BYTE1(PID_PROTOCOL_VERSION));
    out.write(S_BYTE0(PID_PROTOCOL_VERSION_SIZE));
    out.write(S_BYTE1(PID_PROTOCOL_VERSION_SIZE));
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(0); // padding
    out.write(0); // padding
    out.write(S_BYTE0(PID_VENDOR_ID));
    out.write(S_BYTE1(PID_VENDOR_ID));
    out.write(S_BYTE0(PID_VENDOR_ID_SIZE));
    out.write(S_BYTE1(PID_VENDOR_ID_SIZE));
    out.write(RTPS_HDR_VENDOR_ID >> 8);
    out.write(RTPS_HDR_VENDOR_ID & 0xff);
    out.write(0); // padding
    out.write(0); // padding
    out.write(S_BYTE0(PID_DURABILITY));
    out.write(S_BYTE1(PID_DURABILITY));
    out.write(S_BYTE0(PID_DURABILITY_SIZE));
    out.write(S_BYTE1(PID_DURABILITY_SIZE));
    out.write(L_BYTE0(durability_qos));
    out.write(L_BYTE1(durability_qos));
    out.write(L_BYTE2(durability_qos));
    out.write(L_BYTE3(durability_qos));
    out.write(S_BYTE0(PID_DEADLINE));
    out.write(S_BYTE1(PID_DEADLINE));
    out.write(S_BYTE0(PID_DEADLINE_SIZE));
    out.write(S_BYTE1(PID_DEADLINE_SIZE));
    out.write(L_BYTE0(deadline.seconds));
    out.write(L_BYTE1(deadline.seconds));
    out.write(L_BYTE2(deadline.seconds));
    out.write(L_BYTE3(deadline.seconds));
    out.write(L_BYTE0(deadline.fraction));
    out.write(L_BYTE1(deadline.fraction));
    out.write(L_BYTE2(deadline.fraction));
    out.write(L_BYTE3(deadline.fraction));
    out.write(S_BYTE0(PID_LATENCY_BUDGET));
    out.write(S_BYTE1(PID_LATENCY_BUDGET));
    out.write(S_BYTE0(PID_LATENCY_BUDGET_SIZE));
    out.write(S_BYTE1(PID_LATENCY_BUDGET_SIZE));
    out.write(L_BYTE0(latency_budget.seconds));
    out.write(L_BYTE1(latency_budget.seconds));
    out.write(L_BYTE2(latency_budget.seconds));
    out.write(L_BYTE3(latency_budget.seconds));
    out.write(L_BYTE0(latency_budget.fraction));
    out.write(L_BYTE1(latency_budget.fraction));
    out.write(L_BYTE2(latency_budget.fraction));
    out.write(L_BYTE3(latency_budget.fraction));
    out.write(S_BYTE0(PID_LIVELINESS));
    out.write(S_BYTE1(PID_LIVELINESS));
    out.write(S_BYTE0(PID_LIVELINESS_SIZE));
    out.write(S_BYTE1(PID_LIVELINESS_SIZE));
    out.write(L_BYTE0(liveliness_qos));
    out.write(L_BYTE1(liveliness_qos));
    out.write(L_BYTE2(liveliness_qos));
    out.write(L_BYTE3(liveliness_qos));
    out.write(L_BYTE0(liveliness_duration.seconds));
    out.write(L_BYTE1(liveliness_duration.seconds));
    out.write(L_BYTE2(liveliness_duration.seconds));
    out.write(L_BYTE3(liveliness_duration.seconds));
    out.write(L_BYTE0(liveliness_duration.fraction));
    out.write(L_BYTE1(liveliness_duration.fraction));
    out.write(L_BYTE2(liveliness_duration.fraction));
    out.write(L_BYTE3(liveliness_duration.fraction));
    out.write(S_BYTE0(PID_RELIABILITY));
    out.write(S_BYTE1(PID_RELIABILITY));
    out.write(S_BYTE0(PID_RELIABILITY_SIZE));
    out.write(S_BYTE1(PID_RELIABILITY_SIZE));
    out.write(L_BYTE0(reliability_qos));
    out.write(L_BYTE1(reliability_qos));
    out.write(L_BYTE2(reliability_qos));
    out.write(L_BYTE3(reliability_qos));
    out.write(L_BYTE0(reliability_duration.seconds));
    out.write(L_BYTE1(reliability_duration.seconds));
    out.write(L_BYTE2(reliability_duration.seconds));
    out.write(L_BYTE3(reliability_duration.seconds));
    out.write(L_BYTE0(reliability_duration.fraction));
    out.write(L_BYTE1(reliability_duration.fraction));
    out.write(L_BYTE2(reliability_duration.fraction));
    out.write(L_BYTE3(reliability_duration.fraction));
    out.write(S_BYTE0(PID_LIFESPAN));
    out.write(S_BYTE1(PID_LIFESPAN));
    out.write(S_BYTE0(PID_LIFESPAN_SIZE));
    out.write(S_BYTE1(PID_LIFESPAN_SIZE));
    out.write(L_BYTE0(lifespan.seconds));
    out.write(L_BYTE1(lifespan.seconds));
    out.write(L_BYTE2(lifespan.seconds));
    out.write(L_BYTE3(lifespan.seconds));
    out.write(L_BYTE0(lifespan.fraction));
    out.write(L_BYTE1(lifespan.fraction));
    out.write(L_BYTE2(lifespan.fraction));
    out.write(L_BYTE3(lifespan.fraction));
    out.write(S_BYTE0(PID_OWNERSHIP));
    out.write(S_BYTE1(PID_OWNERSHIP));
    out.write(S_BYTE0(PID_OWNERSHIP_SIZE));
    out.write(S_BYTE1(PID_OWNERSHIP_SIZE));
    out.write(L_BYTE0(ownership));
    out.write(L_BYTE1(ownership));
    out.write(L_BYTE2(ownership));
    out.write(L_BYTE3(ownership));
    out.write(S_BYTE0(PID_OWNERSHIP_STRENGTH));
    out.write(S_BYTE1(PID_OWNERSHIP_STRENGTH));
    out.write(S_BYTE0(PID_OWNERSHIP_STRENGTH_SIZE));
    out.write(S_BYTE1(PID_OWNERSHIP_STRENGTH_SIZE));
    out.write(L_BYTE0(ownership_strength));
    out.write(L_BYTE1(ownership_strength));
    out.write(L_BYTE2(ownership_strength));
    out.write(L_BYTE3(ownership_strength));
    out.write(S_BYTE0(PID_DESTINATION_ORDER));
    out.write(S_BYTE1(PID_DESTINATION_ORDER));
    out.write(S_BYTE0(PID_DESTINATION_ORDER_SIZE));
    out.write(S_BYTE1(PID_DESTINATION_ORDER_SIZE));
    out.write(L_BYTE0(destination_order));
    out.write(L_BYTE1(destination_order));
    out.write(L_BYTE2(destination_order));
    out.write(L_BYTE3(destination_order));
    out.write(S_BYTE0(PID_SENTINEL));
    out.write(S_BYTE1(PID_SENTINEL));
    out.write(0); // PID_SENTINEL_SIZE
    out.write(0); // PID_SENTINEL_SIZE
}

/* Cyber func=inline */
void sedp_heartbeat(const uint8_t writer_guid_prefix[12],
                    const uint8_t writer_entity_id[4],
                    const uint8_t reader_guid_prefix[12],
                    const uint8_t reader_entity_id[4],
                    const int64_t first_seqnum, const int64_t last_seqnum,
                    const uint32_t cnt, hls_stream<uint8_t> &out) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t sbm_flags = 0;
#endif // SBM_ENDIAN_BIG

    int32_t  first_seqnum_h = first_seqnum >> 32;
    uint32_t first_seqnum_l = first_seqnum & 0xffffffff;
    int32_t  last_seqnum_h = last_seqnum >> 32;
    uint32_t last_seqnum_l = last_seqnum & 0xffffffff;

    out.write('R');
    out.write('T');
    out.write('P');
    out.write('S');
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(RTPS_HDR_VENDOR_ID >> 8);
    out.write(RTPS_HDR_VENDOR_ID & 0xff);
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(SBM_ID_INFO_DST);
    out.write(sbm_flags);
    out.write(S_BYTE0(GUID_PREFIX_SIZE));
    out.write(S_BYTE1(GUID_PREFIX_SIZE));
    out.write(reader_guid_prefix[0]);
    out.write(reader_guid_prefix[1]);
    out.write(reader_guid_prefix[2]);
    out.write(reader_guid_prefix[3]);
    out.write(reader_guid_prefix[4]);
    out.write(reader_guid_prefix[5]);
    out.write(reader_guid_prefix[6]);
    out.write(reader_guid_prefix[7]);
    out.write(reader_guid_prefix[8]);
    out.write(reader_guid_prefix[9]);
    out.write(reader_guid_prefix[10]);
    out.write(reader_guid_prefix[11]);
    out.write(SBM_ID_HEARTBEAT);
    out.write(sbm_flags | SBM_FLAGS_FINAL);
    out.write(S_BYTE0(SBM_HEARTBEAT_DATA_SIZE));
    out.write(S_BYTE1(SBM_HEARTBEAT_DATA_SIZE));
    out.write(reader_entity_id[0]);
    out.write(reader_entity_id[1]);
    out.write(reader_entity_id[2]);
    out.write(reader_entity_id[3]);
    out.write(writer_entity_id[0]);
    out.write(writer_entity_id[1]);
    out.write(writer_entity_id[2]);
    out.write(writer_entity_id[3]);
    out.write(L_BYTE0(first_seqnum_h));
    out.write(L_BYTE1(first_seqnum_h));
    out.write(L_BYTE2(first_seqnum_h));
    out.write(L_BYTE3(first_seqnum_h));
    out.write(L_BYTE0(first_seqnum_l));
    out.write(L_BYTE1(first_seqnum_l));
    out.write(L_BYTE2(first_seqnum_l));
    out.write(L_BYTE3(first_seqnum_l));
    out.write(L_BYTE0(last_seqnum_h));
    out.write(L_BYTE1(last_seqnum_h));
    out.write(L_BYTE2(last_seqnum_h));
    out.write(L_BYTE3(last_seqnum_h));
    out.write(L_BYTE0(last_seqnum_l));
    out.write(L_BYTE1(last_seqnum_l));
    out.write(L_BYTE2(last_seqnum_l));
    out.write(L_BYTE3(last_seqnum_l));
    out.write(L_BYTE0(cnt));
    out.write(L_BYTE1(cnt));
    out.write(L_BYTE2(cnt));
    out.write(L_BYTE3(cnt));
}

/* Cyber func=inline */
void sedp_acknack(const uint8_t writer_guid_prefix[12],
                  const uint8_t writer_entity_id[4],
                  const uint8_t reader_guid_prefix[12],
                  const uint8_t reader_entity_id[4], uint8_t snstate_base,
                  bool snstate_is_empty, const uint32_t cnt,
                  hls_stream<uint8_t> &out) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t sbm_flags = 0;
#endif // SBM_ENDIAN_BIG

    out.write('R');
    out.write('T');
    out.write('P');
    out.write('S');
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(RTPS_HDR_VENDOR_ID >> 8);
    out.write(RTPS_HDR_VENDOR_ID & 0xff);
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(SBM_ID_INFO_DST);
    out.write(sbm_flags);
    out.write(S_BYTE0(GUID_PREFIX_SIZE));
    out.write(S_BYTE1(GUID_PREFIX_SIZE));
    out.write(reader_guid_prefix[0]);
    out.write(reader_guid_prefix[1]);
    out.write(reader_guid_prefix[2]);
    out.write(reader_guid_prefix[3]);
    out.write(reader_guid_prefix[4]);
    out.write(reader_guid_prefix[5]);
    out.write(reader_guid_prefix[6]);
    out.write(reader_guid_prefix[7]);
    out.write(reader_guid_prefix[8]);
    out.write(reader_guid_prefix[9]);
    out.write(reader_guid_prefix[10]);
    out.write(reader_guid_prefix[11]);
    out.write(SBM_ID_ACKNACK);
    out.write(sbm_flags | SBM_FLAGS_FINAL);
    out.write(S_BYTE0(SBM_ACKNACK_DATA_SIZE));
    out.write(S_BYTE1(SBM_ACKNACK_DATA_SIZE));
    out.write(reader_entity_id[0]);
    out.write(reader_entity_id[1]);
    out.write(reader_entity_id[2]);
    out.write(reader_entity_id[3]);
    out.write(writer_entity_id[0]);
    out.write(writer_entity_id[1]);
    out.write(writer_entity_id[2]);
    out.write(writer_entity_id[3]);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(snstate_base);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(1);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(
        snstate_is_empty
            ? 0x00
            : 0x80); // Reporting missing seqnum by ACKNACK is done one by one.
    out.write(L_BYTE0(cnt));
    out.write(L_BYTE1(cnt));
    out.write(L_BYTE2(cnt));
    out.write(L_BYTE3(cnt));
}
