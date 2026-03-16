#include "discovery_protocol_in.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

#define FLAG_LOCATOR_FOUND        0b00001
#define FLAG_LEASE_DURATION_FOUND 0b00010
#define FLAG_ENTITY_ID_FOUND      0b00100
#define FLAG_TOPIC_MATCHED        0b01000
#define FLAG_TYPE_MATCHED         0b10000

typedef hls_uint<5> flag_t;

static void set_rtps_data_common(rtps_data_t  *rtps_data,
                                 const uint8_t guid_prefix[GUID_PREFIX_SIZE],
                                 const uint8_t ip_addr[4],
                                 const uint8_t udp_port[2]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
        rtps_data->guid_prefix[j] = guid_prefix[j];
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        rtps_data->data[j] = ip_addr[j];
    }
    rtps_data->data[4] = udp_port[0];
    rtps_data->data[5] = udp_port[1];
}

static void set_rtps_data_spdp(rtps_data_t *rtps_data, flag_t flags,
                               const uint8_t lease_duration[8]) {
#pragma HLS inline
    rtps_data->type = RTPS_TYPE_SPDP;
    if (flags & FLAG_LEASE_DURATION_FOUND) {
        /* Cyber unroll_times=all */
        for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
            rtps_data->data[j + 6] = lease_duration[j];
        }
    } else {
        // Use default value 100 sec.
        rtps_data->data[6] = 0;
        rtps_data->data[7] = 0;
        rtps_data->data[8] = 0;
        rtps_data->data[9] = 0;
        rtps_data->data[10] = 100;
        rtps_data->data[11] = 0;
        rtps_data->data[12] = 0;
        rtps_data->data[13] = 0;
    }
}

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

static void set_rtps_data_sedp(rtps_data_t      *rtps_data,
                               builtin_ep_type_t ep_type, flag_t flags,
                               hls_uint<PUB_TOPICS_MAX> pub_topics_unmatched,
                               hls_uint<PUB_TOPICS_MAX> pub_types_unmatched,
                               uint8_t seqnum, const uint8_t entity_id[4]) {
#pragma HLS inline
    static const flag_t found = FLAG_LOCATOR_FOUND | FLAG_ENTITY_ID_FOUND
                                | FLAG_TOPIC_MATCHED | FLAG_TYPE_MATCHED;

    /* Cyber unroll_times=all */
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        rtps_data->data[j + 6] = entity_id[j];
    }
    rtps_data->data[11] = seqnum;

    if ((flags & found) == found) {
        if (ep_type & BUILTIN_EP_SUB) {
            hls_uint<PUB_TOPICS_MAX> pub_matched
                = ~pub_topics_unmatched & ~pub_types_unmatched;
            rtps_data->data[10] = get_matched_pub_topic_id(pub_matched);
            rtps_data->type = RTPS_TYPE_SEDP_SUB;
        } else {
            rtps_data->type = RTPS_TYPE_SEDP_PUB;
        }
    } else {
        if (ep_type & BUILTIN_EP_SUB) {
            rtps_data->type = RTPS_TYPE_SEDP_SUB_SN_ONLY;
        } else {
            rtps_data->type = RTPS_TYPE_SEDP_PUB_SN_ONLY;
        }
    }
}

static bool is_locator_param(uint16_t param_id, bool is_spdp,
                             builtin_ep_type_t ep_type) {
    return (is_spdp && (param_id == PID_METATRAFFIC_UNICAST_LOCATOR))
           || ((ep_type != 0) && (param_id == PID_UNICAST_LOCATOR));
}

static bool is_valid_udp_port(const uint8_t udp_port[2],
                              uint16_t      port_num_seed) {
    uint16_t port_num = (udp_port[0] << 8) | udp_port[1];
    return (port_num >= port_num_seed) && (port_num < (port_num_seed + DG));
}

static bool is_topic_info_param(uint16_t param_id, flag_t flags) {
    return ((param_id == PID_TOPIC_NAME) && ((flags & FLAG_TOPIC_MATCHED) == 0))
           || ((param_id == PID_TYPE_NAME)
               && ((flags & FLAG_TYPE_MATCHED) == 0));
}

template <unsigned int TOPICS_MAX>
static hls_uint<TOPICS_MAX>
compare_topic_info_len(uint16_t input_len, const uint8_t name_len[TOPICS_MAX]) {
#pragma HLS inline
    hls_uint<TOPICS_MAX> unmatched = 0;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < TOPICS_MAX; j++) {
#pragma HLS unroll
        if (input_len != name_len[j]) {
            unmatched |= (1 << j);
        }
    }
    return unmatched;
}

template <unsigned int TOPICS_MAX, unsigned int MAX_NAME_LEN>
static hls_uint<TOPICS_MAX>
compare_topic_info(uint8_t data, uint16_t offset,
                   const uint8_t name[TOPICS_MAX][MAX_NAME_LEN]) {
#pragma HLS inline
    hls_uint<TOPICS_MAX> unmatched = 0;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < TOPICS_MAX; j++) {
#pragma HLS unroll
        if ((offset < MAX_NAME_LEN) && (data != name[j][offset])) {
            unmatched |= (1 << j);
        }
    }
    return unmatched;
}

#define STATE_READ_SBM_DATA_HDR 0
#define STATE_READ_SP_HDR       1
#define STATE_READ_PARAM_HDR    2
#define STATE_READ_PARAM        3
#define STATE_SKIP              4

void discovery_protocol_in(
    hls_uint<10> x, hls_stream<rtps_data_t> &out, uint8_t sbm_flags,
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    const uint8_t reader_ip_addr[4], const uint8_t subnet_mask[4],
    uint16_t      port_num_seed,
    const uint8_t pub_topic_name[PUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t pub_topic_name_len[PUB_TOPICS_MAX],
    const uint8_t pub_type_name[PUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t pub_type_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_topic_name[SUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t sub_topic_name_len[SUB_TOPICS_MAX],
    const uint8_t sub_type_name[SUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t sub_type_name_len[SUB_TOPICS_MAX],
    const uint8_t guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    static const uint8_t par_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PARTICIPANT_READER;
#pragma HLS array_partition variable = par_reader_id complete dim = 0
    static const uint8_t pub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
#pragma HLS array_partition variable = pub_reader_id complete dim = 0
    static const uint8_t sub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = sub_reader_id complete dim = 0

    static hls_uint<3> state = STATE_READ_SBM_DATA_HDR;
    static uint16_t    offset = 0;
    static uint8_t     seqnum;
    static uint16_t    rep_id;
    static uint16_t    param_id;
    static uint16_t    param_len;
    static uint32_t    name_len;

    static bool              is_spdp = true;
    static builtin_ep_type_t ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    static flag_t            flags;

    static hls_uint<PUB_TOPICS_MAX> pub_topics_unmatched;
    static hls_uint<PUB_TOPICS_MAX> pub_types_unmatched;
    static hls_uint<SUB_TOPICS_MAX> sub_topics_unmatched;
    static hls_uint<SUB_TOPICS_MAX> sub_types_unmatched;

    static uint8_t ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = ip_addr complete dim = 1
    static uint8_t udp_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = udp_port complete dim = 1
    static uint8_t participant_lease_duration
        [8] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = participant_lease_duration complete dim \
    = 1
    static uint8_t entity_id[4] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = entity_id complete dim = 1

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;
    bool    inline_qos = x & 0x200;
    bool    sbm_le = sbm_flags & SBM_FLAGS_ENDIANNESS;
    bool    param_le = rep_id & SP_ID_CDR_LE;

    if (!inline_qos) {
        switch (state) {
        case STATE_READ_SBM_DATA_HDR: // parse sub-message header
            if (!rtps_compare_data_hdr_reader_id(offset, data, par_reader_id)) {
                is_spdp = false;
            }
            if (!rtps_compare_data_hdr_reader_id(offset, data, pub_reader_id)) {
                ep_type &= ~(BUILTIN_EP_PUB);
            }
            if (!rtps_compare_data_hdr_reader_id(offset, data, sub_reader_id)) {
                ep_type &= ~(BUILTIN_EP_SUB);
            }

            // Ignore other than lower 8-bit of Sequence Number to reduce
            // resources
            if (sbm_le && (offset == (SBM_DATA_HDR_OFFSET_WRITER_SN + 4))) {
                seqnum = data;
            } else if (!sbm_le
                       && (offset == (SBM_DATA_HDR_OFFSET_WRITER_SN + 7))) {
                seqnum = data;
            }

            offset++;
            if (offset == SBM_DATA_HDR_SIZE) {
                offset = 0;
                flags = 0;
                pub_topics_unmatched = ~pub_enable;
                sub_topics_unmatched = ~sub_enable;
                pub_types_unmatched = ~pub_enable;
                sub_types_unmatched = ~sub_enable;
                if (!is_spdp && (ep_type == 0)) {
                    state = STATE_SKIP;
                } else {
                    state = STATE_READ_SP_HDR;
                }
            }
            break;
        case STATE_READ_SP_HDR:
            switch (offset) {
            case SP_HDR_OFFSET_REPRESENTATION_ID:
                rep_id = data << 8;
                break;
            case SP_HDR_OFFSET_REPRESENTATION_ID + 1:
                rep_id |= data;
                break;
            }
            offset++;
            if (offset == SP_HDR_SIZE) {
                offset = 0;
                state = STATE_READ_PARAM_HDR;
            }
            break;
        case STATE_READ_PARAM_HDR:
            switch (offset) {
            case 0:
                param_id = param_le ? data : (data << 8);
                break;
            case 1:
                param_id |= param_le ? (data << 8) : data;
                break;
            case 2:
                param_len = param_le ? data : (data << 8);
                break;
            case 3:
                param_len |= param_le ? (data << 8) : data;
                break;
            }
            offset++;
            if (offset == 4) {
                offset = 0;
                if (param_id == PID_SENTINEL) {
                    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
                    set_rtps_data_common(&rtps_data, guid_prefix, ip_addr,
                                         udp_port);
                    if (is_spdp) {
                        set_rtps_data_spdp(&rtps_data, flags,
                                           participant_lease_duration);
                        if (flags & FLAG_LOCATOR_FOUND) {
                            out.write(rtps_data);
                        }
                    } else if (ep_type != 0) {
                        set_rtps_data_sedp(
                            &rtps_data, ep_type, flags, pub_topics_unmatched,
                            pub_types_unmatched, seqnum, entity_id);
                        out.write(rtps_data);
                    }
                    state = STATE_SKIP;
                } else if (param_len == 0) {
                    state = STATE_READ_PARAM_HDR;
                } else {
                    state = STATE_READ_PARAM;
                }
            }
            break;
        case STATE_READ_PARAM:
            if (is_locator_param(param_id, is_spdp, ep_type)
                && ((flags & FLAG_LOCATOR_FOUND) == 0)) {
                switch (offset) {
                case 4:
                    if (param_le) {
                        udp_port[1] = data;
                    }
                    break;
                case 5:
                    if (param_le) {
                        udp_port[0] = data;
                    }
                    break;
                case 6:
                    if (!param_le) {
                        udp_port[0] = data;
                    }
                    break;
                case 7:
                    if (!param_le) {
                        udp_port[1] = data;
                    }
                    break;
                case 20:
                    ip_addr[0] = data;
                    break;
                case 21:
                    ip_addr[1] = data;
                    break;
                case 22:
                    ip_addr[2] = data;
                    break;
                case 23:
                    ip_addr[3] = data;
                    if (is_same_subnet(ip_addr, reader_ip_addr, subnet_mask)
                        && is_valid_udp_port(udp_port, port_num_seed)) {
                        flags |= FLAG_LOCATOR_FOUND;
                    }
                    break;
                }
            } else if (param_id == PID_PARTICIPANT_LEASE_DURATION) {
                if (offset < 8) {
                    if (param_le) {
                        if (offset < 4) {
                            // Read the seconds of the lease duration.
                            participant_lease_duration[offset + 4] = data;
                        } else {
                            // Read the fractional part of the lease duration.
                            participant_lease_duration[offset - 4] = data;
                        }
                    } else {
                        participant_lease_duration[7 - offset] = data;
                    }
                }
                if (offset == 7) {
                    flags |= FLAG_LEASE_DURATION_FOUND;
                }
            } else if ((param_id == PID_ENDPOINT_GUID)
                       && ((flags & FLAG_ENTITY_ID_FOUND) == 0)) {
                if ((offset >= 12) && (offset < 16)) {
                    entity_id[offset - 12] = data;
                }
                if (offset == 15) {
                    flags |= FLAG_ENTITY_ID_FOUND;
                }
            } else if ((ep_type != 0) && is_topic_info_param(param_id, flags)) {
                if (offset == 0) {
                    name_len = param_le ? data : (data << 24);
                } else if (offset == 1) {
                    name_len |= param_le ? (data << 8) : (data << 16);
                } else if (offset == 2) {
                    name_len |= param_le ? (data << 16) : (data << 8);
                } else if (offset == 3) {
                    name_len |= param_le ? (data << 16) : (data << 8);
                    if (param_id == PID_TOPIC_NAME) {
                        pub_topics_unmatched
                            |= compare_topic_info_len<PUB_TOPICS_MAX>(
                                name_len, pub_topic_name_len);
                        sub_topics_unmatched
                            |= compare_topic_info_len<SUB_TOPICS_MAX>(
                                name_len, sub_topic_name_len);
                    } else if (param_id == PID_TYPE_NAME) {
                        pub_types_unmatched
                            |= compare_topic_info_len<PUB_TOPICS_MAX>(
                                name_len, pub_type_name_len);
                        sub_types_unmatched
                            |= compare_topic_info_len<SUB_TOPICS_MAX>(
                                name_len, sub_type_name_len);
                    }
                } else if (offset < (name_len + 4)) {
                    if (param_id == PID_TOPIC_NAME) {
                        pub_topics_unmatched
                            |= compare_topic_info<PUB_TOPICS_MAX,
                                                  MAX_TOPIC_NAME_LEN>(
                                data, offset - 4, pub_topic_name);
                        sub_topics_unmatched
                            |= compare_topic_info<SUB_TOPICS_MAX,
                                                  MAX_TOPIC_NAME_LEN>(
                                data, offset - 4, sub_topic_name);
                    } else if (param_id == PID_TYPE_NAME) {
                        pub_types_unmatched
                            |= compare_topic_info<PUB_TOPICS_MAX,
                                                  MAX_TOPIC_TYPE_NAME_LEN>(
                                data, offset - 4, pub_type_name);
                        sub_types_unmatched
                            |= compare_topic_info<SUB_TOPICS_MAX,
                                                  MAX_TOPIC_TYPE_NAME_LEN>(
                                data, offset - 4, sub_type_name);
                    }
                }
            }
            offset++;
            if (offset == param_len) {
                if (is_topic_info_param(param_id, flags) && (offset >= 4)
                    && (offset >= name_len)) {
                    hls_uint<PUB_TOPICS_MAX> pub_matched
                        = ~pub_topics_unmatched & ~pub_types_unmatched;
                    hls_uint<SUB_TOPICS_MAX> sub_matched
                        = ~sub_topics_unmatched & ~sub_types_unmatched;
                    if (((ep_type & BUILTIN_EP_SUB) && (pub_matched != 0))
                        || ((ep_type & BUILTIN_EP_PUB) && (sub_matched != 0))) {
                        if (param_id == PID_TOPIC_NAME) {
                            flags |= FLAG_TOPIC_MATCHED;
                        } else if (param_id == PID_TYPE_NAME) {
                            flags |= FLAG_TYPE_MATCHED;
                        }
                    }
                }
                offset = 0;
                state = STATE_READ_PARAM_HDR;
            }
            break;
        }
    }

    if (end) {
        state = STATE_READ_SBM_DATA_HDR;
        offset = 0;
        is_spdp = true;
        ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    }
}
