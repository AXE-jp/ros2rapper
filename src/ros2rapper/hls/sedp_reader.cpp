#include "sedp_reader.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "spdp_reader.hpp"
#include <cstdint>

static bool sedp_set_guid(uint16_t offset, uint8_t data,
                          uint8_t sedp_entity_id[4]) {
#pragma HLS inline
    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
        if (offset == (j + GUID_PREFIX_SIZE)) {
            sedp_entity_id[j] = data;
        }
    }
    return (offset >= (GUID_PREFIX_SIZE + 3));
}

template <unsigned int TOPICS_MAX>
static void sedp_compare_topic_name_len(hls_uint<TOPICS_MAX> *unmatched,
                                        uint32_t              name_length,
                                        const uint8_t name_len[TOPICS_MAX]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < TOPICS_MAX; j++) {
#pragma HLS unroll
        if (name_length != name_len[j]) {
            (*unmatched)[j] = 1;
        }
    }
}

template <unsigned int TOPICS_MAX, unsigned int MAX_NAME_LEN>
static void
sedp_compare_topic_name(hls_uint<TOPICS_MAX> *unmatched, uint16_t offset,
                        uint8_t       data,
                        const uint8_t name[TOPICS_MAX][MAX_NAME_LEN]) {
#pragma HLS inline
    if (offset < MAX_NAME_LEN) {
        for (auto j = 0; j < TOPICS_MAX; j++) {
#pragma HLS unroll
            if (data != name[j][offset]) {
                (*unmatched)[j] = 1;
            }
        }
    }
}

template <unsigned int MAX_NAME_LEN>
static bool sedp_compare_topic_info(
    uint16_t offset, uint8_t data, hls_uint<PUB_TOPICS_MAX> *pub_unmatched,
    hls_uint<SUB_TOPICS_MAX> *sub_unmatched, bool param_le,
    uint32_t *name_length, const uint8_t pub_name[PUB_TOPICS_MAX][MAX_NAME_LEN],
    const uint8_t pub_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_name[SUB_TOPICS_MAX][MAX_NAME_LEN],
    const uint8_t sub_name_len[SUB_TOPICS_MAX]) {
#pragma HLS inline
    if (offset == 0) {
        *name_length = param_le ? data : (data << 24);
    } else if (offset == 1) {
        *name_length |= param_le ? (data << 8) : (data << 16);
    } else if (offset == 2) {
        *name_length |= param_le ? (data << 16) : (data << 8);
    } else if (offset == 3) {
        *name_length |= param_le ? (data << 24) : data;
        sedp_compare_topic_name_len<PUB_TOPICS_MAX>(pub_unmatched, *name_length,
                                                    pub_name_len);
        sedp_compare_topic_name_len<SUB_TOPICS_MAX>(sub_unmatched, *name_length,
                                                    sub_name_len);
    } else if (offset < (*name_length + 4)) {
        sedp_compare_topic_name<PUB_TOPICS_MAX, MAX_NAME_LEN>(
            pub_unmatched, offset - 4, data, pub_name);
        sedp_compare_topic_name<SUB_TOPICS_MAX, MAX_NAME_LEN>(
            sub_unmatched, offset - 4, data, sub_name);
        if (offset == (*name_length + 3)) {
            return true;
        }
    }
    return false;
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

static void send_received_sedp(hls_stream<rtps_data_t> &out,
                               builtin_ep_type_t ep_type, uint8_t seqnum,
                               bool found, hls_uint<PUB_TOPICS_MAX> pub_matched,
                               hls_uint<SUB_TOPICS_MAX> sub_matched,
                               const uint8_t src_guid_prefix[GUID_PREFIX_SIZE],
                               const uint8_t sedp_ip_addr[4],
                               const uint8_t sedp_udp_port[2],
                               const uint8_t sedp_entity_id[4]) {
#pragma HLS inline
    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
    if (ep_type & BUILTIN_EP_SUB) {
        if (!found || (pub_matched == 0)) {
            rtps_data.type = RTPS_TYPE_SEDP_SUB_SN_ONLY;
        } else {
            rtps_data.type = RTPS_TYPE_SEDP_SUB;
            rtps_data.data[10] = get_matched_pub_topic_id(pub_matched);
        }
    } else {
        if (!found || (sub_matched == 0)) {
            rtps_data.type = RTPS_TYPE_SEDP_PUB_SN_ONLY;
        } else {
            rtps_data.type = RTPS_TYPE_SEDP_PUB;
        }
    }
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
        rtps_data.guid_prefix[j] = src_guid_prefix[j];
    }
    rtps_data.data[0] = sedp_ip_addr[0];
    rtps_data.data[1] = sedp_ip_addr[1];
    rtps_data.data[2] = sedp_ip_addr[2];
    rtps_data.data[3] = sedp_ip_addr[3];
    rtps_data.data[4] = sedp_udp_port[0];
    rtps_data.data[5] = sedp_udp_port[1];
    rtps_data.data[6] = sedp_entity_id[0];
    rtps_data.data[7] = sedp_entity_id[1];
    rtps_data.data[8] = sedp_entity_id[2];
    rtps_data.data[9] = sedp_entity_id[3];
    rtps_data.data[11] = seqnum;

    out.write(rtps_data);
}

typedef enum {
    SEDP_READER_STATE_PAYLOAD_HDR,
    SEDP_READER_STATE_PARAM_HDR,
    SEDP_READER_STATE_PARAM_PAYLOAD,
    SEDP_READER_STATE_WAIT_END
} sedp_reader_state_t;

void sedp_reader(
    hls_uint<9> x, hls_stream<rtps_data_t> &out,
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    builtin_ep_type_t ep_type, uint8_t seqnum, const uint8_t reader_ip_addr[4],
    const uint8_t subnet_mask[4], uint16_t port_num_seed,
    const uint8_t pub_topic_name[PUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t pub_topic_name_len[PUB_TOPICS_MAX],
    const uint8_t pub_type_name[PUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t pub_type_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_topic_name[SUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t sub_topic_name_len[SUB_TOPICS_MAX],
    const uint8_t sub_type_name[SUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t sub_type_name_len[SUB_TOPICS_MAX],
    const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    static sedp_reader_state_t state = SEDP_READER_STATE_PAYLOAD_HDR;
    static uint16_t            offset = 0;
    static uint16_t            rep_id;
    static uint16_t            param_id;
    static uint16_t            param_length;
    static uint32_t            name_length;
    bool                       param_le = (rep_id == SP_ID_PL_CDR_LE);

    static bool                     locator_found;
    static bool                     guid_found;
    static bool                     topic_name_found;
    static bool                     type_name_found;
    static hls_uint<PUB_TOPICS_MAX> pub_topics_unmatched;
    static hls_uint<SUB_TOPICS_MAX> sub_topics_unmatched;
    static hls_uint<PUB_TOPICS_MAX> pub_types_unmatched;
    static hls_uint<SUB_TOPICS_MAX> sub_types_unmatched;

    static uint8_t sedp_ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = sedp_ip_addr complete dim = 1
    static uint8_t sedp_udp_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = sedp_udp_port complete dim = 1
    static uint8_t sedp_entity_id[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = sedp_entity_id complete dim = 1

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;

    switch (state) {
    case SEDP_READER_STATE_PAYLOAD_HDR:
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
            locator_found = false;
            guid_found = false;
            topic_name_found = false;
            type_name_found = false;
            pub_topics_unmatched = ~pub_enable;
            sub_topics_unmatched = ~sub_enable;
            pub_types_unmatched = ~pub_enable;
            sub_types_unmatched = ~sub_enable;
            if ((rep_id == SP_ID_PL_CDR_LE) || (rep_id == SP_ID_PL_CDR_BE)) {
                state = SEDP_READER_STATE_PARAM_HDR;
            } else {
                state = SEDP_READER_STATE_WAIT_END;
            }
        }
        break;
    case SEDP_READER_STATE_PARAM_HDR:
        switch (offset) {
        case 0:
            param_id = param_le ? data : (data << 8);
            break;
        case 1:
            param_id |= param_le ? (data << 8) : data;
            break;
        case 2:
            param_length = param_le ? data : (data << 8);
            break;
        case 3:
            param_length |= param_le ? (data << 8) : data;
            break;
        }
        offset++;
        if (offset == 4) {
            offset = 0;
            if (param_id == PID_SENTINEL) {
                bool found = locator_found && guid_found && topic_name_found
                             && type_name_found;
                hls_uint<PUB_TOPICS_MAX> pub_matched
                    = ~pub_topics_unmatched & ~pub_types_unmatched;
                hls_uint<SUB_TOPICS_MAX> sub_matched
                    = ~sub_topics_unmatched & ~sub_types_unmatched;
                send_received_sedp(out, ep_type, seqnum, found, pub_matched,
                                   sub_matched, src_guid_prefix, sedp_ip_addr,
                                   sedp_udp_port, sedp_entity_id);
                state = SEDP_READER_STATE_WAIT_END;
            } else if (param_length == 0) {
                state = SEDP_READER_STATE_PARAM_HDR;
            } else {
                state = SEDP_READER_STATE_PARAM_PAYLOAD;
            }
        }
        break;
    case SEDP_READER_STATE_PARAM_PAYLOAD:
        if (!locator_found && (param_id == PID_UNICAST_LOCATOR)) {
            locator_found = spdp_set_locator(
                offset, data, param_le, sedp_ip_addr, sedp_udp_port,
                reader_ip_addr, subnet_mask, port_num_seed);
        } else if (!guid_found && (param_id == PID_ENDPOINT_GUID)) {
            guid_found = sedp_set_guid(offset, data, sedp_entity_id);
        } else if (!topic_name_found && (param_id == PID_TOPIC_NAME)) {
            topic_name_found = sedp_compare_topic_info<MAX_TOPIC_NAME_LEN>(
                offset, data, &pub_topics_unmatched, &sub_topics_unmatched,
                param_le, &name_length, pub_topic_name, pub_topic_name_len,
                sub_topic_name, sub_topic_name_len);
        } else if (!type_name_found && (param_id == PID_TYPE_NAME)) {
            type_name_found = sedp_compare_topic_info<MAX_TOPIC_TYPE_NAME_LEN>(
                offset, data, &pub_types_unmatched, &sub_types_unmatched,
                param_le, &name_length, pub_type_name, pub_type_name_len,
                sub_type_name, sub_type_name_len);
        }
        offset++;
        if (offset == param_length) {
            offset = 0;
            state = SEDP_READER_STATE_PARAM_HDR;
        }
        break;
    case SEDP_READER_STATE_WAIT_END:
        break;
    }

    if (end) {
        state = SEDP_READER_STATE_PAYLOAD_HDR;
        offset = 0;
    }
}
