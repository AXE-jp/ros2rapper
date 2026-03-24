#include "rtps_sbm_data_in.hpp"
#include "app_reader.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include "sedp_reader.hpp"
#include "spdp_reader.hpp"

static void
send_received_status_info(hls_stream<rtps_data_t> &out,
                          const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
    rtps_data.type = RTPS_TYPE_RM_ENDPOINT;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        rtps_data.guid_prefix[j] = src_guid_prefix[j];
    }
    out.write(rtps_data);
}

enum rtps_sbm_data_in_state_t {
    RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR,
    RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR,
    RTPS_SBM_DATA_IN_STATE_INLINE_QOS_STATUS_INFO,
    RTPS_SBM_DATA_IN_STATE_INLINE_QOS_SKIP,
    RTPS_SBM_DATA_IN_STATE_PAYLOAD
};

void rtps_sbm_data_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                      hls_uint<PUB_TOPICS_MAX> pub_enable,
                      hls_uint<SUB_TOPICS_MAX> sub_enable, uint8_t sbm_flags,
                      const uint8_t src_guid_prefix[GUID_PREFIX_SIZE],
                      hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                      hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                      hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
                      uint8_t                  sub_app_data_0[MAX_APP_DATA_LEN],
                      uint8_t                  sub_app_data_1[MAX_APP_DATA_LEN],
                      uint8_t                  sub_app_data_2[MAX_APP_DATA_LEN],
                      uint8_t                  sub_app_data_3[MAX_APP_DATA_LEN],
                      hls_stream<uint64_t>    &sub_app_data_recvinfo,
                      const receiver_config_t &conf) {
#pragma HLS inline
    static const uint8_t participant_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PARTICIPANT_READER;
#pragma HLS array_partition variable = participant_reader_id complete dim = 0
    static const uint8_t pub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
#pragma HLS array_partition variable = pub_reader_id complete dim = 0
    static const uint8_t sub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = sub_reader_id complete dim = 0
    static const uint8_t app_reader_id_list[SUB_TOPICS_MAX]
                                           [4] /* Cyber array=EXPAND */
        = ENTITYID_APP_READER_LIST;
#pragma HLS array_partition variable = app_reader_id_list complete dim = 0

    static rtps_sbm_data_in_state_t state = RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR;
    static uint16_t                 offset = 0;
    static uint16_t                 param_id;
    static uint16_t                 length;
    bool                            sbm_le = sbm_flags & SBM_FLAGS_ENDIANNESS;
    bool sbm_inline_qos = sbm_flags & SBM_FLAGS_INLINE_QOS;

    static bool                     is_spdp = true;
    static builtin_ep_type_t        ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    static hls_uint<SUB_TOPICS_MAX> app_unmatched = 0;
    static uint8_t                  seqnum;

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;

    switch (state) {
    case RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR:
        // Octets to InlineQoS
        if (offset == SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS) {
            length = sbm_le ? data : (data << 8);
        } else if (offset == (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 1)) {
            length |= sbm_le ? (data << 8) : data;
        }
        // Reader Entity ID
        if (!rtps_compare_data_hdr_reader_id(offset, data,
                                             participant_reader_id)) {
            is_spdp = false;
        }
        if (!rtps_compare_data_hdr_reader_id(offset, data, pub_reader_id)) {
            ep_type &= ~(BUILTIN_EP_PUB);
        }
        if (!rtps_compare_data_hdr_reader_id(offset, data, sub_reader_id)) {
            ep_type &= ~(BUILTIN_EP_SUB);
        }
        /* Cyber unroll_times=all */
        for (auto j = 0; j < SUB_TOPICS_MAX; j++) {
#pragma HLS unroll
            if (!sub_enable[j]
                || !rtps_compare_data_hdr_reader_id(offset, data,
                                                    app_reader_id_list[j])) {
                app_unmatched[j] = 1;
            }
        }
        // Sequence Number
        // Ignore other than lower 8-bit of Sequence Number to reduce resources
        if (sbm_le && (offset == (SBM_DATA_HDR_OFFSET_WRITER_SN + 4))) {
            seqnum = data;
        } else if (!sbm_le && (offset == (SBM_DATA_HDR_OFFSET_WRITER_SN + 7))) {
            seqnum = data;
        }
        offset++;
        if ((offset >= (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 2))
            && (offset
                == (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 2 + length))) {
            offset = 0;
            if (sbm_inline_qos) {
                state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR;
            } else {
                state = RTPS_SBM_DATA_IN_STATE_PAYLOAD;
            }
        }
        break;
    case RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR:
        switch (offset) {
        case 0:
            param_id = sbm_le ? data : (data << 8);
            break;
        case 1:
            param_id |= sbm_le ? (data << 8) : data;
            break;
        case 2:
            length = sbm_le ? data : (data << 8);
            break;
        case 3:
            length |= sbm_le ? (data << 8) : data;
            break;
        }
        offset++;
        if (offset == 4) {
            offset = 0;
            if (length == 0) {
                if (param_id == PID_SENTINEL) {
                    state = RTPS_SBM_DATA_IN_STATE_PAYLOAD;
                } else {
                    state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR;
                }
            } else if (param_id == PID_STATUS_INFO) {
                state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_STATUS_INFO;
            } else {
                state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_SKIP;
            }
        }
        break;
    case RTPS_SBM_DATA_IN_STATE_INLINE_QOS_SKIP:
        offset++;
        if (offset == length) {
            offset = 0;
            if (param_id == PID_SENTINEL) {
                state = RTPS_SBM_DATA_IN_STATE_PAYLOAD;
            } else {
                state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR;
            }
        }
        break;
    case RTPS_SBM_DATA_IN_STATE_INLINE_QOS_STATUS_INFO:
        // See RTPS 2.3 specification 9.6.3.9.
        if ((offset == 3) && ((data & 3) != 0)) {
            // disposed (0x01) or unregistered (0x02)
            send_received_status_info(out, src_guid_prefix);
        }
        offset++;
        if (offset == length) {
            offset = 0;
            state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR;
        }
        break;
    case RTPS_SBM_DATA_IN_STATE_PAYLOAD:
        if (is_spdp) {
            spdp_reader(x, out, conf.ip_addr, conf.subnet_mask,
                        conf.port_num_seed, src_guid_prefix);
        } else if (ep_type != 0) {
            sedp_reader(x, out, pub_enable, sub_enable, ep_type, seqnum,
                        conf.ip_addr, conf.subnet_mask, conf.port_num_seed,
                        conf.pub_topic_name, conf.pub_topic_name_len,
                        conf.pub_topic_type_name, conf.pub_topic_type_name_len,
                        conf.sub_topic_name, conf.sub_topic_name_len,
                        conf.sub_topic_type_name, conf.sub_topic_type_name_len,
                        src_guid_prefix);
        } else if (~app_unmatched != 0) {
            app_reader(x, ~app_unmatched, sub_app_data_req, sub_app_data_rel,
                       sub_app_data_grant, sub_app_data_0, sub_app_data_1,
                       sub_app_data_2, sub_app_data_3, sub_app_data_recvinfo);
        }
        break;
    }

    if (end) {
        state = RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR;
        offset = 0;
        is_spdp = true;
        ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
        app_unmatched = 0;
    }
}
