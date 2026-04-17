// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "app.hpp"
#include "ros2.hpp"

#define APP_HDR_SIZE APP_TOT_LEN(0)

/* Cyber func=inline */
void app_writer(const uint8_t vendor_id[2],
                const uint8_t writer_guid_prefix[12],
                const uint8_t writer_entity_id[4],
                const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id[4], const int64_t seqnum,
#ifdef PUB_DATA_FF
                VOLATILE
#endif // PUB_DATA_FF
                const uint32_t app_data[MAX_APP_DATA_LEN / 4],
                uint32_t app_data_len, hls_stream<uint8_t> &out,
                timestamp now) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t  sbm_flags = SBM_FLAGS_ENDIANNESS;
    static const uint16_t rep_id = SP_ID_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t  sbm_flags = 0;
    static const uint16_t rep_id = SP_ID_CDR_BE;
#endif // SBM_ENDIAN_BIG

    static const uint16_t ext_flags = 0;
    static const uint16_t rep_opt = 0;

    const uint16_t tot_len = APP_TOT_LEN(app_data_len);
    const uint16_t octets_to_next_header
        = APP_OCTETS_TO_NEXT_HEADER(app_data_len);

    int32_t  seqnum_h = seqnum >> 32;
    uint32_t seqnum_l = seqnum & 0xffffffff;

    out.write('R');
    out.write('T');
    out.write('P');
    out.write('S');
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(vendor_id[0]);
    out.write(vendor_id[1]);
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

#ifdef PUB_DATA_FF
    /* Cyber unroll_times=all */
#else  // !PUB_DATA_FF
    /* Cyber folding=4 */
#endif // PUB_DATA_FF
    for (auto j = 0; j < (MAX_APP_DATA_LEN / 4); j++) {
#ifdef PUB_DATA_FF
#pragma HLS unroll
#else // !PUB_DATA_FF
#pragma HLS pipeline II = 4
#endif // PUB_DATA_FF
        if ((4 * j) >= app_data_len) {
            break;
        }
        uint32_t data = app_data[j];
        out.write(data & 0xff);
        out.write((data >> 8) & 0xff);
        out.write((data >> 16) & 0xff);
        out.write(data >> 24);
    }
}

enum {
    STATE_PARSE_RTPS_HDR,
    STATE_PARSE_SUBMSG_HDR,
    STATE_PARSE_INFO_DST,
    STATE_PARSE_DATA,
    STATE_PARSE_PAYLOAD_HDR,
    STATE_PARSE_PAYLOAD_DATA,
    STATE_PARSE_OTHER,
    STATE_WAIT_END,
};

/* Cyber func=inline */
void app_reader(hls_uint<9> in, const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id_list[SUB_TOPICS_MAX][4],
                hls_uint<SUB_TOPICS_MAX> sub_enabled,
                VOLATILE hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                VOLATILE hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                VOLATILE hls_uint<SUB_TOPICS_MAX> *sub_app_data_grant,
                uint8_t               sub_app_data_0[MAX_APP_DATA_LEN],
                uint8_t               sub_app_data_1[MAX_APP_DATA_LEN],
                uint8_t               sub_app_data_2[MAX_APP_DATA_LEN],
                uint8_t               sub_app_data_3[MAX_APP_DATA_LEN],
                hls_stream<uint64_t> &sub_app_data_recvinfo) {
#pragma HLS inline

    static hls_uint<3> state;
    static uint16_t    offset;
    static uint8_t     sbm_id;
    static bool        sbm_le;
    static uint16_t    sbm_len;
    static uint16_t    rep_id;

    static hls_uint<SUB_TOPICS_MAX> topics_unmatched;

    uint8_t data;
    bool    end;

    data = in & 0x0FF;
    end = in & 0x100;

    switch (state) {
    case STATE_PARSE_RTPS_HDR: // parse/check RTPS header
        if (!rtps_compare_protocol(offset, data)) {
            state = STATE_WAIT_END;
            break;
        }
        offset++;
        if (offset == RTPS_HDR_SIZE) {
            offset = 0;
            state = STATE_PARSE_SUBMSG_HDR;
        }
        break;
    case STATE_PARSE_SUBMSG_HDR: // parse/check sub-message header
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
            if (sbm_id == SBM_ID_INFO_DST)
                state = STATE_PARSE_INFO_DST;
            else if (sbm_id == SBM_ID_DATA)
                state = STATE_PARSE_DATA;
            else
                state = STATE_PARSE_OTHER;
        }
        break;
    case STATE_PARSE_INFO_DST: // parse/check sub-message : INFO_DST
        if (offset < 12) {
            if (reader_guid_prefix[offset] != data) {
                state = STATE_WAIT_END;
                break;
            }
        }
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = STATE_PARSE_SUBMSG_HDR;
        }
        break;
    case STATE_PARSE_DATA: // parse/check sub-message : DATA
        /* Cyber unroll_times=all */
        for (auto j = 0; j < SUB_TOPICS_MAX; j++) {
#pragma HLS unroll
            if (!sub_enabled[j]
                || !rtps_compare_data_hdr_reader_id(offset, data,
                                                    reader_entity_id_list[j])) {
                topics_unmatched |= hls_uint<SUB_TOPICS_MAX>(1 << j);
            }
        }
        offset++;
        if (offset == SBM_DATA_HDR_SIZE) {
            if (~topics_unmatched == 0) {
                state = STATE_WAIT_END;
            } else {
                sbm_len -= SBM_DATA_HDR_SIZE;
                offset = 0;
                *sub_app_data_req = ~topics_unmatched;
                state = STATE_PARSE_PAYLOAD_HDR;
            }
        }
        break;
    case STATE_PARSE_PAYLOAD_HDR: // parse/check serialized_payload
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
            topics_unmatched |= ~hls_uint<SUB_TOPICS_MAX>(*sub_app_data_grant);
            if (sbm_len != 0 && ~topics_unmatched != 0)
                state = STATE_PARSE_PAYLOAD_DATA;
            else
                state = STATE_WAIT_END;
        }
        break;
    case STATE_PARSE_PAYLOAD_DATA:
        if (!topics_unmatched[0]) {
            sub_app_data_0[offset] = data;
        }
        if (!topics_unmatched[1]) {
            sub_app_data_1[offset] = data;
        }
        if (!topics_unmatched[2]) {
            sub_app_data_2[offset] = data;
        }
        if (!topics_unmatched[3]) {
            sub_app_data_3[offset] = data;
        }
        offset++;
        if (offset == MAX_APP_DATA_LEN || offset == sbm_len) {
            *sub_app_data_rel = ~topics_unmatched;
            uint64_t info = (uint64_t)sbm_len | ((uint64_t)rep_id << 16)
                            | ((uint64_t)(~topics_unmatched) << 32);
            sub_app_data_recvinfo.write(info);
            state = STATE_WAIT_END;
        }
        break;
    case STATE_PARSE_OTHER:
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = STATE_PARSE_SUBMSG_HDR;
        }
        break;
    case STATE_WAIT_END:; // do nothing
    }

    if (end) {
        topics_unmatched = 0;
        offset = 0;
        state = STATE_PARSE_RTPS_HDR;
    }
}
