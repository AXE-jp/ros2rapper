// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "app.hpp"
#include "ros2.hpp"

#define APP_HDR_SIZE APP_TOT_LEN(0)

/* Cyber func=inline */
void app_writer(const uint8_t writer_guid_prefix[12],
                const uint8_t writer_entity_id[4],
                const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id[4], const int64_t seqnum,
                volatile const uint8_t app_data[MAX_APP_DATA_LEN],
                uint32_t app_data_len, uint8_t buf[]) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t  sbm_flags = SBM_FLAGS_ENDIANNESS;
    static const uint16_t rep_id = SP_ID_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t  sbm_flags = 0;
    static const uint16_t rep_id = SP_ID_CDR_BE;
#endif // SBM_ENDIAN_BIG

    static const timestamp now = TIME_ZERO;
    static const uint16_t  ext_flags = 0;
    static const uint16_t  rep_opt = 0;

    const uint16_t tot_len = APP_TOT_LEN(app_data_len);
    const uint16_t octets_to_next_header
        = APP_OCTETS_TO_NEXT_HEADER(app_data_len);

    int32_t  seqnum_h = seqnum >> 32;
    uint32_t seqnum_l = seqnum & 0xffffffff;

    buf[0] = 'R';
    buf[1] = 'T';
    buf[2] = 'P';
    buf[3] = 'S';
    buf[4] = RTPS_HDR_PROTOCOL_VERSION >> 8;
    buf[5] = RTPS_HDR_PROTOCOL_VERSION & 0xff;
    buf[6] = RTPS_HDR_VENDOR_ID >> 8;
    buf[7] = RTPS_HDR_VENDOR_ID & 0xff;
    buf[8] = writer_guid_prefix[0];
    buf[9] = writer_guid_prefix[1];
    buf[10] = writer_guid_prefix[2];
    buf[11] = writer_guid_prefix[3];
    buf[12] = writer_guid_prefix[4];
    buf[13] = writer_guid_prefix[5];
    buf[14] = writer_guid_prefix[6];
    buf[15] = writer_guid_prefix[7];
    buf[16] = writer_guid_prefix[8];
    buf[17] = writer_guid_prefix[9];
    buf[18] = writer_guid_prefix[10];
    buf[19] = writer_guid_prefix[11];
    buf[20] = SBM_ID_INFO_DST;
    buf[21] = sbm_flags;
    buf[22] = S_BYTE0(GUID_PREFIX_SIZE);
    buf[23] = S_BYTE1(GUID_PREFIX_SIZE);
    buf[24] = reader_guid_prefix[0];
    buf[25] = reader_guid_prefix[1];
    buf[26] = reader_guid_prefix[2];
    buf[27] = reader_guid_prefix[3];
    buf[28] = reader_guid_prefix[4];
    buf[29] = reader_guid_prefix[5];
    buf[30] = reader_guid_prefix[6];
    buf[31] = reader_guid_prefix[7];
    buf[32] = reader_guid_prefix[8];
    buf[33] = reader_guid_prefix[9];
    buf[34] = reader_guid_prefix[10];
    buf[35] = reader_guid_prefix[11];
    buf[36] = SBM_ID_INFO_TS;
    buf[37] = sbm_flags;
    buf[38] = S_BYTE0(TIMESTAMP_SIZE);
    buf[39] = S_BYTE1(TIMESTAMP_SIZE);
    buf[40] = L_BYTE0(now.seconds);
    buf[41] = L_BYTE1(now.seconds);
    buf[42] = L_BYTE2(now.seconds);
    buf[43] = L_BYTE3(now.seconds);
    buf[44] = L_BYTE0(now.fraction);
    buf[45] = L_BYTE1(now.fraction);
    buf[46] = L_BYTE2(now.fraction);
    buf[47] = L_BYTE3(now.fraction);
    buf[48] = SBM_ID_DATA;
    buf[49] = sbm_flags | SBM_FLAGS_DATA;
    buf[50] = S_BYTE0(octets_to_next_header);
    buf[51] = S_BYTE1(octets_to_next_header);
    buf[52] = ext_flags >> 8;
    buf[53] = ext_flags & 0xff;
    buf[54] = S_BYTE0(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
    buf[55] = S_BYTE1(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
    buf[56] = reader_entity_id[0];
    buf[57] = reader_entity_id[1];
    buf[58] = reader_entity_id[2];
    buf[59] = reader_entity_id[3];
    buf[60] = writer_entity_id[0];
    buf[61] = writer_entity_id[1];
    buf[62] = writer_entity_id[2];
    buf[63] = writer_entity_id[3];
    buf[64] = L_BYTE0(seqnum_h);
    buf[65] = L_BYTE1(seqnum_h);
    buf[66] = L_BYTE2(seqnum_h);
    buf[67] = L_BYTE3(seqnum_h);
    buf[68] = L_BYTE0(seqnum_l);
    buf[69] = L_BYTE1(seqnum_l);
    buf[70] = L_BYTE2(seqnum_l);
    buf[71] = L_BYTE3(seqnum_l);
    buf[72] = rep_id >> 8;
    buf[73] = rep_id & 0xff;
    buf[74] = rep_opt >> 8;
    buf[75] = rep_opt & 0xff;

    /* Cyber unroll_times=all */
    for (int i = APP_HDR_SIZE; i < APP_HDR_SIZE + MAX_APP_DATA_LEN; i++) {
#pragma HLS unroll
        buf[i] = app_data[i - APP_HDR_SIZE];
    }

    /* Cyber unroll_times=all */
    for (int i = APP_HDR_SIZE + MAX_APP_DATA_LEN; i < MAX_TX_UDP_PAYLOAD_LEN;
         i++) {
#pragma HLS unroll
        buf[i] = 0;
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
                const uint8_t      reader_entity_id[4],
                volatile uint8_t  *sub_app_data_recv,
                volatile uint8_t  *sub_app_data_req,
                volatile uint8_t  *sub_app_data_rel,
                volatile uint8_t  *sub_app_data_grant,
                uint8_t            sub_app_data[MAX_APP_DATA_LEN],
                volatile uint8_t  *sub_app_data_len,
                volatile uint16_t *sub_app_data_rep_id) {
#pragma HLS inline

    static hls_uint<3> state;
    static uint16_t    offset;
    static uint8_t     sbm_id;
    static bool        sbm_le;
    static uint16_t    sbm_len;
    static uint16_t    rep_id;

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
        if (!rtps_compare_reader_id(offset, data, reader_entity_id)) {
            state = STATE_WAIT_END;
            break;
        }
        offset++;
        if (offset == SBM_DATA_HDR_SIZE) {
            sbm_len -= SBM_DATA_HDR_SIZE;
            offset = 0;
            *sub_app_data_req = 0;
            state = STATE_PARSE_PAYLOAD_HDR;
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
            if (sbm_len != 0 && *sub_app_data_grant == 1)
                state = STATE_PARSE_PAYLOAD_DATA;
            else
                state = STATE_WAIT_END;
        }
        break;
    case STATE_PARSE_PAYLOAD_DATA:
        sub_app_data[offset] = data;
        offset++;
        if (offset == MAX_APP_DATA_LEN || offset == sbm_len) {
            *sub_app_data_len = sbm_len;
            *sub_app_data_rep_id = rep_id;
            *sub_app_data_recv = 0;
            *sub_app_data_rel = 0;
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
        offset = 0;
        state = STATE_PARSE_RTPS_HDR;
    }
}
