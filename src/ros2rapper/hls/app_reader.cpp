#include "app_reader.hpp"
#include "common.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "rtps.hpp"

#define STATE_PARSE_DATA         0
#define STATE_PARSE_PAYLOAD_HDR  1
#define STATE_PARSE_PAYLOAD_DATA 2
#define STATE_WAIT_END           3

void app_reader(hls_stream<hls_uint<10>> &in,
                hls_uint<SUB_TOPICS_MAX>  sub_enable,
                hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
                uint8_t                   sub_app_data_0[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_1[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_2[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_3[MAX_APP_DATA_LEN],
                hls_stream<uint64_t>     &sub_app_data_recvinfo) {

#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS interface mode = ap_vld port = sub_app_data_req
#pragma HLS interface mode = ap_vld port = sub_app_data_rel
#pragma HLS interface mode = ap_none port = sub_app_data_grant
#pragma HLS interface mode = ap_memory port = sub_app_data_0 storage_type      \
    = ram_1p
#pragma HLS interface mode = ap_memory port = sub_app_data_1 storage_type      \
    = ram_1p
#pragma HLS interface mode = ap_memory port = sub_app_data_2 storage_type      \
    = ram_1p
#pragma HLS interface mode = ap_memory port = sub_app_data_3 storage_type      \
    = ram_1p
#pragma HLS interface mode = axis port = sub_app_data_recvinfo

    static const uint8_t reader_entity_id_list[SUB_TOPICS_MAX]
                                              [4] /* Cyber array=EXPAND */
        = ENTITYID_APP_READER_LIST;
#pragma HLS array_partition variable = reader_entity_id_list complete dim = 0

    static hls_uint<2>              state = STATE_PARSE_DATA;
    static uint16_t                 offset = 0;
    static hls_uint<SUB_TOPICS_MAX> topics_unmatched = 0;
    static uint16_t                 rep_id;

    hls_uint<10> x = in.read();
    uint8_t      data = x & 0xff;
    bool         end = x & 0x100;
    bool         valid = x & 0x200;

    if (valid) {
        switch (state) {
        case STATE_PARSE_DATA: // parse/check sub-message : DATA
            /* Cyber unroll_times=all */
            for (auto j = 0; j < SUB_TOPICS_MAX; j++) {
#pragma HLS unroll
                if (!sub_enable[j]
                    || !rtps_compare_data_hdr_reader_id(
                        offset, data, reader_entity_id_list[j])) {
                    topics_unmatched |= hls_uint<SUB_TOPICS_MAX>(1 << j);
                }
            }
            offset++;
            if (offset == SBM_DATA_HDR_SIZE) {
                offset = 0;
                if (~topics_unmatched == 0) {
                    state = STATE_WAIT_END;
                } else {
                    state = STATE_PARSE_PAYLOAD_HDR;
                    *sub_app_data_req = ~topics_unmatched;
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
                offset = 0;
                topics_unmatched
                    |= ~hls_uint<SUB_TOPICS_MAX>(sub_app_data_grant);
                if (~topics_unmatched == 0) {
                    state = STATE_WAIT_END;
                } else {
                    state = STATE_PARSE_PAYLOAD_DATA;
                }
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
            if (end || (offset == MAX_APP_DATA_LEN)) {
                *sub_app_data_rel = sub_app_data_grant;
                uint64_t info = static_cast<uint64_t>(offset);
                info |= static_cast<uint64_t>(rep_id) << 16;
                info |= static_cast<uint64_t>(~topics_unmatched) << 32;
                sub_app_data_recvinfo.write(info);
                state = STATE_WAIT_END;
            }
            break;
        }
    }

    if (end) {
        *sub_app_data_rel = sub_app_data_grant;
        state = STATE_PARSE_DATA;
        offset = 0;
        topics_unmatched = 0;
    }
}
