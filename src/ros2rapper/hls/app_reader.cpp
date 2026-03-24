#include "app_reader.hpp"
#include "common.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "rtps.hpp"

enum app_reader_state_t {
    APP_READER_STATE_PAYLOAD_HDR,
    APP_READER_STATE_PAYLOAD,
    APP_READER_STATE_WAIT_END
};

void app_reader(hls_uint<9> x, hls_uint<SUB_TOPICS_MAX> app_matched,
                hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
                hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
                hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
                uint8_t                   sub_app_data_0[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_1[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_2[MAX_APP_DATA_LEN],
                uint8_t                   sub_app_data_3[MAX_APP_DATA_LEN],
                hls_stream<uint64_t>     &sub_app_data_recvinfo) {
#pragma HLS inline
    static app_reader_state_t state = APP_READER_STATE_PAYLOAD_HDR;
    static uint16_t           offset = 0;
    static uint16_t           rep_id;

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;

    hls_uint<SUB_TOPICS_MAX> app_valid = app_matched & sub_app_data_grant;

    switch (state) {
    case APP_READER_STATE_PAYLOAD_HDR:
        switch (offset) {
        case SP_HDR_OFFSET_REPRESENTATION_ID:
            rep_id = data << 8;
            break;
        case SP_HDR_OFFSET_REPRESENTATION_ID + 1:
            rep_id |= data;
            break;
        }
        if (offset == 0) {
            *sub_app_data_req = app_matched;
        }
        offset++;
        if (offset == SP_HDR_SIZE) {
            offset = 0;
            if (app_valid != 0) {
                state = APP_READER_STATE_PAYLOAD;
            } else {
                state = APP_READER_STATE_WAIT_END;
            }
        }
        break;
    case APP_READER_STATE_PAYLOAD:
        if (app_valid[0]) {
            sub_app_data_0[offset] = data;
        }
        if (app_valid[1]) {
            sub_app_data_1[offset] = data;
        }
        if (app_valid[2]) {
            sub_app_data_2[offset] = data;
        }
        if (app_valid[3]) {
            sub_app_data_3[offset] = data;
        }
        offset++;
        if (end || (offset == MAX_APP_DATA_LEN)) {
            *sub_app_data_rel = sub_app_data_grant;
            uint64_t info = static_cast<uint64_t>(offset);
            info |= static_cast<uint64_t>(rep_id) << 16;
            info |= static_cast<uint64_t>(app_valid) << 32;
            sub_app_data_recvinfo.write(info);
            state = APP_READER_STATE_WAIT_END;
        }
        break;
    case APP_READER_STATE_WAIT_END:
        break;
    }

    if (end) {
        *sub_app_data_rel = sub_app_data_grant;
        state = APP_READER_STATE_PAYLOAD_HDR;
        offset = 0;
    }
}
