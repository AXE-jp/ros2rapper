#include "rtps_in.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

#define RTPS_IN_STATE_RTPS_HDR     0
#define RTPS_IN_STATE_SBM_HDR      1
#define RTPS_IN_STATE_SBM_INFO_DST 2
#define RTPS_IN_STATE_SBM_PAYLOAD  3
#define RTPS_IN_STATE_SKIP         4

static bool compare_guid_prefix(uint16_t offset, uint8_t data,
                                const uint8_t guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        if ((offset == j) && (data != guid_prefix[j])) {
            return false;
        }
    }
    return true;
}

void rtps_in_send_output(hls_stream<hls_uint<10>> &out, uint8_t data, bool end,
                         bool valid) {
#pragma HLS inline
    hls_uint<10> x = data;
    if (end) {
        x |= 0x100;
    }
    if (valid) {
        x |= 0x200;
    }
    out.write(x);
}

void rtps_in(hls_stream<hls_uint<9>>  &in,
             hls_stream<hls_uint<10>> &out_guid_prefix,
             hls_stream<hls_uint<10>> &out_sbm_heartbeat,
             hls_stream<hls_uint<10>> &out_sbm_data, hls_uint<1> enable,
             const uint8_t guid_prefix[GUID_PREFIX_SIZE]) {

#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out_guid_prefix
#pragma HLS interface mode = axis port = out_sbm_heartbeat
#pragma HLS interface mode = axis port = out_sbm_data
#pragma HLS interface mode = ap_none port = enable
#pragma HLS array_reshape variable = guid_prefix type = complete dim = 0
#pragma HLS interface mode = ap_none port = guid_prefix
    static hls_uint<3> state = RTPS_IN_STATE_RTPS_HDR;
    static uint16_t    offset = 0;
    static uint8_t     sbm_id;
    static bool        sbm_le;
    static uint16_t    sbm_len;

    hls_uint<9> x = in.read();
    uint8_t     data = x & 0xff;
    bool        end = x & 0x100;

    bool sbm_end = end;
    bool guid_prefix_valid = false;
    bool sbm_heartbeat_valid = false;
    bool sbm_data_valid = false;

#define set_sbm_valid_flags()                                                  \
    do {                                                                       \
        if (sbm_id == SBM_ID_HEARTBEAT) {                                      \
            sbm_heartbeat_valid = true;                                        \
        } else if (sbm_id == SBM_ID_DATA) {                                    \
            sbm_data_valid = true;                                             \
        }                                                                      \
    } while (0)

    switch (state) {
    case RTPS_IN_STATE_RTPS_HDR:
        if (!rtps_compare_protocol(offset, data)) {
            state = RTPS_IN_STATE_SKIP;
            break;
        }
        if ((offset >= RTPS_HDR_OFFSET_GUID_PREFIX)
            && (offset < (RTPS_HDR_OFFSET_GUID_PREFIX + GUID_PREFIX_SIZE))) {
            guid_prefix_valid = true;
        }
        offset++;
        if (offset == RTPS_HDR_SIZE) {
            offset = 0;
            if (enable) {
                state = RTPS_IN_STATE_SBM_HDR;
            } else {
                state = RTPS_IN_STATE_SKIP;
            }
        }
        break;
    case RTPS_IN_STATE_SBM_HDR:
        switch (offset) {
        case SBM_HDR_OFFSET_SUBMESSAGE_ID:
            sbm_id = data;
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_le = data & SBM_FLAGS_ENDIANNESS;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER:
            sbm_len = sbm_le ? data : (data << 8);
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER + 1:
            sbm_len |= sbm_le ? (data << 8) : data;
            break;
        }
        // Set sbm_*_valid flags after "sbm_id = data".
        set_sbm_valid_flags();
        offset++;
        if (offset == SBM_HDR_SIZE) {
            offset = 0;
            if (sbm_id == SBM_ID_INFO_DST) {
                state = RTPS_IN_STATE_SBM_INFO_DST;
            } else {
                state = RTPS_IN_STATE_SBM_PAYLOAD;
            }
        }
        break;
    case RTPS_IN_STATE_SBM_INFO_DST:
        if (!compare_guid_prefix(offset, data, guid_prefix)) {
            state = RTPS_IN_STATE_SKIP;
            break;
        }
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = RTPS_IN_STATE_SBM_HDR;
        }
        break;
    case RTPS_IN_STATE_SBM_PAYLOAD:
        set_sbm_valid_flags();
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = RTPS_IN_STATE_SBM_HDR;
            sbm_end = true;
        }
        break;
    }

    if (end) {
        state = RTPS_IN_STATE_RTPS_HDR;
        offset = 0;
    }

    rtps_in_send_output(out_guid_prefix, data, end, guid_prefix_valid);
    rtps_in_send_output(out_sbm_heartbeat, data, sbm_end, sbm_heartbeat_valid);
    rtps_in_send_output(out_sbm_data, data, sbm_end, sbm_data_valid);
}
