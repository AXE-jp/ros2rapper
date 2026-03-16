#include "rtps_in.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

#define RTPS_IN_RTPS_HDR     0
#define RTPS_IN_SBM_HDR      1
#define RTPS_IN_SBM_INFO_DST 2
#define RTPS_IN_SBM_BODY     3
#define RTPS_IN_SKIP         4

void rtps_in(hls_stream<hls_uint<9>> &in, hls_stream<rtps_in_data_t> &out,
             hls_uint<1>   enable,
             const uint8_t guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = ap_none port = enable
#pragma HLS array_reshape variable = guid_prefix type = complete dim = 0
#pragma HLS interface mode = ap_none port = guid_prefix
    static hls_uint<3> state;
    static uint16_t    offset;
    static uint8_t     sbm_id;
    static bool        sbm_le;
    static uint16_t    sbm_len;

    hls_uint<9> x = in.read();
    uint8_t     data = x & 0xff;
    bool        end = x & 0x100;

    rtps_in_data_t out_data;
    out_data.data = x;

    switch (state) {
    case RTPS_IN_RTPS_HDR:
        if (!rtps_compare_protocol(offset, data)) {
            state = RTPS_IN_SKIP;
            break;
        }
        if ((offset >= RTPS_HDR_OFFSET_GUID_PREFIX)
            && (offset < (RTPS_HDR_OFFSET_GUID_PREFIX + GUID_PREFIX_SIZE))) {
            out_data.index = offset - RTPS_HDR_OFFSET_GUID_PREFIX;
            out.write(out_data);
        }
        offset++;
        if (offset == RTPS_HDR_SIZE) {
            if (enable) {
                state = RTPS_IN_SBM_HDR;
                offset = 0;
            } else {
                state = RTPS_IN_SKIP;
            }
        }
        break;
    case RTPS_IN_SBM_HDR:
        switch (offset) {
        case SBM_HDR_OFFSET_SUBMESSAGE_ID:
            sbm_id = data;
            out_data.index = RTPS_IN_DATA_SBM_ID;
            out.write(out_data);
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_le = data & SBM_FLAGS_ENDIANNESS;
            out_data.index = RTPS_IN_DATA_SBM_FLAGS;
            out.write(out_data);
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER:
            sbm_len = sbm_le ? data : (data << 8);
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER + 1:
            sbm_len |= sbm_le ? (data << 8) : data;
            break;
        }
        offset++;
        if (offset == SBM_HDR_SIZE) {
            if (sbm_id == SBM_ID_INFO_DST) {
                state = RTPS_IN_SBM_INFO_DST;
                offset = 0;
            } else {
                state = RTPS_IN_SBM_BODY;
                offset = 0;
            }
        }
        break;
    case RTPS_IN_SBM_INFO_DST:
        if ((offset < GUID_PREFIX_SIZE)
            && (data != guid_prefix[offset])) {
            state = RTPS_IN_SKIP;
        }
        offset++;
        if (offset == sbm_len) {
            state = RTPS_IN_SBM_HDR;
            offset = 0;
        }
        break;
    case RTPS_IN_SBM_BODY:
        offset++;
        if (offset == sbm_len) {
            state = RTPS_IN_SBM_HDR;
            offset = 0;
            out_data.data |= 0x100;
        }
        out_data.index = RTPS_IN_DATA_SBM_PAYLOAD;
        out.write(out_data);
        break;
    }

    if (end) {
        state = RTPS_IN_RTPS_HDR;
        offset = 0;
    }
}
