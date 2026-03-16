#include "rtps_in.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

#define RTPS_IN_RTPS_HDR     0
#define RTPS_IN_SBM_HDR      1
#define RTPS_IN_SBM_INFO_DST 2
#define RTPS_IN_SBM_BODY     3
#define RTPS_IN_SKIP         4

void rtps_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
             bool enable, const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE],
             uint8_t guid_prefix_out[GUID_PREFIX_SIZE], uint8_t *sbm_id_out,
             uint8_t *sbm_flags_out) {
#pragma HLS inline
    static hls_uint<3> state;
    static uint16_t    offset;

    static uint8_t guid_prefix[GUID_PREFIX_SIZE];
#pragma HLS array_partition variable = guid_prefix
    static uint8_t  sbm_id;
    static uint8_t  sbm_flags;
    static uint16_t sbm_len;
    bool            sbm_le = sbm_flags & SBM_FLAGS_ENDIANNESS;

    hls_uint<9> x = in.read();
    uint8_t     data = x & 0xff;
    bool        end = x & 0x100;

    switch (state) {
    case RTPS_IN_RTPS_HDR:
        if (!rtps_compare_protocol(offset, data)) {
            state = RTPS_IN_SKIP;
            break;
        }
        if ((offset >= RTPS_HDR_OFFSET_GUID_PREFIX)
            && (offset < (RTPS_HDR_OFFSET_GUID_PREFIX + GUID_PREFIX_SIZE))) {
            guid_prefix[offset - RTPS_HDR_OFFSET_GUID_PREFIX] = data;
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
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_flags = data;
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
            && (data != reader_guid_prefix[offset])) {
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
        if (offset < sbm_len) {
            out.write(x);
        } else {
            out.write(0x100 | data);
            state = RTPS_IN_SBM_HDR;
            offset = 0;
        }
        break;
    }

    if (end) {
        state = RTPS_IN_RTPS_HDR;
        offset = 0;
    }

    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        guid_prefix_out[j] = guid_prefix[j];
    }
    *sbm_id_out = sbm_id;
    *sbm_flags_out = sbm_flags;
}
