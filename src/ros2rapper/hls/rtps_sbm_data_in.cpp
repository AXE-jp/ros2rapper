#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

#define STATE_SBM_DATA_HDR                           0
#define STATE_SBM_DATA_INLINE_QOS_PARAM_HDR          1
#define STATE_SBM_DATA_INLINE_QOS_SKIP_TO_NEXT_PARAM 2
#define STATE_SBM_DATA_INLINE_QOS_STATUS_INFO        3
#define STATE_SBM_DATA_PAYLOAD                       4

void rtps_data_in(hls_uint<9> x, hls_uint<10> *out,
                  hls_stream<rtps_data_t> &rtps_data_stream, uint8_t sbm_flags,
                  const uint8_t guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    static hls_uint<3> state;
    static uint16_t    offset;
    static uint16_t    param_id;
    static uint16_t    length;

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;
    bool    sbm_le = sbm_flags & SBM_FLAGS_ENDIANNESS;
    bool    sbm_inline_qos = sbm_flags & SBM_FLAGS_INLINE_QOS;

    switch (state) {
    case STATE_SBM_DATA_HDR:
        if (offset == SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS) {
            length = sbm_le ? data : (data << 8);
        } else if (offset == (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 1)) {
            length |= sbm_le ? (data << 8) : data;
        }
        if (offset < SBM_DATA_HDR_SIZE) {
            *out = x;
        } else {
            *out = 0x200 | x;
        }
        offset++;
        if ((offset >= (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 2))
            && (offset
                == (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 2 + length))) {
            offset = 0;
            if (sbm_inline_qos) {
                state = STATE_SBM_DATA_INLINE_QOS_PARAM_HDR;
            } else {
                state = STATE_SBM_DATA_PAYLOAD;
            }
        }
        break;
    case STATE_SBM_DATA_INLINE_QOS_PARAM_HDR:
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
        *out = 0x200 | x;
        offset++;
        if (offset == 4) {
            offset = 0;
            if (length == 0) {
                if (param_id == PID_SENTINEL) {
                    state = STATE_SBM_DATA_PAYLOAD;
                } else {
                    state = STATE_SBM_DATA_INLINE_QOS_PARAM_HDR;
                }
            } else if (param_id == PID_STATUS_INFO) {
                state = STATE_SBM_DATA_INLINE_QOS_STATUS_INFO;
            } else {
                state = STATE_SBM_DATA_INLINE_QOS_SKIP_TO_NEXT_PARAM;
            }
        }
        break;
    case STATE_SBM_DATA_INLINE_QOS_SKIP_TO_NEXT_PARAM:
        *out = 0x200 | x;
        offset++;
        if (offset == length) {
            offset = 0;
            if (param_id == PID_SENTINEL) {
                state = STATE_SBM_DATA_PAYLOAD;
            } else {
                state = STATE_SBM_DATA_INLINE_QOS_PARAM_HDR;
            }
        }
        break;
    case STATE_SBM_DATA_INLINE_QOS_STATUS_INFO:
        // See RTPS 2.3 specification 9.6.3.9.
        if ((offset == 3) && ((data & 3) != 0)) {
            // disposed (0x01) or unregistered (0x02)
            rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
            rtps_data.type = RTPS_TYPE_RM_ENDPOINT;
            /* Cyber unroll_times=all */
            for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
                rtps_data.guid_prefix[j] = guid_prefix[j];
            }
            rtps_data_stream.write(rtps_data);
        }
        *out = 0x200 | x;
        offset++;
        if (offset == length) {
            offset = 0;
            state = STATE_SBM_DATA_INLINE_QOS_PARAM_HDR;
        }
        break;
    default:
        *out = x;
        break;
    }

    if (end) {
        offset = 0;
        state = STATE_SBM_DATA_HDR;
    }
}
