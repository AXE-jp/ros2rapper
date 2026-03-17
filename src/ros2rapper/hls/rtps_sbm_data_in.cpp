#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

#define RTPS_SBM_DATA_IN_STATE_SBM_HDR                0
#define RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR           1
#define RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR   2
#define RTPS_SBM_DATA_IN_STATE_INLINE_QOS_SKIP        3
#define RTPS_SBM_DATA_IN_STATE_INLINE_QOS_STATUS_INFO 4
#define RTPS_SBM_DATA_IN_STATE_PAYLOAD                5

void rtps_sbm_data_in(hls_stream<hls_uint<10>> &in,
                      hls_stream<hls_uint<1>>  &out_status_info,
                      hls_stream<hls_uint<10>> &out_spdp_reader,
                      hls_stream<hls_uint<10>> &out_sedp_reader,
                      hls_stream<hls_uint<10>> &out_app_reader) {

#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out_status_info
#pragma HLS interface mode = axis port = out_spdp_reader
#pragma HLS interface mode = axis port = out_sedp_reader
#pragma HLS interface mode = axis port = out_app_reader
    static hls_uint<3> state = RTPS_SBM_DATA_IN_STATE_SBM_HDR;
    static uint16_t    offset = 0;
    static bool        sbm_le;
    static bool        sbm_inline_qos;

    static uint16_t param_id;
    static uint16_t length;

    hls_uint<10> x = in.read();
    uint8_t      data = x & 0xff;
    bool         end = x & 0x100;
    bool         valid = x & 0x200;

    bool in_inline_qos = false;
    bool remove_endpoint = false;

    if (valid) {
        switch (state) {
        case RTPS_SBM_DATA_IN_STATE_SBM_HDR:
            if (offset == SBM_HDR_OFFSET_FLAGS) {
                sbm_le = data & SBM_FLAGS_ENDIANNESS;
                sbm_inline_qos = data & SBM_FLAGS_INLINE_QOS;
            }
            offset++;
            if (offset == SBM_HDR_SIZE) {
                offset = 0;
                state = RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR;
            }
            break;
        case RTPS_SBM_DATA_IN_STATE_SBM_DATA_HDR:
            if (offset == SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS) {
                length = sbm_le ? data : (data << 8);
            } else if (offset
                       == (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 1)) {
                length |= sbm_le ? (data << 8) : data;
            }
            if (offset >= SBM_DATA_HDR_SIZE) {
                in_inline_qos = true;
            }
            offset++;
            if ((offset >= (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 2))
                && (offset
                    == (SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS + 2
                        + length))) {
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
            in_inline_qos = true;
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
            in_inline_qos = true;
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
                remove_endpoint = true;
            }
            in_inline_qos = true;
            offset++;
            if (offset == length) {
                offset = 0;
                state = RTPS_SBM_DATA_IN_STATE_INLINE_QOS_PARAM_HDR;
            }
            break;
        }
    }

    if (end) {
        state = RTPS_SBM_DATA_IN_STATE_SBM_HDR;
        offset = 0;
    }

    hls_uint<10> y = data;
    if (end) {
        y |= 0x100;
    }
    if (valid && !in_inline_qos) {
        y |= 0x200;
    }
    out_status_info.write(remove_endpoint);
    out_spdp_reader.write(y);
    out_sedp_reader.write(y);
    out_app_reader.write(y);
}
