#include "sedp_heartbeat_in.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include <cstdint>

#define SEDP_HEARTBEAT_IN_STATE_SBM_HDR     0
#define SEDP_HEARTBEAT_IN_STATE_SBM_PAYLOAD 1

void sedp_heartbeat_in(hls_stream<hls_uint<10>>        &in,
                       hls_stream<sedp_heartbeat_in_t> &out) {

#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
    static const uint8_t pub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
#pragma HLS array_partition variable = pub_reader_id complete dim = 0
    static const uint8_t sub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = sub_reader_id complete dim = 0

    static hls_uint<1> state = SEDP_HEARTBEAT_IN_STATE_SBM_HDR;
    static uint16_t    offset = 0;
    static bool        sbm_le;

    static builtin_ep_type_t ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    static uint8_t           first_sn;
    static uint8_t           last_sn;

    hls_uint<10> x = in.read();
    uint8_t      data = x & 0xff;
    bool         end = x & 0x100;
    bool         valid = x & 0x200;

    sedp_heartbeat_in_t out_data;
    out_data.ep_type = 0;

    if (valid) {
        switch (state) {
        case SEDP_HEARTBEAT_IN_STATE_SBM_HDR:
            if (offset == SBM_HDR_OFFSET_FLAGS) {
                sbm_le = data & SBM_FLAGS_ENDIANNESS;
            }
            offset++;
            if (offset == SBM_HDR_SIZE) {
                offset = 0;
                state = SEDP_HEARTBEAT_IN_STATE_SBM_PAYLOAD;
            }
            break;
        case SEDP_HEARTBEAT_IN_STATE_SBM_PAYLOAD:
            if (!rtps_compare_heartbeat_hdr_reader_id(offset, data,
                                                      pub_reader_id)) {
                ep_type &= ~(BUILTIN_EP_PUB);
            }
            if (!rtps_compare_heartbeat_hdr_reader_id(offset, data,
                                                      sub_reader_id)) {
                ep_type &= ~(BUILTIN_EP_SUB);
            }
            // Ignore other than lower 8-bit of Sequence Number to reduce
            // resources
            if (sbm_le
                && (offset == (SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 4))) {
                first_sn = data;
            } else if (!sbm_le
                       && (offset
                           == (SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 7))) {
                first_sn = data;
            } else if (sbm_le
                       && (offset == (SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 4))) {
                last_sn = data;
            } else if (!sbm_le
                       && (offset == (SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 7))) {
                last_sn = data;
            }
            offset++;
            if (offset == SBM_HEARTBEAT_DATA_SIZE) {
                out_data.ep_type = ep_type;
            }
            break;
        }
    }

    if (end) {
        state = SEDP_HEARTBEAT_IN_STATE_SBM_HDR;
        offset = 0;
        ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    }

    out_data.first_sn = first_sn;
    out_data.last_sn = last_sn;
    out.write(out_data);
}
