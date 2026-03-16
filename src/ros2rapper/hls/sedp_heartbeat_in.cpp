#include "sedp_heartbeat_in.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

void sedp_heartbeat_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                       const uint8_t guid_prefix[GUID_PREFIX_SIZE],
                       uint8_t sbm_flags, uint16_t sbm_len) {
#pragma HLS inline
    static const uint8_t pub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
#pragma HLS array_partition variable = pub_reader_id complete dim = 0
    static const uint8_t sub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = sub_reader_id complete dim = 0

    static uint16_t          offset;
    static builtin_ep_type_t ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    static uint8_t           sbm_sn_0;
    static uint8_t           sbm_sn_1;

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;
    bool    sbm_le = sbm_flags & SBM_FLAGS_ENDIANNESS;

    if (!rtps_compare_heartbeat_hdr_reader_id(offset, data, pub_reader_id)) {
        ep_type &= ~(BUILTIN_EP_PUB);
    }
    if (!rtps_compare_heartbeat_hdr_reader_id(offset, data, sub_reader_id)) {
        ep_type &= ~(BUILTIN_EP_SUB);
    }
    // Ignore other than lower 8-bit of Sequence Number to reduce resources
    if (sbm_le && (offset == SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 4)) {
        sbm_sn_0 = data;
    } else if (!sbm_le && (offset == SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 7)) {
        sbm_sn_0 = data;
    } else if (sbm_le && (offset == SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 4)) {
        sbm_sn_1 = data;
    } else if (!sbm_le && (offset == SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 7)) {
        sbm_sn_1 = data;
    }

    offset++;
    if (offset == sbm_len) {
        rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
        /* Cyber unroll_times=all */
        for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
            rtps_data.guid_prefix[j] = guid_prefix[j];
        }
        rtps_data.data[0] = sbm_sn_0;
        rtps_data.data[1] = sbm_sn_1;
        if (ep_type & BUILTIN_EP_PUB) {
            rtps_data.type = RTPS_TYPE_SEDP_HEARTBEAT_PUB;
            out.write(rtps_data);
        } else if (ep_type & BUILTIN_EP_SUB) {
            rtps_data.type = RTPS_TYPE_SEDP_HEARTBEAT_SUB;
            out.write(rtps_data);
        }
    }

    if (end) {
        offset = 0;
        ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    }
}
