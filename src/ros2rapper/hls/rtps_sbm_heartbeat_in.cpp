#include "rtps_sbm_heartbeat_in.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include <cstdint>

void rtps_sbm_heartbeat_in(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                           bool          sbm_le,
                           const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    static const uint8_t pub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
#pragma HLS array_partition variable = pub_reader_id complete dim = 0
    static const uint8_t sub_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable = sub_reader_id complete dim = 0

    static uint16_t          offset = 0;
    static builtin_ep_type_t ep_type = BUILTIN_EP_PUB | BUILTIN_EP_SUB;
    static uint8_t           first_sn;
    static uint8_t           last_sn;

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;

    if (!rtps_compare_heartbeat_hdr_reader_id(offset, data, pub_reader_id)) {
        ep_type &= ~(BUILTIN_EP_PUB);
    }
    if (!rtps_compare_heartbeat_hdr_reader_id(offset, data, sub_reader_id)) {
        ep_type &= ~(BUILTIN_EP_SUB);
    }
    // Ignore other than lower 8-bit of Sequence Number to reduce resources
    if (sbm_le && (offset == (SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 4))) {
        first_sn = data;
    } else if (!sbm_le
               && (offset == (SBM_HEARTBEAT_DATA_OFFSET_FIRST_SN + 7))) {
        first_sn = data;
    } else if (sbm_le && (offset == (SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 4))) {
        last_sn = data;
    } else if (!sbm_le && (offset == (SBM_HEARTBEAT_DATA_OFFSET_LAST_SN + 7))) {
        last_sn = data;
    }

    offset++;
    if (offset == SBM_HEARTBEAT_DATA_SIZE) {
        rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
        /* Cyber unroll_times=all */
        for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
            rtps_data.guid_prefix[j] = src_guid_prefix[j];
        }
        rtps_data.data[0] = first_sn;
        rtps_data.data[1] = last_sn;
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
