// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "app.hpp"
#include "common.hpp"
#include "ros2.hpp"

#define APP_HDR_SIZE APP_TOT_LEN(0)

/* Cyber func=inline */
void app_writer(const uint8_t vendor_id[2],
                const uint8_t writer_guid_prefix[12],
                const uint8_t writer_entity_id[4],
                const uint8_t reader_guid_prefix[12],
                const uint8_t reader_entity_id[4], const int64_t seqnum,
#ifdef PUB_DATA_FF
                VOLATILE
#endif // PUB_DATA_FF
                const uint32_t app_data[MAX_APP_DATA_LEN / 4],
                uint32_t app_data_len, hls_stream<uint8_t> &out,
                timestamp now) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t  sbm_flags = SBM_FLAGS_ENDIANNESS;
    static const uint16_t rep_id = SP_ID_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t  sbm_flags = 0;
    static const uint16_t rep_id = SP_ID_CDR_BE;
#endif // SBM_ENDIAN_BIG

    static const uint16_t ext_flags = 0;
    static const uint16_t rep_opt = 0;

    const uint16_t tot_len = APP_TOT_LEN(app_data_len);
    const uint16_t octets_to_next_header
        = APP_OCTETS_TO_NEXT_HEADER(app_data_len);

    int32_t  seqnum_h = seqnum >> 32;
    uint32_t seqnum_l = seqnum & 0xffffffff;

    out.write('R');
    out.write('T');
    out.write('P');
    out.write('S');
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(vendor_id[0]);
    out.write(vendor_id[1]);
    out.write(writer_guid_prefix[0]);
    out.write(writer_guid_prefix[1]);
    out.write(writer_guid_prefix[2]);
    out.write(writer_guid_prefix[3]);
    out.write(writer_guid_prefix[4]);
    out.write(writer_guid_prefix[5]);
    out.write(writer_guid_prefix[6]);
    out.write(writer_guid_prefix[7]);
    out.write(writer_guid_prefix[8]);
    out.write(writer_guid_prefix[9]);
    out.write(writer_guid_prefix[10]);
    out.write(writer_guid_prefix[11]);
    out.write(SBM_ID_INFO_DST);
    out.write(sbm_flags);
    out.write(S_BYTE0(GUID_PREFIX_SIZE));
    out.write(S_BYTE1(GUID_PREFIX_SIZE));
    out.write(reader_guid_prefix[0]);
    out.write(reader_guid_prefix[1]);
    out.write(reader_guid_prefix[2]);
    out.write(reader_guid_prefix[3]);
    out.write(reader_guid_prefix[4]);
    out.write(reader_guid_prefix[5]);
    out.write(reader_guid_prefix[6]);
    out.write(reader_guid_prefix[7]);
    out.write(reader_guid_prefix[8]);
    out.write(reader_guid_prefix[9]);
    out.write(reader_guid_prefix[10]);
    out.write(reader_guid_prefix[11]);
    out.write(SBM_ID_INFO_TS);
    out.write(sbm_flags);
    out.write(S_BYTE0(TIMESTAMP_SIZE));
    out.write(S_BYTE1(TIMESTAMP_SIZE));
    out.write(L_BYTE0(now.seconds));
    out.write(L_BYTE1(now.seconds));
    out.write(L_BYTE2(now.seconds));
    out.write(L_BYTE3(now.seconds));
    out.write(L_BYTE0(now.fraction));
    out.write(L_BYTE1(now.fraction));
    out.write(L_BYTE2(now.fraction));
    out.write(L_BYTE3(now.fraction));
    out.write(SBM_ID_DATA);
    out.write(sbm_flags | SBM_FLAGS_DATA);
    out.write(S_BYTE0(octets_to_next_header));
    out.write(S_BYTE1(octets_to_next_header));
    out.write(ext_flags >> 8);
    out.write(ext_flags & 0xff);
    out.write(S_BYTE0(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS));
    out.write(S_BYTE1(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS));
    out.write(reader_entity_id[0]);
    out.write(reader_entity_id[1]);
    out.write(reader_entity_id[2]);
    out.write(reader_entity_id[3]);
    out.write(writer_entity_id[0]);
    out.write(writer_entity_id[1]);
    out.write(writer_entity_id[2]);
    out.write(writer_entity_id[3]);
    out.write(L_BYTE0(seqnum_h));
    out.write(L_BYTE1(seqnum_h));
    out.write(L_BYTE2(seqnum_h));
    out.write(L_BYTE3(seqnum_h));
    out.write(L_BYTE0(seqnum_l));
    out.write(L_BYTE1(seqnum_l));
    out.write(L_BYTE2(seqnum_l));
    out.write(L_BYTE3(seqnum_l));
    out.write(rep_id >> 8);
    out.write(rep_id & 0xff);
    out.write(rep_opt >> 8);
    out.write(rep_opt & 0xff);

#ifdef PUB_DATA_FF
    /* Cyber unroll_times=all */
#else  // !PUB_DATA_FF
    /* Cyber folding=4 */
#endif // PUB_DATA_FF
    for (auto j = 0; j < (MAX_APP_DATA_LEN / 4); j++) {
#ifdef PUB_DATA_FF
#pragma HLS unroll
#else // !PUB_DATA_FF
#pragma HLS pipeline II = 4
#endif // PUB_DATA_FF
        if ((4 * j) >= app_data_len) {
            break;
        }
        uint32_t data = app_data[j];
        out.write(data & 0xff);
        out.write((data >> 8) & 0xff);
        out.write((data >> 16) & 0xff);
        out.write(data >> 24);
    }
}
