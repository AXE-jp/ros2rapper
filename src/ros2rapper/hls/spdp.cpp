// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "spdp.hpp"
#include "common.hpp"
#include "duration.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "ros2.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"

/* Cyber func=inline */
void spdp_writer(const uint8_t vendor_id[2],
                 const uint8_t writer_guid_prefix[12],
                 const uint8_t metatraffic_addr[4],
                 const uint8_t metatraffic_port[2],
                 const uint8_t default_addr[4], const uint8_t default_port[2],
                 duration lease_duration, hls_stream<uint8_t> &out,
                 const uint8_t entity_name[MAX_NODE_NAME_LEN],
                 uint8_t entity_name_len, timestamp now) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t  sbm_flags = SBM_FLAGS_ENDIANNESS;
    static const uint16_t rep_id = SP_ID_PL_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t  sbm_flags = 0;
    static const uint16_t rep_id = SP_ID_PL_CDR_BE;
#endif // SBM_ENDIAN_BIG

    static const uint16_t ext_flags = 0;
    static const uint16_t rep_opt = 0;

    static const uint16_t octets_to_next_header
        = SPDP_WRITER_OCTETS_TO_NEXT_HEADER;

    static const uint8_t writer_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PARTICIPANT_WRITER;
    static const uint8_t reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PARTICIPANT_READER;
    static const uint8_t participant_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_PARTICIPANT;
#pragma HLS array_partition variable = writer_entity_id complete dim = 0
#pragma HLS array_partition variable = reader_entity_id complete dim = 0
#pragma HLS array_partition variable = participant_entity_id complete dim = 0

    static const uint32_t endpoint_set
        = ENDPOINT_PARTICIPANT_ANNOUNCER | ENDPOINT_PARTICIPANT_DETECTOR
          | ENDPOINT_PUBLICATIONS_ANNOUNCER | ENDPOINT_PUBLICATIONS_DETECTOR
          | ENDPOINT_SUBSCRIPTIONS_ANNOUNCER | ENDPOINT_SUBSCRIPTIONS_DETECTOR;

    const uint16_t pid_entity_name_size = SP_STR_DATA_SIZE(entity_name_len);

    static const uint32_t user_data_len = 25;
    static const uint16_t pid_user_data_size = SP_STR_DATA_SIZE(user_data_len);

    static const int64_t seqnum = 1;

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
    out.write(S_BYTE0(PID_PROTOCOL_VERSION));
    out.write(S_BYTE1(PID_PROTOCOL_VERSION));
    out.write(S_BYTE0(PID_PROTOCOL_VERSION_SIZE));
    out.write(S_BYTE1(PID_PROTOCOL_VERSION_SIZE));
    out.write(RTPS_HDR_PROTOCOL_VERSION >> 8);
    out.write(RTPS_HDR_PROTOCOL_VERSION & 0xff);
    out.write(0); // padding
    out.write(0); // padding
    out.write(S_BYTE0(PID_VENDOR_ID));
    out.write(S_BYTE1(PID_VENDOR_ID));
    out.write(S_BYTE0(PID_VENDOR_ID_SIZE));
    out.write(S_BYTE1(PID_VENDOR_ID_SIZE));
    out.write(vendor_id[0]);
    out.write(vendor_id[1]);
    out.write(0); // padding
    out.write(0); // padding
    out.write(S_BYTE0(PID_PARTICIPANT_GUID));
    out.write(S_BYTE1(PID_PARTICIPANT_GUID));
    out.write(S_BYTE0(PID_PARTICIPANT_GUID_SIZE));
    out.write(S_BYTE1(PID_PARTICIPANT_GUID_SIZE));
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
    out.write(participant_entity_id[0]);
    out.write(participant_entity_id[1]);
    out.write(participant_entity_id[2]);
    out.write(participant_entity_id[3]);
    out.write(S_BYTE0(PID_METATRAFFIC_UNICAST_LOCATOR));
    out.write(S_BYTE1(PID_METATRAFFIC_UNICAST_LOCATOR));
    out.write(S_BYTE0(PID_METATRAFFIC_UNICAST_LOCATOR_SIZE));
    out.write(S_BYTE1(PID_METATRAFFIC_UNICAST_LOCATOR_SIZE));
    out.write(L_BYTE0(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE1(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE2(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE3(LOCATOR_KIND_UDPv4));
#ifdef SBM_ENDIAN_LITTLE
    out.write(metatraffic_port[1]);
    out.write(metatraffic_port[0]);
    out.write(0);
    out.write(0);
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    out.write(0);
    out.write(0);
    out.write(metatraffic_port[0]);
    out.write(metatraffic_port[1]);
#endif // SBM_ENDIAN_BIG
    /* Cyber unroll_times = all */
    for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
        out.write(0);
    }
    out.write(metatraffic_addr[0]);
    out.write(metatraffic_addr[1]);
    out.write(metatraffic_addr[2]);
    out.write(metatraffic_addr[3]);
    out.write(S_BYTE0(PID_DEFAULT_UNICAST_LOCATOR));
    out.write(S_BYTE1(PID_DEFAULT_UNICAST_LOCATOR));
    out.write(S_BYTE0(PID_DEFAULT_UNICAST_LOCATOR_SIZE));
    out.write(S_BYTE1(PID_DEFAULT_UNICAST_LOCATOR_SIZE));
    out.write(L_BYTE0(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE1(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE2(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE3(LOCATOR_KIND_UDPv4));
#ifdef SBM_ENDIAN_LITTLE
    out.write(default_port[1]);
    out.write(default_port[0]);
    out.write(0);
    out.write(0);
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    out.write(0);
    out.write(0);
    out.write(default_port[0]);
    out.write(default_port[1]);
#endif // SBM_ENDIAN_BIG
    /* Cyber unroll_times = all */
    for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
        out.write(0);
    }
    out.write(default_addr[0]);
    out.write(default_addr[1]);
    out.write(default_addr[2]);
    out.write(default_addr[3]);
    out.write(S_BYTE0(PID_PARTICIPANT_LEASE_DURATION));
    out.write(S_BYTE1(PID_PARTICIPANT_LEASE_DURATION));
    out.write(S_BYTE0(PID_PARTICIPANT_LEASE_DURATION_SIZE));
    out.write(S_BYTE1(PID_PARTICIPANT_LEASE_DURATION_SIZE));
    out.write(L_BYTE0(lease_duration.seconds));
    out.write(L_BYTE1(lease_duration.seconds));
    out.write(L_BYTE2(lease_duration.seconds));
    out.write(L_BYTE3(lease_duration.seconds));
    out.write(L_BYTE0(lease_duration.fraction));
    out.write(L_BYTE1(lease_duration.fraction));
    out.write(L_BYTE2(lease_duration.fraction));
    out.write(L_BYTE3(lease_duration.fraction));
    out.write(S_BYTE0(PID_BUILTIN_ENDPOINT_SET));
    out.write(S_BYTE1(PID_BUILTIN_ENDPOINT_SET));
    out.write(S_BYTE0(PID_BUILTIN_ENDPOINT_SET_SIZE));
    out.write(S_BYTE1(PID_BUILTIN_ENDPOINT_SET_SIZE));
    out.write(L_BYTE0(endpoint_set));
    out.write(L_BYTE1(endpoint_set));
    out.write(L_BYTE2(endpoint_set));
    out.write(L_BYTE3(endpoint_set));
    out.write(S_BYTE0(PID_ENTITY_NAME));
    out.write(S_BYTE1(PID_ENTITY_NAME));
    out.write(S_BYTE0(pid_entity_name_size));
    out.write(S_BYTE1(pid_entity_name_size));
    out.write(L_BYTE0(entity_name_len));
    out.write(L_BYTE1(entity_name_len));
    out.write(L_BYTE2(entity_name_len));
    out.write(L_BYTE3(entity_name_len));
    /* Cyber unroll_times = all */
    for (auto j = 0; j < MAX_NODE_NAME_LEN; j++) {
#pragma HLS unroll
        out.write(entity_name[j]);
    }
    // TODO: Store node and namespace name in user data (cf.
    // https://github.com/ros2/ros2/issues/438)
    /*
    out.write(S_BYTE0(PID_USER_DATA));
    out.write(S_BYTE1(PID_USER_DATA));
    out.write(S_BYTE0(pid_user_data_size));
    out.write(S_BYTE1(pid_user_data_size));
    out.write(L_BYTE0(user_data_len));
    out.write(L_BYTE1(user_data_len));
    out.write(L_BYTE2(user_data_len));
    out.write(L_BYTE3(user_data_len));
    out.write('n');
    out.write('a');
    out.write('m');
    out.write('e');
    out.write('=');
    out.write('t');
    out.write('a');
    out.write('l');
    out.write('k');
    out.write('e');
    out.write('r');
    out.write(';');
    out.write('n');
    out.write('a');
    out.write('m');
    out.write('e');
    out.write('s');
    out.write('p');
    out.write('a');
    out.write('c');
    out.write('e');
    out.write('=');
    out.write('/');
    out.write(';');
    out.write('\0');
    out.write(0); // padding
    out.write(0); // padding
    out.write(0); // padding
    */
    out.write(S_BYTE0(PID_SENTINEL));
    out.write(S_BYTE1(PID_SENTINEL));
    out.write(0); // PID_SENTINEL_SIZE
    out.write(0); // PID_SENTINEL_SIZE
}
