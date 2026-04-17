// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "sedp.hpp"
#include "common.hpp"
#include "duration.hpp"
#include "endpoint.hpp"
#include "ip.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include "spdp.hpp"

/* Cyber func=inline */
void sedp_writer(
    const uint8_t vendor_id[2], const uint8_t writer_guid_prefix[12],
    const uint8_t writer_entity_id[4], const uint8_t reader_guid_prefix[12],
    const uint8_t reader_entity_id[4], int64_t seqnum,
    const uint8_t usertraffic_addr[4], const uint8_t usertraffic_port[2],
    const uint8_t app_entity_id[4], hls_stream<uint8_t> &out,
    const uint8_t topic_name[MAX_TOPIC_NAME_LEN], uint8_t topic_name_len,
    const uint8_t type_name[MAX_TOPIC_TYPE_NAME_LEN], uint8_t type_name_len,
    timestamp now) {
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
        = SEDP_WRITER_OCTETS_TO_NEXT_HEADER;

    static const uint8_t participant_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_PARTICIPANT;
#pragma HLS array_partition variable = participant_entity_id complete dim = 0

    uint16_t pid_topic_name_size = SP_STR_DATA_SIZE(topic_name_len);

    uint16_t pid_type_name_size = SP_STR_DATA_SIZE(type_name_len);

    static const uint32_t type_max_size_serialized = 4 + MAX_APP_DATA_LEN;
    static const uint32_t durability_qos = 0;
    static const duration deadline = DURATION_INFINITE;
    static const duration latency_budget = DURATION_ZERO;
    static const uint32_t liveliness_qos = 0;
    static const duration liveliness_duration = DURATION_INFINITE;
    static const uint32_t reliability_qos = 1;
    static const duration reliability_duration
        = {0, (uint64_t)((0.1 * (1ULL << 32)) + 0.5)}; // 100 ms
    static const duration lifespan = DURATION_INFINITE;
    static const uint32_t ownership = 0;
    static const uint32_t ownership_strength = 0;
    static const uint32_t destination_order = 0;

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
    out.write(S_BYTE0(PID_UNICAST_LOCATOR));
    out.write(S_BYTE1(PID_UNICAST_LOCATOR));
    out.write(S_BYTE0(PID_UNICAST_LOCATOR_SIZE));
    out.write(S_BYTE1(PID_UNICAST_LOCATOR_SIZE));
    out.write(L_BYTE0(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE1(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE2(LOCATOR_KIND_UDPv4));
    out.write(L_BYTE3(LOCATOR_KIND_UDPv4));
#ifdef SBM_ENDIAN_LITTLE
    out.write(usertraffic_port[1]);
    out.write(usertraffic_port[0]);
    out.write(0);
    out.write(0);
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    out.write(0);
    out.write(0);
    out.write(usertraffic_port[0]);
    out.write(usertraffic_port[1]);
#endif // SBM_ENDIAN_BIG
    /* Cyber unroll_times=all */
    for (auto j = 0; j < 12; j++) {
#pragma HLS unroll
        out.write(0);
    }
    out.write(usertraffic_addr[0]);
    out.write(usertraffic_addr[1]);
    out.write(usertraffic_addr[2]);
    out.write(usertraffic_addr[3]);
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
    out.write(S_BYTE0(PID_TOPIC_NAME));
    out.write(S_BYTE1(PID_TOPIC_NAME));
    out.write(S_BYTE0(pid_topic_name_size));
    out.write(S_BYTE1(pid_topic_name_size));
    out.write(L_BYTE0(topic_name_len));
    out.write(L_BYTE1(topic_name_len));
    out.write(L_BYTE2(topic_name_len));
    out.write(L_BYTE3(topic_name_len));
    /* Cyber unroll_times=all */
    for (auto j = 0; j < MAX_TOPIC_NAME_LEN; j++) {
#pragma HLS unroll
        out.write(topic_name[j]);
    }
    out.write(S_BYTE0(PID_TYPE_NAME));
    out.write(S_BYTE1(PID_TYPE_NAME));
    out.write(S_BYTE0(pid_type_name_size));
    out.write(S_BYTE1(pid_type_name_size));
    out.write(L_BYTE0(type_name_len));
    out.write(L_BYTE1(type_name_len));
    out.write(L_BYTE2(type_name_len));
    out.write(L_BYTE3(type_name_len));
    /* Cyber unroll_times=all */
    for (auto j = 0; j < MAX_TOPIC_TYPE_NAME_LEN; j++) {
#pragma HLS unroll
        out.write(type_name[j]);
    }
    out.write(S_BYTE0(PID_KEY_HASH));
    out.write(S_BYTE1(PID_KEY_HASH));
    out.write(S_BYTE0(PID_KEY_HASH_SIZE));
    out.write(S_BYTE1(PID_KEY_HASH_SIZE));
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
    out.write(app_entity_id[0]);
    out.write(app_entity_id[1]);
    out.write(app_entity_id[2]);
    out.write(app_entity_id[3]);
    out.write(S_BYTE0(PID_ENDPOINT_GUID));
    out.write(S_BYTE1(PID_ENDPOINT_GUID));
    out.write(S_BYTE0(PID_ENDPOINT_GUID_SIZE));
    out.write(S_BYTE1(PID_ENDPOINT_GUID_SIZE));
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
    out.write(app_entity_id[0]);
    out.write(app_entity_id[1]);
    out.write(app_entity_id[2]);
    out.write(app_entity_id[3]);
    out.write(S_BYTE0(PID_TYPE_MAX_SIZE_SERIALIZED));
    out.write(S_BYTE1(PID_TYPE_MAX_SIZE_SERIALIZED));
    out.write(S_BYTE0(PID_TYPE_MAX_SIZE_SERIALIZED_SIZE));
    out.write(S_BYTE1(PID_TYPE_MAX_SIZE_SERIALIZED_SIZE));
    out.write(L_BYTE0(type_max_size_serialized));
    out.write(L_BYTE1(type_max_size_serialized));
    out.write(L_BYTE2(type_max_size_serialized));
    out.write(L_BYTE3(type_max_size_serialized));
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
    out.write(S_BYTE0(PID_DURABILITY));
    out.write(S_BYTE1(PID_DURABILITY));
    out.write(S_BYTE0(PID_DURABILITY_SIZE));
    out.write(S_BYTE1(PID_DURABILITY_SIZE));
    out.write(L_BYTE0(durability_qos));
    out.write(L_BYTE1(durability_qos));
    out.write(L_BYTE2(durability_qos));
    out.write(L_BYTE3(durability_qos));
    out.write(S_BYTE0(PID_DEADLINE));
    out.write(S_BYTE1(PID_DEADLINE));
    out.write(S_BYTE0(PID_DEADLINE_SIZE));
    out.write(S_BYTE1(PID_DEADLINE_SIZE));
    out.write(L_BYTE0(deadline.seconds));
    out.write(L_BYTE1(deadline.seconds));
    out.write(L_BYTE2(deadline.seconds));
    out.write(L_BYTE3(deadline.seconds));
    out.write(L_BYTE0(deadline.fraction));
    out.write(L_BYTE1(deadline.fraction));
    out.write(L_BYTE2(deadline.fraction));
    out.write(L_BYTE3(deadline.fraction));
    out.write(S_BYTE0(PID_LATENCY_BUDGET));
    out.write(S_BYTE1(PID_LATENCY_BUDGET));
    out.write(S_BYTE0(PID_LATENCY_BUDGET_SIZE));
    out.write(S_BYTE1(PID_LATENCY_BUDGET_SIZE));
    out.write(L_BYTE0(latency_budget.seconds));
    out.write(L_BYTE1(latency_budget.seconds));
    out.write(L_BYTE2(latency_budget.seconds));
    out.write(L_BYTE3(latency_budget.seconds));
    out.write(L_BYTE0(latency_budget.fraction));
    out.write(L_BYTE1(latency_budget.fraction));
    out.write(L_BYTE2(latency_budget.fraction));
    out.write(L_BYTE3(latency_budget.fraction));
    out.write(S_BYTE0(PID_LIVELINESS));
    out.write(S_BYTE1(PID_LIVELINESS));
    out.write(S_BYTE0(PID_LIVELINESS_SIZE));
    out.write(S_BYTE1(PID_LIVELINESS_SIZE));
    out.write(L_BYTE0(liveliness_qos));
    out.write(L_BYTE1(liveliness_qos));
    out.write(L_BYTE2(liveliness_qos));
    out.write(L_BYTE3(liveliness_qos));
    out.write(L_BYTE0(liveliness_duration.seconds));
    out.write(L_BYTE1(liveliness_duration.seconds));
    out.write(L_BYTE2(liveliness_duration.seconds));
    out.write(L_BYTE3(liveliness_duration.seconds));
    out.write(L_BYTE0(liveliness_duration.fraction));
    out.write(L_BYTE1(liveliness_duration.fraction));
    out.write(L_BYTE2(liveliness_duration.fraction));
    out.write(L_BYTE3(liveliness_duration.fraction));
    out.write(S_BYTE0(PID_RELIABILITY));
    out.write(S_BYTE1(PID_RELIABILITY));
    out.write(S_BYTE0(PID_RELIABILITY_SIZE));
    out.write(S_BYTE1(PID_RELIABILITY_SIZE));
    out.write(L_BYTE0(reliability_qos));
    out.write(L_BYTE1(reliability_qos));
    out.write(L_BYTE2(reliability_qos));
    out.write(L_BYTE3(reliability_qos));
    out.write(L_BYTE0(reliability_duration.seconds));
    out.write(L_BYTE1(reliability_duration.seconds));
    out.write(L_BYTE2(reliability_duration.seconds));
    out.write(L_BYTE3(reliability_duration.seconds));
    out.write(L_BYTE0(reliability_duration.fraction));
    out.write(L_BYTE1(reliability_duration.fraction));
    out.write(L_BYTE2(reliability_duration.fraction));
    out.write(L_BYTE3(reliability_duration.fraction));
    out.write(S_BYTE0(PID_LIFESPAN));
    out.write(S_BYTE1(PID_LIFESPAN));
    out.write(S_BYTE0(PID_LIFESPAN_SIZE));
    out.write(S_BYTE1(PID_LIFESPAN_SIZE));
    out.write(L_BYTE0(lifespan.seconds));
    out.write(L_BYTE1(lifespan.seconds));
    out.write(L_BYTE2(lifespan.seconds));
    out.write(L_BYTE3(lifespan.seconds));
    out.write(L_BYTE0(lifespan.fraction));
    out.write(L_BYTE1(lifespan.fraction));
    out.write(L_BYTE2(lifespan.fraction));
    out.write(L_BYTE3(lifespan.fraction));
    out.write(S_BYTE0(PID_OWNERSHIP));
    out.write(S_BYTE1(PID_OWNERSHIP));
    out.write(S_BYTE0(PID_OWNERSHIP_SIZE));
    out.write(S_BYTE1(PID_OWNERSHIP_SIZE));
    out.write(L_BYTE0(ownership));
    out.write(L_BYTE1(ownership));
    out.write(L_BYTE2(ownership));
    out.write(L_BYTE3(ownership));
    out.write(S_BYTE0(PID_OWNERSHIP_STRENGTH));
    out.write(S_BYTE1(PID_OWNERSHIP_STRENGTH));
    out.write(S_BYTE0(PID_OWNERSHIP_STRENGTH_SIZE));
    out.write(S_BYTE1(PID_OWNERSHIP_STRENGTH_SIZE));
    out.write(L_BYTE0(ownership_strength));
    out.write(L_BYTE1(ownership_strength));
    out.write(L_BYTE2(ownership_strength));
    out.write(L_BYTE3(ownership_strength));
    out.write(S_BYTE0(PID_DESTINATION_ORDER));
    out.write(S_BYTE1(PID_DESTINATION_ORDER));
    out.write(S_BYTE0(PID_DESTINATION_ORDER_SIZE));
    out.write(S_BYTE1(PID_DESTINATION_ORDER_SIZE));
    out.write(L_BYTE0(destination_order));
    out.write(L_BYTE1(destination_order));
    out.write(L_BYTE2(destination_order));
    out.write(L_BYTE3(destination_order));
    out.write(S_BYTE0(PID_SENTINEL));
    out.write(S_BYTE1(PID_SENTINEL));
    out.write(0); // PID_SENTINEL_SIZE
    out.write(0); // PID_SENTINEL_SIZE
}

/* Cyber func=inline */
void sedp_heartbeat(const uint8_t vendor_id[2],
                    const uint8_t writer_guid_prefix[12],
                    const uint8_t writer_entity_id[4],
                    const uint8_t reader_guid_prefix[12],
                    const uint8_t reader_entity_id[4],
                    const int64_t first_seqnum, const int64_t last_seqnum,
                    const uint32_t cnt, hls_stream<uint8_t> &out) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t sbm_flags = 0;
#endif // SBM_ENDIAN_BIG

    int32_t  first_seqnum_h = first_seqnum >> 32;
    uint32_t first_seqnum_l = first_seqnum & 0xffffffff;
    int32_t  last_seqnum_h = last_seqnum >> 32;
    uint32_t last_seqnum_l = last_seqnum & 0xffffffff;

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
    out.write(SBM_ID_HEARTBEAT);
    out.write(sbm_flags | SBM_FLAGS_FINAL);
    out.write(S_BYTE0(SBM_HEARTBEAT_DATA_SIZE));
    out.write(S_BYTE1(SBM_HEARTBEAT_DATA_SIZE));
    out.write(reader_entity_id[0]);
    out.write(reader_entity_id[1]);
    out.write(reader_entity_id[2]);
    out.write(reader_entity_id[3]);
    out.write(writer_entity_id[0]);
    out.write(writer_entity_id[1]);
    out.write(writer_entity_id[2]);
    out.write(writer_entity_id[3]);
    out.write(L_BYTE0(first_seqnum_h));
    out.write(L_BYTE1(first_seqnum_h));
    out.write(L_BYTE2(first_seqnum_h));
    out.write(L_BYTE3(first_seqnum_h));
    out.write(L_BYTE0(first_seqnum_l));
    out.write(L_BYTE1(first_seqnum_l));
    out.write(L_BYTE2(first_seqnum_l));
    out.write(L_BYTE3(first_seqnum_l));
    out.write(L_BYTE0(last_seqnum_h));
    out.write(L_BYTE1(last_seqnum_h));
    out.write(L_BYTE2(last_seqnum_h));
    out.write(L_BYTE3(last_seqnum_h));
    out.write(L_BYTE0(last_seqnum_l));
    out.write(L_BYTE1(last_seqnum_l));
    out.write(L_BYTE2(last_seqnum_l));
    out.write(L_BYTE3(last_seqnum_l));
    out.write(L_BYTE0(cnt));
    out.write(L_BYTE1(cnt));
    out.write(L_BYTE2(cnt));
    out.write(L_BYTE3(cnt));
}

/* Cyber func=inline */
void sedp_acknack(const uint8_t vendor_id[2],
                  const uint8_t writer_guid_prefix[12],
                  const uint8_t writer_entity_id[4],
                  const uint8_t reader_guid_prefix[12],
                  const uint8_t reader_entity_id[4], uint8_t snstate_base,
                  bool snstate_is_empty, const uint32_t cnt,
                  hls_stream<uint8_t> &out) {
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
    static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
    static const uint8_t sbm_flags = 0;
#endif // SBM_ENDIAN_BIG

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
    out.write(SBM_ID_ACKNACK);
    out.write(sbm_flags | SBM_FLAGS_FINAL);
    out.write(S_BYTE0(SBM_ACKNACK_DATA_SIZE));
    out.write(S_BYTE1(SBM_ACKNACK_DATA_SIZE));
    out.write(reader_entity_id[0]);
    out.write(reader_entity_id[1]);
    out.write(reader_entity_id[2]);
    out.write(reader_entity_id[3]);
    out.write(writer_entity_id[0]);
    out.write(writer_entity_id[1]);
    out.write(writer_entity_id[2]);
    out.write(writer_entity_id[3]);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(snstate_base);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(1);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(0);
    out.write(
        snstate_is_empty
            ? 0x00
            : 0x80); // Reporting missing seqnum by ACKNACK is done one by one.
    out.write(L_BYTE0(cnt));
    out.write(L_BYTE1(cnt));
    out.write(L_BYTE2(cnt));
    out.write(L_BYTE3(cnt));
}
