// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "duration.hpp"
#include "ip.hpp"
#include "ros2_receiver.hpp"
#include "spdp.hpp"

/* Cyber func=inline */
void reset_sedp_endpoint_children(bool unmatched[APP_READER_MAX]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
        unmatched[j] = false;
    }
}

#define FLAGS_FOUND_GUID     0x01
#define FLAGS_FOUND_LOCATOR  0x02
#define FLAGS_UNMATCH_DOMAIN 0x04

/* Cyber func=inline */
void spdp_reader(hls_uint<9> in, hls_stream<rtps_data_t> &out,
                 hls_uint<1> enable, const uint8_t ip_addr[4],
                 const uint8_t subnet_mask[4], uint16_t port_num_seed) {
#pragma HLS inline
    static const uint8_t par_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PARTICIPANT_READER;
#pragma HLS array_partition variable = par_reader_id complete dim = 0

    static hls_uint<4> state;
    static uint16_t    offset;
    static hls_uint<3> flags;

    static uint8_t  sbm_id;
    static bool     sbm_le;
    static uint16_t sbm_len;
    static uint16_t rep_id;
    static uint16_t param_id;
    static uint16_t param_len;

    static bool lease_duration_found;

    static uint8_t spdp_guid_prefix
        [GUID_PREFIX_SIZE] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = spdp_guid_prefix complete dim = 1
    static uint8_t spdp_ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = spdp_ip_addr complete dim = 1
    static uint8_t spdp_udp_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = spdp_udp_port complete dim = 1
    static uint8_t
        spdp_lease_duration[8] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = spdp_lease_duration complete dim = 1

    if (!enable) {
        return;
    }

    uint8_t data = in & 0xff;
    bool    end = in & 0x100;

    switch (state) {
    case 0:
        if (!rtps_compare_protocol(offset, data)) {
            state = 8;
            break;
        }
        offset++;
        if (offset == RTPS_HDR_SIZE) {
            offset = 0;
            state = 1;
        }
        break;
    case 1:
        switch (offset) {
        case SBM_HDR_OFFSET_SUBMESSAGE_ID:
            sbm_id = data;
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_le = data & SBM_FLAGS_ENDIANNESS ? true : false;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER:
            sbm_len = sbm_le ? data : data << 8;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER + 1:
            sbm_len |= sbm_le ? data << 8 : data;
        }
        offset++;
        if (offset == SBM_HDR_SIZE) {
            offset = 0;
            if (sbm_id == SBM_ID_DATA)
                state = 2;
            else
                state = 7;
        }
        break;
    case 2:
        if (!rtps_compare_data_hdr_reader_id(offset, data, par_reader_id)) {
            state = 8;
            break;
        }
        offset++;
        if (offset == SBM_DATA_HDR_SIZE) {
            sbm_len -= SBM_DATA_HDR_SIZE;
            offset = 0;
            state = 3;
        }
        break;
    case 3:
        switch (offset) {
        case SP_HDR_OFFSET_REPRESENTATION_ID:
            rep_id = data << 8;
            break;
        case SP_HDR_OFFSET_REPRESENTATION_ID + 1:
            rep_id |= data;
        }
        offset++;
        if (offset == SP_HDR_SIZE) {
            sbm_len -= SP_HDR_SIZE;
            offset = 0;
            state = 4;
        }
        break;
    case 4:
        if (offset == 0)
            param_id = rep_id & SP_ID_CDR_LE ? data : data << 8;
        else
            param_id |= rep_id & SP_ID_CDR_LE ? data << 8 : data;
        offset++;
        if (offset == sizeof(param_id)) {
            sbm_len -= sizeof(param_id);
            offset = 0;
            state = 5;
        }
        break;
    case 5:
        if (offset == 0)
            param_len = rep_id & SP_ID_CDR_LE ? data : data << 8;
        else
            param_len |= rep_id & SP_ID_CDR_LE ? data << 8 : data;
        offset++;
        if (offset == sizeof(param_len)) {
            if (param_id == PID_SENTINEL) {
                hls_uint<3> found = FLAGS_FOUND_GUID | FLAGS_FOUND_LOCATOR;
                if (flags == found) {
                    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
                    rtps_data.type = RTPS_TYPE_SPDP;
                    /* Cyber unroll_times=all */
                    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
                        rtps_data.guid_prefix[j] = spdp_guid_prefix[j];
                    }
                    /* Cyber unroll_times=all */
                    for (auto j = 0; j < 4; j++) {
#pragma HLS unroll
                        rtps_data.data[j] = spdp_ip_addr[j];
                    }
                    rtps_data.data[4] = spdp_udp_port[0];
                    rtps_data.data[5] = spdp_udp_port[1];
                    if (lease_duration_found) {
                        /* Cyber unroll_times=all */
                        for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
                            rtps_data.data[j + 6] = spdp_lease_duration[j];
                        }
                    } else {
                        // Use default value 100 sec.
                        rtps_data.data[6] = 0;
                        rtps_data.data[7] = 0;
                        rtps_data.data[8] = 0;
                        rtps_data.data[9] = 0;
                        rtps_data.data[10] = 100;
                        rtps_data.data[11] = 0;
                        rtps_data.data[12] = 0;
                        rtps_data.data[13] = 0;
                    }
                    out.write(rtps_data);
                }
                flags = 0;
                offset = 0;
                lease_duration_found = false;
                state = 1;
            } else {
                sbm_len -= sizeof(param_len);
                param_len = ROUND_UP(param_len, 4);
                offset = 0;
                state = (param_len == 0) ? 4 : 6;
            }
        }
        break;
    case 6:
        switch (param_id) {
        case PID_PARTICIPANT_GUID:
            if (flags & FLAGS_FOUND_GUID)
                break;
            if (offset < 12) {
                spdp_guid_prefix[offset] = data;
            }
            break;
        case PID_METATRAFFIC_UNICAST_LOCATOR:
            if (flags & FLAGS_FOUND_LOCATOR)
                break;
            if (offset < 4) {
                ; // do nothing
            } else if (offset < 8) {
                if (rep_id & SP_ID_CDR_LE) {
                    if (offset == 4) {
                        spdp_udp_port[1] = data;
                    } else if (offset == 5) {
                        spdp_udp_port[0] = data;
                    }
                } else {
                    if (offset == 6) {
                        spdp_udp_port[0] = data;
                    } else if (offset == 7) {
                        spdp_udp_port[1] = data;
                    }
                }
            } else if (offset < 20) {
                ; // do nothing
            } else if (offset == 20) {
                spdp_ip_addr[0] = data;
            } else if (offset == 21) {
                spdp_ip_addr[1] = data;
            } else if (offset == 22) {
                spdp_ip_addr[2] = data;
            } else if (offset == 23) {
                spdp_ip_addr[3] = data;
            }
            break;
        case PID_PARTICIPANT_LEASE_DURATION:
            if (offset < 8) {
                if (rep_id & SP_ID_CDR_LE) {
                    if (offset < 4) {
                        // Read the seconds of the lease duration.
                        spdp_lease_duration[offset + 4] = data;
                    } else {
                        // Read the fractional part of the lease duration.
                        spdp_lease_duration[offset - 4] = data;
                    }
                } else {
                    spdp_lease_duration[7 - offset] = data;
                }
            }
            break;
        }
        offset++;
        if (offset == param_len) {
            if (param_id == PID_PARTICIPANT_GUID) {
                flags |= (hls_uint<3>)FLAGS_FOUND_GUID;
            } else if (param_id == PID_METATRAFFIC_UNICAST_LOCATOR) {
                uint16_t udp_port = (spdp_udp_port[0] << 8) | spdp_udp_port[1];
                if (udp_port >= port_num_seed
                    && udp_port - port_num_seed < DG) {
                    if (is_same_subnet(spdp_ip_addr, ip_addr, subnet_mask)) {
                        flags |= (hls_uint<3>)FLAGS_FOUND_LOCATOR;
                    }
                }
            } else if (param_id == PID_PARTICIPANT_LEASE_DURATION) {
                lease_duration_found = true;
            }
            offset = 0;
            state = 4;
        }
        break;
    case 7:
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = 1;
        }
        break;
    case 8:; // do nothing
    }

    if (end) {
        flags = 0;
        offset = 0;
        lease_duration_found = false;
        state = 0;
    }
}

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
