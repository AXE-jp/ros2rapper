/*
    Copyright © 2021-2022 AXE, Inc. All Rights Reserved.

    This file is part of ROS2rapper.

    ROS2rapper is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ROS2rapper is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with ROS2rapper.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "common.hpp"

#include "duration.hpp"
#include "spdp.hpp"

/* Cyber func=inline */
static void compare_guid_prefix(const uint8_t x,
				const sedp_endpoint tbl[SEDP_READER_MAX],
				const int idx,
				hls_uint<SEDP_READER_MAX> &unmatched)
{
#pragma HLS inline
	/* Cyber unroll_times=all */
	for (int i = 0; i < SEDP_READER_MAX; i++) {
#pragma HLS unroll
		if (tbl[i].guid_prefix[idx] != x)
			unmatched |= (hls_uint<SEDP_READER_MAX>)(0x1 << i);
	}
}

#define FLAGS_FOUND_GUID	0x01
#define FLAGS_FOUND_LOCATOR	0x02
#define FLAGS_UNMATCH_DOMAIN	0x04

/* Cyber func=inline */
void spdp_reader(hls_stream<hls_uint<9>> &in,
		 sedp_reader_id_t &reader_cnt,
		 sedp_endpoint reader_tbl[SEDP_READER_MAX])
{
#pragma HLS inline
	static const uint8_t par_reader_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_PARTICIPANT_READER;
#pragma HLS array_partition variable=par_reader_id complete dim=0

	static hls_uint<4> state;
	static uint16_t offset;
	static hls_uint<3> flags;
	static hls_uint<SEDP_READER_MAX> unmatched;

	static uint8_t sbm_id;
	static bool sbm_le;
	static uint16_t sbm_len;
	static uint16_t rep_id;
	static uint16_t param_id;
	static uint16_t param_len;
	static uint16_t udp_port;

	hls_uint<9> x;

	if (!in.read_nb(x))
		return;

	if (reader_cnt == SEDP_READER_MAX)
		return;

	sedp_endpoint &reader = reader_tbl[reader_cnt];
	uint8_t data = x & 0xff;
	bool end = x & 0x100;

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
		if (!rtps_compare_reader_id(offset, data, par_reader_id)) {
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
				hls_uint<3> found =
					FLAGS_FOUND_GUID | FLAGS_FOUND_LOCATOR;
				if (flags == found) {
					hls_uint<SEDP_READER_MAX> valid =
						(0x1 << reader_cnt) - 1;
					if ((unmatched & valid) == valid)
						reader_cnt++;
				}
				unmatched = 0;
				flags = 0;
				offset = 0;
				state = 1;
			} else {
				sbm_len -= sizeof(param_len);
				param_len = ROUND_UP(param_len, 4);
				offset = 0;
				state = 6;
			}
		}
		break;
	case 6:
		switch (param_id) {
		case PID_PARTICIPANT_GUID:
			if (flags & FLAGS_FOUND_GUID)
				break;
			if (offset < 12) {
				reader.guid_prefix[offset] = data;
				compare_guid_prefix(data,
						    reader_tbl,
						    offset,
						    unmatched);
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
						udp_port = data;
						reader.udp_port[1] = data;
					} else if (offset == 5) {
						udp_port |= data << 8;
						reader.udp_port[0] = data;
					}
				} else {
					if (offset == 6) {
						udp_port = data << 8;
						reader.udp_port[0] = data;
					} else if (offset == 7) {
						udp_port |= data;
						reader.udp_port[1] = data;
					}
				}
			} else if (offset < 20) {
				; // do nothing
			} else if (offset < 24) {
				reader.ip_addr[offset-20] = data;
			}
		}
		offset++;
		if (offset == param_len) {
			if (param_id == PID_PARTICIPANT_GUID) {
				flags |= (hls_uint<3>)FLAGS_FOUND_GUID;
			} else if (param_id ==
				   PID_METATRAFFIC_UNICAST_LOCATOR) {
				if (DOMAIN_ID(udp_port) != TARGET_DOMAIN_ID) {
					flags |= (hls_uint<3>)
						FLAGS_UNMATCH_DOMAIN;
				}
				flags |= (hls_uint<3>)FLAGS_FOUND_LOCATOR;
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
	case 8:
		; // do nothing
	}

	if (end) {
		unmatched = 0;
		flags = 0;
		offset = 0;
		state = 0;
	}
}

/* Cyber func=inline */
void spdp_writer(const uint8_t writer_guid_prefix[12],
		 const uint8_t metatraffic_addr[4],
		 const uint8_t metatraffic_port[2],
		 const uint8_t default_addr[4],
		 const uint8_t default_port[2],
		 uint8_t buf[SPDP_WRITER_TOT_LEN])
{
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
	static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
	static const uint16_t rep_id = SP_ID_PL_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	static const uint8_t sbm_flags = 0;
	static const uint16_t rep_id = SP_ID_PL_CDR_BE;
#endif // SBM_ENDIAN_BIG

	static const timestamp now = TIME_ZERO;
	static const uint16_t ext_flags = 0;
	static const uint16_t rep_opt = 0;

	static const uint16_t octets_to_next_header =
		SPDP_WRITER_OCTETS_TO_NEXT_HEADER;

	static const uint8_t writer_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_PARTICIPANT_WRITER;
	static const uint8_t reader_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_PARTICIPANT_READER;
	static const uint8_t participant_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_PARTICIPANT;
#pragma HLS array_partition variable=writer_entity_id complete dim=0
#pragma HLS array_partition variable=reader_entity_id complete dim=0
#pragma HLS array_partition variable=participant_entity_id complete dim=0

	static const duration lease_duration = {20, 0};

	static const uint32_t endpoint_set =
		ENDPOINT_PARTICIPANT_ANNOUNCER |
		ENDPOINT_PARTICIPANT_DETECTOR |
		ENDPOINT_PUBLICATIONS_ANNOUNCER |
		ENDPOINT_PUBLICATIONS_DETECTOR |
		ENDPOINT_SUBSCRIPTIONS_ANNOUNCER |
		ENDPOINT_SUBSCRIPTIONS_DETECTOR |
		ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER |
		ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER;

	static const uint32_t entity_name_len = 7;
	static const uint16_t pid_entity_name_size =
		SP_DATA_SIZE(entity_name_len);

	static const uint32_t user_data_len = 25;
	static const uint16_t pid_user_data_size =
		SP_DATA_SIZE(user_data_len);

	static const int64_t seqnum = 1;

	int32_t seqnum_h = seqnum >> 32;
	uint32_t seqnum_l = seqnum & 0xffffffff;

	buf[0] = 'R';
	buf[1] = 'T';
	buf[2] = 'P';
	buf[3] = 'S';
	buf[4] = RTPS_HDR_PROTOCOL_VERSION >> 8;
	buf[5] = RTPS_HDR_PROTOCOL_VERSION & 0xff;
	buf[6] = RTPS_HDR_VENDOR_ID >> 8;
	buf[7] = RTPS_HDR_VENDOR_ID & 0xff;
	buf[8] = writer_guid_prefix[0];
	buf[9] = writer_guid_prefix[1];
	buf[10] = writer_guid_prefix[2];
	buf[11] = writer_guid_prefix[3];
	buf[12] = writer_guid_prefix[4];
	buf[13] = writer_guid_prefix[5];
	buf[14] = writer_guid_prefix[6];
	buf[15] = writer_guid_prefix[7];
	buf[16] = writer_guid_prefix[8];
	buf[17] = writer_guid_prefix[9];
	buf[18] = writer_guid_prefix[10];
	buf[19] = writer_guid_prefix[11];
	buf[20] = SBM_ID_INFO_TS;
	buf[21] = sbm_flags;
	buf[22] = S_BYTE0(TIMESTAMP_SIZE);
	buf[23] = S_BYTE1(TIMESTAMP_SIZE);
	buf[24] = L_BYTE0(now.seconds);
	buf[25] = L_BYTE1(now.seconds);
	buf[26] = L_BYTE2(now.seconds);
	buf[27] = L_BYTE3(now.seconds);
	buf[28] = L_BYTE0(now.fraction);
	buf[29] = L_BYTE1(now.fraction);
	buf[30] = L_BYTE2(now.fraction);
	buf[31] = L_BYTE3(now.fraction);
	buf[32] = SBM_ID_DATA;
	buf[33] = sbm_flags | SBM_FLAGS_DATA;
	buf[34] = S_BYTE0(octets_to_next_header);
	buf[35] = S_BYTE1(octets_to_next_header);
	buf[36] = ext_flags >> 8;
	buf[37] = ext_flags & 0xff;
	buf[38] = S_BYTE0(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
	buf[39] = S_BYTE1(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
	buf[40] = reader_entity_id[0];
	buf[41] = reader_entity_id[1];
	buf[42] = reader_entity_id[2];
	buf[43] = reader_entity_id[3];
	buf[44] = writer_entity_id[0];
	buf[45] = writer_entity_id[1];
	buf[46] = writer_entity_id[2];
	buf[47] = writer_entity_id[3];
	buf[48] = L_BYTE0(seqnum_h);
	buf[49] = L_BYTE1(seqnum_h);
	buf[50] = L_BYTE2(seqnum_h);
	buf[51] = L_BYTE3(seqnum_h);
	buf[52] = L_BYTE0(seqnum_l);
	buf[53] = L_BYTE1(seqnum_l);
	buf[54] = L_BYTE2(seqnum_l);
	buf[55] = L_BYTE3(seqnum_l);
	buf[56] = rep_id >> 8;
	buf[57] = rep_id & 0xff;
	buf[58] = rep_opt >> 8;
	buf[59] = rep_opt & 0xff;
	buf[60] = S_BYTE0(PID_PROTOCOL_VERSION);
	buf[61] = S_BYTE1(PID_PROTOCOL_VERSION);
	buf[62] = S_BYTE0(PID_PROTOCOL_VERSION_SIZE);
	buf[63] = S_BYTE1(PID_PROTOCOL_VERSION_SIZE);
	buf[64] = RTPS_HDR_PROTOCOL_VERSION >> 8;
	buf[65] = RTPS_HDR_PROTOCOL_VERSION & 0xff;
	buf[66] = 0; // padding
	buf[67] = 0; // padding
	buf[68] = S_BYTE0(PID_VENDOR_ID);
	buf[69] = S_BYTE1(PID_VENDOR_ID);
	buf[70] = S_BYTE0(PID_VENDOR_ID_SIZE);
	buf[71] = S_BYTE1(PID_VENDOR_ID_SIZE);
	buf[72] = RTPS_HDR_VENDOR_ID >> 8;
	buf[73] = RTPS_HDR_VENDOR_ID & 0xff;
	buf[74] = 0; // padding
	buf[75] = 0; // padding
	buf[76] = S_BYTE0(PID_PARTICIPANT_GUID);
	buf[77] = S_BYTE1(PID_PARTICIPANT_GUID);
	buf[78] = S_BYTE0(PID_PARTICIPANT_GUID_SIZE);
	buf[79] = S_BYTE1(PID_PARTICIPANT_GUID_SIZE);
	buf[80] = writer_guid_prefix[0];
	buf[81] = writer_guid_prefix[1];
	buf[82] = writer_guid_prefix[2];
	buf[83] = writer_guid_prefix[3];
	buf[84] = writer_guid_prefix[4];
	buf[85] = writer_guid_prefix[5];
	buf[86] = writer_guid_prefix[6];
	buf[87] = writer_guid_prefix[7];
	buf[88] = writer_guid_prefix[8];
	buf[89] = writer_guid_prefix[9];
	buf[90] = writer_guid_prefix[10];
	buf[91] = writer_guid_prefix[11];
	buf[92] = participant_entity_id[0];
	buf[93] = participant_entity_id[1];
	buf[94] = participant_entity_id[2];
	buf[95] = participant_entity_id[3];
	buf[96] = S_BYTE0(PID_METATRAFFIC_UNICAST_LOCATOR);
	buf[97] = S_BYTE1(PID_METATRAFFIC_UNICAST_LOCATOR);
	buf[98] = S_BYTE0(PID_METATRAFFIC_UNICAST_LOCATOR_SIZE);
	buf[99] = S_BYTE1(PID_METATRAFFIC_UNICAST_LOCATOR_SIZE);
	buf[100] = L_BYTE0(LOCATOR_KIND_UDPv4);
	buf[101] = L_BYTE1(LOCATOR_KIND_UDPv4);
	buf[102] = L_BYTE2(LOCATOR_KIND_UDPv4);
	buf[103] = L_BYTE3(LOCATOR_KIND_UDPv4);
#ifdef SBM_ENDIAN_LITTLE
	buf[104] = metatraffic_port[1];
	buf[105] = metatraffic_port[0];
	buf[106] = 0;
	buf[107] = 0;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	buf[104] = 0;
	buf[105] = 0;
	buf[106] = metatraffic_port[0];
	buf[107] = metatraffic_port[1];
#endif // SBM_ENDIAN_BIG
	buf[108] = 0;
	buf[109] = 0;
	buf[110] = 0;
	buf[111] = 0;
	buf[112] = 0;
	buf[113] = 0;
	buf[114] = 0;
	buf[115] = 0;
	buf[116] = 0;
	buf[117] = 0;
	buf[118] = 0;
	buf[119] = 0;
	buf[120] = metatraffic_addr[0];
	buf[121] = metatraffic_addr[1];
	buf[122] = metatraffic_addr[2];
	buf[123] = metatraffic_addr[3];
	buf[124] = S_BYTE0(PID_DEFAULT_UNICAST_LOCATOR);
	buf[125] = S_BYTE1(PID_DEFAULT_UNICAST_LOCATOR);
	buf[126] = S_BYTE0(PID_DEFAULT_UNICAST_LOCATOR_SIZE);
	buf[127] = S_BYTE1(PID_DEFAULT_UNICAST_LOCATOR_SIZE);
	buf[128] = L_BYTE0(LOCATOR_KIND_UDPv4);
	buf[129] = L_BYTE1(LOCATOR_KIND_UDPv4);
	buf[130] = L_BYTE2(LOCATOR_KIND_UDPv4);
	buf[131] = L_BYTE3(LOCATOR_KIND_UDPv4);
#ifdef SBM_ENDIAN_LITTLE
	buf[132] = default_port[1];
	buf[133] = default_port[0];
	buf[134] = 0;
	buf[135] = 0;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	buf[132] = 0;
	buf[133] = 0;
	buf[134] = default_port[0];
	buf[135] = default_port[1];
#endif // SBM_ENDIAN_BIG
	buf[136] = 0;
	buf[137] = 0;
	buf[138] = 0;
	buf[139] = 0;
	buf[140] = 0;
	buf[141] = 0;
	buf[142] = 0;
	buf[143] = 0;
	buf[144] = 0;
	buf[145] = 0;
	buf[146] = 0;
	buf[147] = 0;
	buf[148] = default_addr[0];
	buf[149] = default_addr[1];
	buf[150] = default_addr[2];
	buf[151] = default_addr[3];
	buf[152] = S_BYTE0(PID_PARTICIPANT_LEASE_DURATION);
	buf[153] = S_BYTE1(PID_PARTICIPANT_LEASE_DURATION);
	buf[154] = S_BYTE0(PID_PARTICIPANT_LEASE_DURATION_SIZE);
	buf[155] = S_BYTE1(PID_PARTICIPANT_LEASE_DURATION_SIZE);
	buf[156] = L_BYTE0(lease_duration.seconds);
	buf[157] = L_BYTE1(lease_duration.seconds);
	buf[158] = L_BYTE2(lease_duration.seconds);
	buf[159] = L_BYTE3(lease_duration.seconds);
	buf[160] = L_BYTE0(lease_duration.fraction);
	buf[161] = L_BYTE1(lease_duration.fraction);
	buf[162] = L_BYTE2(lease_duration.fraction);
	buf[163] = L_BYTE3(lease_duration.fraction);
	buf[164] = S_BYTE0(PID_BUILTIN_ENDPOINT_SET);
	buf[165] = S_BYTE1(PID_BUILTIN_ENDPOINT_SET);
	buf[166] = S_BYTE0(PID_BUILTIN_ENDPOINT_SET_SIZE);
	buf[167] = S_BYTE1(PID_BUILTIN_ENDPOINT_SET_SIZE);
	buf[168] = L_BYTE0(endpoint_set);
	buf[169] = L_BYTE1(endpoint_set);
	buf[170] = L_BYTE2(endpoint_set);
	buf[171] = L_BYTE3(endpoint_set);
	buf[172] = S_BYTE0(PID_ENTITY_NAME);
	buf[173] = S_BYTE1(PID_ENTITY_NAME);
	buf[174] = S_BYTE0(pid_entity_name_size);
	buf[175] = S_BYTE1(pid_entity_name_size);
	buf[176] = L_BYTE0(entity_name_len);
	buf[177] = L_BYTE1(entity_name_len);
	buf[178] = L_BYTE2(entity_name_len);
	buf[179] = L_BYTE3(entity_name_len);
	buf[180] = 't';
	buf[181] = 'a';
	buf[182] = 'l';
	buf[183] = 'k';
	buf[184] = 'e';
	buf[185] = 'r';
	buf[186] = '\0';
	buf[187] = 0; // padding
	buf[188] = S_BYTE0(PID_USER_DATA);
	buf[189] = S_BYTE1(PID_USER_DATA);
	buf[190] = S_BYTE0(pid_user_data_size);
	buf[191] = S_BYTE1(pid_user_data_size);
	buf[192] = L_BYTE0(user_data_len);
	buf[193] = L_BYTE1(user_data_len);
	buf[194] = L_BYTE2(user_data_len);
	buf[195] = L_BYTE3(user_data_len);
	buf[196] = 'n';
	buf[197] = 'a';
	buf[198] = 'm';
	buf[199] = 'e';
	buf[200] = '=';
	buf[201] = 't';
	buf[202] = 'a';
	buf[203] = 'l';
	buf[204] = 'k';
	buf[205] = 'e';
	buf[206] = 'r';
	buf[207] = ';';
	buf[208] = 'n';
	buf[209] = 'a';
	buf[210] = 'm';
	buf[211] = 'e';
	buf[212] = 's';
	buf[213] = 'p';
	buf[214] = 'a';
	buf[215] = 'c';
	buf[216] = 'e';
	buf[217] = '=';
	buf[218] = '/';
	buf[219] = ';';
	buf[220] = '\0';
	buf[221] = 0; // padding
	buf[222] = 0; // padding
	buf[223] = 0; // padding
	buf[224] = S_BYTE0(PID_SENTINEL);
	buf[225] = S_BYTE1(PID_SENTINEL);
	buf[226] = 0; // PID_SENTINEL_SIZE
	buf[227] = 0; // PID_SENTINEL_SIZE
}
