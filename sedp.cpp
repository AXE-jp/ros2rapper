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
#include "sedp.hpp"

/* Cyber func=inline */
static void compare_guid_prefix(const uint8_t x,
				const app_endpoint tbl[APP_READER_MAX],
				const int idx,
				hls_uint<APP_READER_MAX> &unmatched)
{
#pragma HLS inline
	/* Cyber unroll_times=all */
	for (int i = 0; i < APP_READER_MAX; i++) {
#pragma HLS unroll
		if (tbl[i].guid_prefix[idx] != x)
			unmatched |= (hls_uint<APP_READER_MAX>)(0x1 << i);
	}
}

/* Cyber func=inline */
static void compare_entity_id(const uint8_t x,
			      const app_endpoint tbl[APP_READER_MAX],
			      const int idx,
			      hls_uint<APP_READER_MAX> &unmatched)
{
#pragma HLS inline
	/* Cyber unroll_times=all */
	for (int i = 0; i < APP_READER_MAX; i++) {
#pragma HLS unroll
		if (tbl[i].entity_id[idx] != x)
			unmatched |= (hls_uint<APP_READER_MAX>)(0x1 << i);
	}
}

#define FLAGS_FOUND_GUID	0x01
#define FLAGS_FOUND_LOCATOR	0x02
#define FLAGS_UNMATCH_DOMAIN	0x04
#define FLAGS_UNMATCH_TOPIC	0x08
#define FLAGS_UNMATCH_TYPE	0x10

/* Cyber func=inline */
void sedp_reader(hls_stream<hls_uint<9>> &in,
		 app_reader_id_t &reader_cnt,
		 app_endpoint reader_tbl[APP_READER_MAX],
		 uint16_t port_num_seed,
		 const uint8_t guid_prefix[12],
		 uint8_t topic_name_len,
		 const uint8_t topic_name[MAX_TOPIC_NAME_LEN],
		 uint8_t type_name_len,
		 const uint8_t type_name[MAX_TOPIC_TYPE_NAME_LEN])
{
#pragma HLS inline
	static const uint8_t sub_reader_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable=sub_reader_id complete dim=0

	static hls_uint<4> state;
	static uint16_t offset;
	static hls_uint<5> flags;
	static hls_uint<APP_READER_MAX> unmatched;

	static uint8_t sbm_id;
	static bool sbm_le;
	static uint16_t sbm_len;
	static uint16_t rep_id;
	static uint16_t param_id;
	static uint16_t param_len;
	static uint16_t udp_port;
	static uint32_t sp_len;

	hls_uint<9> x;

	if (!in.read_nb(x))
		return;

	if (reader_cnt == APP_READER_MAX)
		return;

	app_endpoint &reader = reader_tbl[reader_cnt];
	uint8_t data = x & 0xff;
	bool end = x & 0x100;

	switch (state) {
	case 0:
		if (!rtps_compare_protocol(offset, data)) {
			state = 9;
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
			if (sbm_id == SBM_ID_INFO_DST)
				state = 2;
			else if (sbm_id == SBM_ID_DATA)
				state = 3;
			else
				state = 8;
		}
		break;
	case 2:
		if (offset < 12) {
			if (guid_prefix[offset] != data) {
				state = 9;
				break;
			}
		}
		offset++;
		if (offset == sbm_len) {
			offset = 0;
			state = 1;
		}
		break;
	case 3:
		if (!rtps_compare_reader_id(offset, data, sub_reader_id)) {
			state = 9;
			break;
		}
		offset++;
		if (offset == SBM_DATA_HDR_SIZE) {
			sbm_len -= SBM_DATA_HDR_SIZE;
			offset = 0;
			state = 4;
		}
		break;
	case 4:
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
			state = 5;
		}
		break;
	case 5:
		if (offset == 0)
			param_id = rep_id & SP_ID_CDR_LE ? data : data << 8;
		else
			param_id |= rep_id & SP_ID_CDR_LE ? data << 8 : data;
		offset++;
		if (offset == sizeof(param_id)) {
			sbm_len -= sizeof(param_id);
			offset = 0;
			state = 6;
		}
		break;
	case 6:
		if (offset == 0)
			param_len = rep_id & SP_ID_CDR_LE ? data : data << 8;
		else
			param_len |= rep_id & SP_ID_CDR_LE ? data << 8 : data;
		offset++;
		if (offset == sizeof(param_len)) {
			if (param_id == PID_SENTINEL) {
				hls_uint<5> found =
					FLAGS_FOUND_GUID | FLAGS_FOUND_LOCATOR;
				if (flags == found) {
					hls_uint<APP_READER_MAX> valid =
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
				state = 7;
			}
		}
		break;
	case 7:
		switch (param_id) {
		case PID_UNICAST_LOCATOR:
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
			break;
		case PID_TOPIC_NAME:
			if (offset < 4) {
				if (rep_id & SP_ID_CDR_LE) {
					if (offset == 0)
						sp_len = data;
					else if (offset == 1)
						sp_len |= data << 8;
					else if (offset == 2)
						sp_len |= data << 16;
					else
						sp_len |= data << 24;
				} else {
					if (offset == 0)
						sp_len = data << 24;
					else if (offset == 1)
						sp_len |= data << 16;
					else if (offset == 2)
						sp_len |= data << 8;
					else
						sp_len |= data;
				}
			} else {
				if (sp_len != topic_name_len) {
					flags |= (hls_uint<5>)
						FLAGS_UNMATCH_TOPIC;
					break;
				}
				if (offset < sp_len + 4 &&
				    topic_name[offset-4] != data) {
					flags |= (hls_uint<5>)
						FLAGS_UNMATCH_TOPIC;
				}
			}
			break;
		case PID_TYPE_NAME:
			if (offset < 4) {
				if (rep_id & SP_ID_CDR_LE) {
					if (offset == 0)
						sp_len = data;
					else if (offset == 1)
						sp_len |= data << 8;
					else if (offset == 2)
						sp_len |= data << 16;
					else
						sp_len |= data << 24;
				} else {
					if (offset == 0)
						sp_len = data << 24;
					else if (offset == 1)
						sp_len |= data << 16;
					else if (offset == 2)
						sp_len |= data << 8;
					else
						sp_len |= data;
				}
			} else {
				if (sp_len != type_name_len) {
					flags |= (hls_uint<5>)
						FLAGS_UNMATCH_TYPE;
					break;
				}
				if (offset < sp_len + 4 &&
				    type_name[offset-4] != data) {
					flags |= (hls_uint<5>)
						FLAGS_UNMATCH_TYPE;
				}
			}
			break;
		case PID_ENDPOINT_GUID:
			if (flags & FLAGS_FOUND_GUID)
				break;
			if (offset < 12) {
				reader.guid_prefix[offset] = data;
				compare_guid_prefix(data,
						    reader_tbl,
						    offset,
						    unmatched);
			} else if (offset < 16) {
				reader.entity_id[offset-12] = data;
				compare_entity_id(data,
						  reader_tbl,
						  offset - 12,
						  unmatched);
			}
		}
		offset++;
		if (offset == param_len) {
			if (param_id == PID_ENDPOINT_GUID) {
				flags |= (hls_uint<5>)FLAGS_FOUND_GUID;
			} else if (param_id == PID_UNICAST_LOCATOR) {
				if (udp_port == port_num_seed /*DOMAIN_ID(udp_port) != TARGET_DOMAIN_ID*/) {
					flags |= (hls_uint<5>)
						FLAGS_UNMATCH_DOMAIN;
				}
				flags |= (hls_uint<5>)FLAGS_FOUND_LOCATOR;
			}
			offset = 0;
			state = 5;
		}
		break;
	case 8:
		offset++;
		if (offset == sbm_len) {
			offset = 0;
			state = 1;
		}
		break;
	case 9:
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
void sedp_writer(const uint8_t writer_guid_prefix[12],
		 const uint8_t writer_entity_id[4],
		 const uint8_t reader_guid_prefix[12],
		 const uint8_t reader_entity_id[4],
		 const uint8_t usertraffic_addr[4],
		 const uint8_t usertraffic_port[2],
		 const uint8_t app_entity_id[4],
		 uint8_t buf[SEDP_WRITER_TOT_LEN],
		 uint8_t topic_name_len,
		 const uint8_t topic_name[MAX_TOPIC_NAME_LEN],
		 uint8_t type_name_len,
		 const uint8_t type_name[MAX_TOPIC_TYPE_NAME_LEN])
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
		SEDP_WRITER_OCTETS_TO_NEXT_HEADER;

	static const uint8_t participant_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_PARTICIPANT;
#pragma HLS array_partition variable=participant_entity_id complete dim=0

	uint16_t pid_topic_name_size = SP_DATA_SIZE(topic_name_len);

	uint16_t pid_type_name_size = SP_DATA_SIZE(type_name_len);

	static const uint32_t type_max_size_serialized = 84;
	static const uint32_t durability_qos = 0;
	static const duration deadline = DURATION_INFINITE;
	static const duration latency_budget = DURATION_ZERO;
	static const uint32_t liveliness_qos = 0;
	static const duration liveliness_duration = DURATION_INFINITE;
	static const uint32_t reliability_qos = 1;
	static const duration reliability_duration =
		{0, (uint64_t)((0.1 * (1ULL << 32)) + 0.5)}; // 100 ms
	static const duration lifespan = DURATION_INFINITE;
	static const uint32_t ownership = 0;
	static const uint32_t ownership_strength = 0;
	static const uint32_t destination_order = 0;

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
	buf[20] = SBM_ID_INFO_DST;
	buf[21] = sbm_flags;
	buf[22] = S_BYTE0(GUID_PREFIX_SIZE);
	buf[23] = S_BYTE1(GUID_PREFIX_SIZE);
	buf[24] = reader_guid_prefix[0];
	buf[25] = reader_guid_prefix[1];
	buf[26] = reader_guid_prefix[2];
	buf[27] = reader_guid_prefix[3];
	buf[28] = reader_guid_prefix[4];
	buf[29] = reader_guid_prefix[5];
	buf[30] = reader_guid_prefix[6];
	buf[31] = reader_guid_prefix[7];
	buf[32] = reader_guid_prefix[8];
	buf[33] = reader_guid_prefix[9];
	buf[34] = reader_guid_prefix[10];
	buf[35] = reader_guid_prefix[11];
	buf[36] = SBM_ID_INFO_TS;
	buf[37] = sbm_flags;
	buf[38] = S_BYTE0(TIMESTAMP_SIZE);
	buf[39] = S_BYTE1(TIMESTAMP_SIZE);
	buf[40] = L_BYTE0(now.seconds);
	buf[41] = L_BYTE1(now.seconds);
	buf[42] = L_BYTE2(now.seconds);
	buf[43] = L_BYTE3(now.seconds);
	buf[44] = L_BYTE0(now.fraction);
	buf[45] = L_BYTE1(now.fraction);
	buf[46] = L_BYTE2(now.fraction);
	buf[47] = L_BYTE3(now.fraction);
	buf[48] = SBM_ID_DATA;
	buf[49] = sbm_flags | SBM_FLAGS_DATA;
	buf[50] = S_BYTE0(octets_to_next_header);
	buf[51] = S_BYTE1(octets_to_next_header);
	buf[52] = ext_flags >> 8;
	buf[53] = ext_flags & 0xff;
	buf[54] = S_BYTE0(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
	buf[55] = S_BYTE1(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
	buf[56] = reader_entity_id[0];
	buf[57] = reader_entity_id[1];
	buf[58] = reader_entity_id[2];
	buf[59] = reader_entity_id[3];
	buf[60] = writer_entity_id[0];
	buf[61] = writer_entity_id[1];
	buf[62] = writer_entity_id[2];
	buf[63] = writer_entity_id[3];
	buf[64] = L_BYTE0(seqnum_h);
	buf[65] = L_BYTE1(seqnum_h);
	buf[66] = L_BYTE2(seqnum_h);
	buf[67] = L_BYTE3(seqnum_h);
	buf[68] = L_BYTE0(seqnum_l);
	buf[69] = L_BYTE1(seqnum_l);
	buf[70] = L_BYTE2(seqnum_l);
	buf[71] = L_BYTE3(seqnum_l);
	buf[72] = rep_id >> 8;
	buf[73] = rep_id & 0xff;
	buf[74] = rep_opt >> 8;
	buf[75] = rep_opt & 0xff;
	buf[76] = S_BYTE0(PID_UNICAST_LOCATOR);
	buf[77] = S_BYTE1(PID_UNICAST_LOCATOR);
	buf[78] = S_BYTE0(PID_UNICAST_LOCATOR_SIZE);
	buf[79] = S_BYTE1(PID_UNICAST_LOCATOR_SIZE);
	buf[80] = L_BYTE0(LOCATOR_KIND_UDPv4);
	buf[81] = L_BYTE1(LOCATOR_KIND_UDPv4);
	buf[82] = L_BYTE2(LOCATOR_KIND_UDPv4);
	buf[83] = L_BYTE3(LOCATOR_KIND_UDPv4);
#ifdef SBM_ENDIAN_LITTLE
	buf[84] = usertraffic_port[1];
	buf[85] = usertraffic_port[0];
	buf[86] = 0;
	buf[87] = 0;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	buf[84] = 0;
	buf[85] = 0;
	buf[86] = usertraffic_port[0];
	buf[87] = usertraffic_port[1];
#endif // SBM_ENDIAN_BIG
	buf[88] = 0;
	buf[89] = 0;
	buf[90] = 0;
	buf[91] = 0;
	buf[92] = 0;
	buf[93] = 0;
	buf[94] = 0;
	buf[95] = 0;
	buf[96] = 0;
	buf[97] = 0;
	buf[98] = 0;
	buf[99] = 0;
	buf[100] = usertraffic_addr[0];
	buf[101] = usertraffic_addr[1];
	buf[102] = usertraffic_addr[2];
	buf[103] = usertraffic_addr[3];
	buf[104] = S_BYTE0(PID_PARTICIPANT_GUID);
	buf[105] = S_BYTE1(PID_PARTICIPANT_GUID);
	buf[106] = S_BYTE0(PID_PARTICIPANT_GUID_SIZE);
	buf[107] = S_BYTE1(PID_PARTICIPANT_GUID_SIZE);
	buf[108] = writer_guid_prefix[0];
	buf[109] = writer_guid_prefix[1];
	buf[110] = writer_guid_prefix[2];
	buf[111] = writer_guid_prefix[3];
	buf[112] = writer_guid_prefix[4];
	buf[113] = writer_guid_prefix[5];
	buf[114] = writer_guid_prefix[6];
	buf[115] = writer_guid_prefix[7];
	buf[116] = writer_guid_prefix[8];
	buf[117] = writer_guid_prefix[9];
	buf[118] = writer_guid_prefix[10];
	buf[119] = writer_guid_prefix[11];
	buf[120] = participant_entity_id[0];
	buf[121] = participant_entity_id[1];
	buf[122] = participant_entity_id[2];
	buf[123] = participant_entity_id[3];
	buf[124] = S_BYTE0(PID_TOPIC_NAME);
	buf[125] = S_BYTE1(PID_TOPIC_NAME);
	buf[126] = S_BYTE0(pid_topic_name_size);
	buf[127] = S_BYTE1(pid_topic_name_size);
	buf[128] = L_BYTE0(topic_name_len);
	buf[129] = L_BYTE1(topic_name_len);
	buf[130] = L_BYTE2(topic_name_len);
	buf[131] = L_BYTE3(topic_name_len);
	buf[132] = topic_name[0];
	buf[133] = topic_name[1];
	buf[134] = topic_name[2];
	buf[135] = topic_name[3];
	buf[136] = topic_name[4];
	buf[137] = topic_name[5];
	buf[138] = topic_name[6];
	buf[139] = topic_name[7];
	buf[140] = topic_name[8];
	buf[141] = topic_name[9];
	buf[142] = topic_name[10];
	buf[143] = topic_name[11];
	buf[144] = topic_name[12];
	buf[145] = topic_name[13];
	buf[146] = topic_name[14];
	buf[147] = topic_name[15];
	buf[148] = topic_name[16];
	buf[149] = topic_name[17];
	buf[150] = topic_name[18];
	buf[151] = topic_name[19];
	buf[152] = topic_name[20];
	buf[153] = topic_name[21];
	buf[154] = topic_name[22];
	buf[155] = topic_name[23];
	buf[156] = topic_name[24];
	buf[157] = topic_name[25];
	buf[158] = topic_name[26];
	buf[159] = topic_name[27];
	buf[160] = topic_name[28];
	buf[161] = topic_name[29];
	buf[162] = topic_name[30];
	buf[163] = topic_name[31];
	buf[164] = S_BYTE0(PID_TYPE_NAME);
	buf[165] = S_BYTE1(PID_TYPE_NAME);
	buf[166] = S_BYTE0(pid_type_name_size);
	buf[167] = S_BYTE1(pid_type_name_size);
	buf[168] = L_BYTE0(type_name_len);
	buf[169] = L_BYTE1(type_name_len);
	buf[170] = L_BYTE2(type_name_len);
	buf[171] = L_BYTE3(type_name_len);
	buf[172] = type_name[0];
	buf[173] = type_name[1];
	buf[174] = type_name[2];
	buf[175] = type_name[3];
	buf[176] = type_name[4];
	buf[177] = type_name[5];
	buf[178] = type_name[6];
	buf[179] = type_name[7];
	buf[180] = type_name[8];
	buf[181] = type_name[9];
	buf[182] = type_name[10];
	buf[183] = type_name[11];
	buf[184] = type_name[12];
	buf[185] = type_name[13];
	buf[186] = type_name[14];
	buf[187] = type_name[15];
	buf[188] = type_name[16];
	buf[189] = type_name[17];
	buf[190] = type_name[18];
	buf[191] = type_name[19];
	buf[192] = type_name[20];
	buf[193] = type_name[21];
	buf[194] = type_name[22];
	buf[195] = type_name[23];
	buf[196] = type_name[24];
	buf[197] = type_name[25];
	buf[198] = type_name[26];
	buf[199] = type_name[27];
	buf[200] = type_name[28];
	buf[201] = type_name[29];
	buf[202] = type_name[30];
	buf[203] = type_name[31];
	buf[204] = type_name[32];
	buf[205] = type_name[33];
	buf[206] = type_name[34];
	buf[207] = type_name[35];
	buf[208] = type_name[36];
	buf[209] = type_name[37];
	buf[210] = type_name[38];
	buf[211] = type_name[39];
	buf[212] = type_name[40];
	buf[213] = type_name[41];
	buf[214] = type_name[42];
	buf[215] = type_name[43];
	buf[216] = type_name[44];
	buf[217] = type_name[45];
	buf[218] = type_name[46];
	buf[219] = type_name[47];
	buf[220] = type_name[48];
	buf[221] = type_name[49];
	buf[222] = type_name[50];
	buf[223] = type_name[51];
	buf[224] = type_name[52];
	buf[225] = type_name[53];
	buf[226] = type_name[54];
	buf[227] = type_name[55];
	buf[228] = type_name[56];
	buf[229] = type_name[57];
	buf[230] = type_name[58];
	buf[231] = type_name[59];
	buf[232] = type_name[60];
	buf[233] = type_name[61];
	buf[234] = type_name[62];
	buf[235] = type_name[63];
	buf[236] = S_BYTE0(PID_KEY_HASH);
	buf[237] = S_BYTE1(PID_KEY_HASH);
	buf[238] = S_BYTE0(PID_KEY_HASH_SIZE);
	buf[239] = S_BYTE1(PID_KEY_HASH_SIZE);
	buf[240] = writer_guid_prefix[0];
	buf[241] = writer_guid_prefix[1];
	buf[242] = writer_guid_prefix[2];
	buf[243] = writer_guid_prefix[3];
	buf[244] = writer_guid_prefix[4];
	buf[245] = writer_guid_prefix[5];
	buf[246] = writer_guid_prefix[6];
	buf[247] = writer_guid_prefix[7];
	buf[248] = writer_guid_prefix[8];
	buf[249] = writer_guid_prefix[9];
	buf[250] = writer_guid_prefix[10];
	buf[251] = writer_guid_prefix[11];
	buf[252] = app_entity_id[0];
	buf[253] = app_entity_id[1];
	buf[254] = app_entity_id[2];
	buf[255] = app_entity_id[3];
	buf[256] = S_BYTE0(PID_ENDPOINT_GUID);
	buf[257] = S_BYTE1(PID_ENDPOINT_GUID);
	buf[258] = S_BYTE0(PID_ENDPOINT_GUID_SIZE);
	buf[259] = S_BYTE1(PID_ENDPOINT_GUID_SIZE);
	buf[260] = writer_guid_prefix[0];
	buf[261] = writer_guid_prefix[1];
	buf[262] = writer_guid_prefix[2];
	buf[263] = writer_guid_prefix[3];
	buf[264] = writer_guid_prefix[4];
	buf[265] = writer_guid_prefix[5];
	buf[266] = writer_guid_prefix[6];
	buf[267] = writer_guid_prefix[7];
	buf[268] = writer_guid_prefix[8];
	buf[269] = writer_guid_prefix[9];
	buf[270] = writer_guid_prefix[10];
	buf[271] = writer_guid_prefix[11];
	buf[272] = app_entity_id[0];
	buf[273] = app_entity_id[1];
	buf[274] = app_entity_id[2];
	buf[275] = app_entity_id[3];
	buf[276] = S_BYTE0(PID_TYPE_MAX_SIZE_SERIALIZED);
	buf[277] = S_BYTE1(PID_TYPE_MAX_SIZE_SERIALIZED);
	buf[278] = S_BYTE0(PID_TYPE_MAX_SIZE_SERIALIZED_SIZE);
	buf[279] = S_BYTE1(PID_TYPE_MAX_SIZE_SERIALIZED_SIZE);
	buf[280] = L_BYTE0(type_max_size_serialized);
	buf[281] = L_BYTE1(type_max_size_serialized);
	buf[282] = L_BYTE2(type_max_size_serialized);
	buf[283] = L_BYTE3(type_max_size_serialized);
	buf[284] = S_BYTE0(PID_PROTOCOL_VERSION);
	buf[285] = S_BYTE1(PID_PROTOCOL_VERSION);
	buf[286] = S_BYTE0(PID_PROTOCOL_VERSION_SIZE);
	buf[287] = S_BYTE1(PID_PROTOCOL_VERSION_SIZE);
	buf[288] = RTPS_HDR_PROTOCOL_VERSION >> 8;
	buf[289] = RTPS_HDR_PROTOCOL_VERSION & 0xff;
	buf[290] = 0; // padding
	buf[291] = 0; // padding
	buf[292] = S_BYTE0(PID_VENDOR_ID);
	buf[293] = S_BYTE1(PID_VENDOR_ID);
	buf[294] = S_BYTE0(PID_VENDOR_ID_SIZE);
	buf[295] = S_BYTE1(PID_VENDOR_ID_SIZE);
	buf[296] = RTPS_HDR_VENDOR_ID >> 8;
	buf[297] = RTPS_HDR_VENDOR_ID & 0xff;
	buf[298] = 0; // padding
	buf[299] = 0; // padding
	buf[300] = S_BYTE0(PID_DURABILITY);
	buf[301] = S_BYTE1(PID_DURABILITY);
	buf[302] = S_BYTE0(PID_DURABILITY_SIZE);
	buf[303] = S_BYTE1(PID_DURABILITY_SIZE);
	buf[304] = L_BYTE0(durability_qos);
	buf[305] = L_BYTE1(durability_qos);
	buf[306] = L_BYTE2(durability_qos);
	buf[307] = L_BYTE3(durability_qos);
	buf[308] = S_BYTE0(PID_DEADLINE);
	buf[309] = S_BYTE1(PID_DEADLINE);
	buf[310] = S_BYTE0(PID_DEADLINE_SIZE);
	buf[311] = S_BYTE1(PID_DEADLINE_SIZE);
	buf[312] = L_BYTE0(deadline.seconds);
	buf[313] = L_BYTE1(deadline.seconds);
	buf[314] = L_BYTE2(deadline.seconds);
	buf[315] = L_BYTE3(deadline.seconds);
	buf[316] = L_BYTE0(deadline.fraction);
	buf[317] = L_BYTE1(deadline.fraction);
	buf[318] = L_BYTE2(deadline.fraction);
	buf[319] = L_BYTE3(deadline.fraction);
	buf[320] = S_BYTE0(PID_LATENCY_BUDGET);
	buf[321] = S_BYTE1(PID_LATENCY_BUDGET);
	buf[322] = S_BYTE0(PID_LATENCY_BUDGET_SIZE);
	buf[323] = S_BYTE1(PID_LATENCY_BUDGET_SIZE);
	buf[324] = L_BYTE0(latency_budget.seconds);
	buf[325] = L_BYTE1(latency_budget.seconds);
	buf[326] = L_BYTE2(latency_budget.seconds);
	buf[327] = L_BYTE3(latency_budget.seconds);
	buf[328] = L_BYTE0(latency_budget.fraction);
	buf[329] = L_BYTE1(latency_budget.fraction);
	buf[330] = L_BYTE2(latency_budget.fraction);
	buf[331] = L_BYTE3(latency_budget.fraction);
	buf[332] = S_BYTE0(PID_LIVELINESS);
	buf[333] = S_BYTE1(PID_LIVELINESS);
	buf[334] = S_BYTE0(PID_LIVELINESS_SIZE);
	buf[335] = S_BYTE1(PID_LIVELINESS_SIZE);
	buf[336] = L_BYTE0(liveliness_qos);
	buf[337] = L_BYTE1(liveliness_qos);
	buf[338] = L_BYTE2(liveliness_qos);
	buf[339] = L_BYTE3(liveliness_qos);
	buf[340] = L_BYTE0(liveliness_duration.seconds);
	buf[341] = L_BYTE1(liveliness_duration.seconds);
	buf[342] = L_BYTE2(liveliness_duration.seconds);
	buf[343] = L_BYTE3(liveliness_duration.seconds);
	buf[344] = L_BYTE0(liveliness_duration.fraction);
	buf[345] = L_BYTE1(liveliness_duration.fraction);
	buf[346] = L_BYTE2(liveliness_duration.fraction);
	buf[347] = L_BYTE3(liveliness_duration.fraction);
	buf[348] = S_BYTE0(PID_RELIABILITY);
	buf[349] = S_BYTE1(PID_RELIABILITY);
	buf[350] = S_BYTE0(PID_RELIABILITY_SIZE);
	buf[351] = S_BYTE1(PID_RELIABILITY_SIZE);
	buf[352] = L_BYTE0(reliability_qos);
	buf[353] = L_BYTE1(reliability_qos);
	buf[354] = L_BYTE2(reliability_qos);
	buf[355] = L_BYTE3(reliability_qos);
	buf[356] = L_BYTE0(reliability_duration.seconds);
	buf[357] = L_BYTE1(reliability_duration.seconds);
	buf[358] = L_BYTE2(reliability_duration.seconds);
	buf[359] = L_BYTE3(reliability_duration.seconds);
	buf[360] = L_BYTE0(reliability_duration.fraction);
	buf[361] = L_BYTE1(reliability_duration.fraction);
	buf[362] = L_BYTE2(reliability_duration.fraction);
	buf[363] = L_BYTE3(reliability_duration.fraction);
	buf[364] = S_BYTE0(PID_LIFESPAN);
	buf[365] = S_BYTE1(PID_LIFESPAN);
	buf[366] = S_BYTE0(PID_LIFESPAN_SIZE);
	buf[367] = S_BYTE1(PID_LIFESPAN_SIZE);
	buf[368] = L_BYTE0(lifespan.seconds);
	buf[369] = L_BYTE1(lifespan.seconds);
	buf[370] = L_BYTE2(lifespan.seconds);
	buf[371] = L_BYTE3(lifespan.seconds);
	buf[372] = L_BYTE0(lifespan.fraction);
	buf[373] = L_BYTE1(lifespan.fraction);
	buf[374] = L_BYTE2(lifespan.fraction);
	buf[375] = L_BYTE3(lifespan.fraction);
	buf[376] = S_BYTE0(PID_OWNERSHIP);
	buf[377] = S_BYTE1(PID_OWNERSHIP);
	buf[378] = S_BYTE0(PID_OWNERSHIP_SIZE);
	buf[379] = S_BYTE1(PID_OWNERSHIP_SIZE);
	buf[380] = L_BYTE0(ownership);
	buf[381] = L_BYTE1(ownership);
	buf[382] = L_BYTE2(ownership);
	buf[383] = L_BYTE3(ownership);
	buf[384] = S_BYTE0(PID_OWNERSHIP_STRENGTH);
	buf[385] = S_BYTE1(PID_OWNERSHIP_STRENGTH);
	buf[386] = S_BYTE0(PID_OWNERSHIP_STRENGTH_SIZE);
	buf[387] = S_BYTE1(PID_OWNERSHIP_STRENGTH_SIZE);
	buf[388] = L_BYTE0(ownership_strength);
	buf[389] = L_BYTE1(ownership_strength);
	buf[390] = L_BYTE2(ownership_strength);
	buf[391] = L_BYTE3(ownership_strength);
	buf[392] = S_BYTE0(PID_DESTINATION_ORDER);
	buf[393] = S_BYTE1(PID_DESTINATION_ORDER);
	buf[394] = S_BYTE0(PID_DESTINATION_ORDER_SIZE);
	buf[395] = S_BYTE1(PID_DESTINATION_ORDER_SIZE);
	buf[396] = L_BYTE0(destination_order);
	buf[397] = L_BYTE1(destination_order);
	buf[398] = L_BYTE2(destination_order);
	buf[399] = L_BYTE3(destination_order);
	buf[400] = S_BYTE0(PID_SENTINEL);
	buf[401] = S_BYTE1(PID_SENTINEL);
	buf[402] = 0; // PID_SENTINEL_SIZE
	buf[403] = 0; // PID_SENTINEL_SIZE
}

/* Cyber func=inline */
void sedp_heartbeat(const uint8_t writer_guid_prefix[12],
		    const uint8_t writer_entity_id[4],
		    const uint8_t reader_guid_prefix[12],
		    const uint8_t reader_entity_id[4],
		    const int64_t first_seqnum,
		    const int64_t last_seqnum,
		    const uint32_t cnt,
		    uint8_t buf[SEDP_HEARTBEAT_TOT_LEN])
{
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
	static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	static const uint8_t sbm_flags = 0;
#endif // SBM_ENDIAN_BIG

	int32_t first_seqnum_h = first_seqnum >> 32;
	uint32_t first_seqnum_l = first_seqnum & 0xffffffff;
	int32_t last_seqnum_h = last_seqnum >> 32;
	uint32_t last_seqnum_l = last_seqnum & 0xffffffff;

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
	buf[20] = SBM_ID_INFO_DST;
	buf[21] = sbm_flags;
	buf[22] = S_BYTE0(GUID_PREFIX_SIZE);
	buf[23] = S_BYTE1(GUID_PREFIX_SIZE);
	buf[24] = reader_guid_prefix[0];
	buf[25] = reader_guid_prefix[1];
	buf[26] = reader_guid_prefix[2];
	buf[27] = reader_guid_prefix[3];
	buf[28] = reader_guid_prefix[4];
	buf[29] = reader_guid_prefix[5];
	buf[30] = reader_guid_prefix[6];
	buf[31] = reader_guid_prefix[7];
	buf[32] = reader_guid_prefix[8];
	buf[33] = reader_guid_prefix[9];
	buf[34] = reader_guid_prefix[10];
	buf[35] = reader_guid_prefix[11];
	buf[36] = SBM_ID_HEARTBEAT;
	buf[37] = sbm_flags;
	buf[38] = S_BYTE0(SBM_HEARTBEAT_DATA_SIZE);
	buf[39] = S_BYTE1(SBM_HEARTBEAT_DATA_SIZE);
	buf[40] = reader_entity_id[0];
	buf[41] = reader_entity_id[1];
	buf[42] = reader_entity_id[2];
	buf[43] = reader_entity_id[3];
	buf[44] = writer_entity_id[0];
	buf[45] = writer_entity_id[1];
	buf[46] = writer_entity_id[2];
	buf[47] = writer_entity_id[3];
	buf[48] = L_BYTE0(first_seqnum_h);
	buf[49] = L_BYTE1(first_seqnum_h);
	buf[50] = L_BYTE2(first_seqnum_h);
	buf[51] = L_BYTE3(first_seqnum_h);
	buf[52] = L_BYTE0(first_seqnum_l);
	buf[53] = L_BYTE1(first_seqnum_l);
	buf[54] = L_BYTE2(first_seqnum_l);
	buf[55] = L_BYTE3(first_seqnum_l);
	buf[56] = L_BYTE0(last_seqnum_h);
	buf[57] = L_BYTE1(last_seqnum_h);
	buf[58] = L_BYTE2(last_seqnum_h);
	buf[59] = L_BYTE3(last_seqnum_h);
	buf[60] = L_BYTE0(last_seqnum_l);
	buf[61] = L_BYTE1(last_seqnum_l);
	buf[62] = L_BYTE2(last_seqnum_l);
	buf[63] = L_BYTE3(last_seqnum_l);
	buf[64] = L_BYTE0(cnt);
	buf[65] = L_BYTE1(cnt);
	buf[66] = L_BYTE2(cnt);
	buf[67] = L_BYTE3(cnt);
}

/* Cyber func=inline */
void sedp_acknack(const uint8_t writer_guid_prefix[12],
		  const uint8_t writer_entity_id[4],
		  const uint8_t reader_guid_prefix[12],
		  const uint8_t reader_entity_id[4],
		  const int64_t bitmap_base,
		  const uint32_t num_bits,
		  const uint8_t bitmap[4],
		  const uint32_t cnt,
		  uint8_t buf[SEDP_ACKNACK_TOT_LEN])
{
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
	static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	static const uint8_t sbm_flags = 0;
#endif // SBM_ENDIAN_BIG

	int32_t bitmap_base_h = bitmap_base >> 32;
	uint32_t bitmap_base_l = bitmap_base & 0xffffffff;

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
	buf[20] = SBM_ID_INFO_DST;
	buf[21] = sbm_flags;
	buf[22] = S_BYTE0(GUID_PREFIX_SIZE);
	buf[23] = S_BYTE1(GUID_PREFIX_SIZE);
	buf[24] = reader_guid_prefix[0];
	buf[25] = reader_guid_prefix[1];
	buf[26] = reader_guid_prefix[2];
	buf[27] = reader_guid_prefix[3];
	buf[28] = reader_guid_prefix[4];
	buf[29] = reader_guid_prefix[5];
	buf[30] = reader_guid_prefix[6];
	buf[31] = reader_guid_prefix[7];
	buf[32] = reader_guid_prefix[8];
	buf[33] = reader_guid_prefix[9];
	buf[34] = reader_guid_prefix[10];
	buf[35] = reader_guid_prefix[11];
	buf[36] = SBM_ID_ACKNACK;
	buf[37] = sbm_flags;
	buf[38] = S_BYTE0(SBM_ACKNACK_DATA_SIZE);
	buf[39] = S_BYTE1(SBM_ACKNACK_DATA_SIZE);
	buf[40] = reader_entity_id[0];
	buf[41] = reader_entity_id[1];
	buf[42] = reader_entity_id[2];
	buf[43] = reader_entity_id[3];
	buf[44] = writer_entity_id[0];
	buf[45] = writer_entity_id[1];
	buf[46] = writer_entity_id[2];
	buf[47] = writer_entity_id[3];
	buf[48] = L_BYTE0(bitmap_base_h);
	buf[49] = L_BYTE1(bitmap_base_h);
	buf[50] = L_BYTE2(bitmap_base_h);
	buf[51] = L_BYTE3(bitmap_base_h);
	buf[52] = L_BYTE0(bitmap_base_l);
	buf[53] = L_BYTE1(bitmap_base_l);
	buf[54] = L_BYTE2(bitmap_base_l);
	buf[55] = L_BYTE3(bitmap_base_l);
	buf[56] = L_BYTE0(num_bits);
	buf[57] = L_BYTE1(num_bits);
	buf[58] = L_BYTE2(num_bits);
	buf[59] = L_BYTE3(num_bits);
	buf[60] = bitmap[0];
	buf[61] = bitmap[1];
	buf[62] = bitmap[2];
	buf[63] = bitmap[3];
	buf[64] = L_BYTE0(cnt);
	buf[65] = L_BYTE1(cnt);
	buf[66] = L_BYTE2(cnt);
	buf[67] = L_BYTE3(cnt);
}
