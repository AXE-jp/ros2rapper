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

#include "app.hpp"

#define APP_HDR_SIZE	APP_TOT_LEN(0)

/* Cyber func=inline */
void app_writer(const uint8_t writer_guid_prefix[12],
		const uint8_t writer_entity_id[4],
		const uint8_t reader_guid_prefix[12],
		const uint8_t reader_entity_id[4],
		const int64_t seqnum,
		volatile const uint8_t app_data[],
		uint32_t app_data_len,
		uint8_t buf[])
{
#pragma HLS inline
#ifdef SBM_ENDIAN_LITTLE
	static const uint8_t sbm_flags = SBM_FLAGS_ENDIANNESS;
	static const uint16_t rep_id = SP_ID_CDR_LE;
#endif // SBM_ENDIAN_LITTLE
#ifdef SBM_ENDIAN_BIG
	static const uint8_t sbm_flags = 0;
	static const uint16_t rep_id = SP_ID_CDR_BE;
#endif // SBM_ENDIAN_BIG

	static const timestamp now = TIME_ZERO;
	static const uint16_t ext_flags = 0;
	static const uint16_t rep_opt = 0;

	const uint16_t tot_len =
		APP_TOT_LEN(MAX_APP_DATA_LEN);
	const uint16_t octets_to_next_header =
		APP_OCTETS_TO_NEXT_HEADER(MAX_APP_DATA_LEN);

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
	buf[76] = L_BYTE0(app_data_len);
	buf[77] = L_BYTE1(app_data_len);
	buf[78] = L_BYTE2(app_data_len);
	buf[79] = L_BYTE3(app_data_len);

	/* Cyber unroll_times=all */
	for (int i = APP_HDR_SIZE; i < MAX_TX_UDP_PAYLOAD_LEN; i++) {
#pragma HLS unroll
		if (i < APP_HDR_SIZE + MAX_APP_DATA_LEN)
			buf[i] = app_data[i-APP_HDR_SIZE];
		else
			buf[i] = 0;
	}
}
