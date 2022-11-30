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
		const uint8_t app_data[],
		const uint32_t app_data_len,
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

	uint16_t tot_len =
		APP_TOT_LEN(app_data_len);
	uint16_t octets_to_next_header =
		APP_OCTETS_TO_NEXT_HEADER(app_data_len);

	int32_t seqnum_h = seqnum >> 32;
	uint32_t seqnum_l = seqnum & 0xffffffff;

	uint8_t app_hdr[APP_HDR_SIZE]/* Cyber array=REG */;
#pragma HLS array_partition variable=app_hdr complete dim=0

	app_hdr[0] = 'R';
	app_hdr[1] = 'T';
	app_hdr[2] = 'P';
	app_hdr[3] = 'S';
	app_hdr[4] = RTPS_HDR_PROTOCOL_VERSION >> 8;
	app_hdr[5] = RTPS_HDR_PROTOCOL_VERSION & 0xff;
	app_hdr[6] = RTPS_HDR_VENDOR_ID >> 8;
	app_hdr[7] = RTPS_HDR_VENDOR_ID & 0xff;
	app_hdr[8] = writer_guid_prefix[0];
	app_hdr[9] = writer_guid_prefix[1];
	app_hdr[10] = writer_guid_prefix[2];
	app_hdr[11] = writer_guid_prefix[3];
	app_hdr[12] = writer_guid_prefix[4];
	app_hdr[13] = writer_guid_prefix[5];
	app_hdr[14] = writer_guid_prefix[6];
	app_hdr[15] = writer_guid_prefix[7];
	app_hdr[16] = writer_guid_prefix[8];
	app_hdr[17] = writer_guid_prefix[9];
	app_hdr[18] = writer_guid_prefix[10];
	app_hdr[19] = writer_guid_prefix[11];
	app_hdr[20] = SBM_ID_INFO_DST;
	app_hdr[21] = sbm_flags;
	app_hdr[22] = S_BYTE0(GUID_PREFIX_SIZE);
	app_hdr[23] = S_BYTE1(GUID_PREFIX_SIZE);
	app_hdr[24] = reader_guid_prefix[0];
	app_hdr[25] = reader_guid_prefix[1];
	app_hdr[26] = reader_guid_prefix[2];
	app_hdr[27] = reader_guid_prefix[3];
	app_hdr[28] = reader_guid_prefix[4];
	app_hdr[29] = reader_guid_prefix[5];
	app_hdr[30] = reader_guid_prefix[6];
	app_hdr[31] = reader_guid_prefix[7];
	app_hdr[32] = reader_guid_prefix[8];
	app_hdr[33] = reader_guid_prefix[9];
	app_hdr[34] = reader_guid_prefix[10];
	app_hdr[35] = reader_guid_prefix[11];
	app_hdr[36] = SBM_ID_INFO_TS;
	app_hdr[37] = sbm_flags;
	app_hdr[38] = S_BYTE0(TIMESTAMP_SIZE);
	app_hdr[39] = S_BYTE1(TIMESTAMP_SIZE);
	app_hdr[40] = L_BYTE0(now.seconds);
	app_hdr[41] = L_BYTE1(now.seconds);
	app_hdr[42] = L_BYTE2(now.seconds);
	app_hdr[43] = L_BYTE3(now.seconds);
	app_hdr[44] = L_BYTE0(now.fraction);
	app_hdr[45] = L_BYTE1(now.fraction);
	app_hdr[46] = L_BYTE2(now.fraction);
	app_hdr[47] = L_BYTE3(now.fraction);
	app_hdr[48] = SBM_ID_DATA;
	app_hdr[49] = sbm_flags | SBM_FLAGS_DATA;
	app_hdr[50] = S_BYTE0(octets_to_next_header);
	app_hdr[51] = S_BYTE1(octets_to_next_header);
	app_hdr[52] = ext_flags >> 8;
	app_hdr[53] = ext_flags & 0xff;
	app_hdr[54] = S_BYTE0(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
	app_hdr[55] = S_BYTE1(SBM_DATA_HDR_OCTETS_TO_INLINE_QOS);
	app_hdr[56] = reader_entity_id[0];
	app_hdr[57] = reader_entity_id[1];
	app_hdr[58] = reader_entity_id[2];
	app_hdr[59] = reader_entity_id[3];
	app_hdr[60] = writer_entity_id[0];
	app_hdr[61] = writer_entity_id[1];
	app_hdr[62] = writer_entity_id[2];
	app_hdr[63] = writer_entity_id[3];
	app_hdr[64] = L_BYTE0(seqnum_h);
	app_hdr[65] = L_BYTE1(seqnum_h);
	app_hdr[66] = L_BYTE2(seqnum_h);
	app_hdr[67] = L_BYTE3(seqnum_h);
	app_hdr[68] = L_BYTE0(seqnum_l);
	app_hdr[69] = L_BYTE1(seqnum_l);
	app_hdr[70] = L_BYTE2(seqnum_l);
	app_hdr[71] = L_BYTE3(seqnum_l);
	app_hdr[72] = rep_id >> 8;
	app_hdr[73] = rep_id & 0xff;
	app_hdr[74] = rep_opt >> 8;
	app_hdr[75] = rep_opt & 0xff;
	app_hdr[76] = L_BYTE0(app_data_len);
	app_hdr[77] = L_BYTE1(app_data_len);
	app_hdr[78] = L_BYTE2(app_data_len);
	app_hdr[79] = L_BYTE3(app_data_len);

	/* Cyber unroll_times=all */
	for (int i = 0; i < tot_len; i++) {
#pragma HLS unroll
		if (i < APP_HDR_SIZE)
			buf[i] = app_hdr[i];
		else if (i < APP_HDR_SIZE + MAX_APP_DATA_LEN)
			buf[i] = app_data[i-APP_HDR_SIZE];
		else
			buf[i] = 0; // padding
	}
}
