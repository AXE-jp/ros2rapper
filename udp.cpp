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

#include "checksum.hpp"
#include "ip.hpp"
#include "udp.hpp"

#define PRE_CHECKSUM	0x141c

// Method of PRE_CHECKSUM Calculation
//
// const uint8_t dst_addr[4] = TARGET_IP_ADDR;
// sum = 0;
// checksum(sum, 4, dst_addr[0]);
// checksum(sum, 5, dst_addr[1]);
// checksum(sum, 6, dst_addr[2]);
// checksum(sum, 7, dst_addr[3]);
// checksum(sum, 8, 0);
// checksum(sum, 9, PSEUDO_HDR_PROTOCOL);
// printf("0x%04x\n", sum);

#define IN_STREAM_OFFSET_SRC_ADDR_0 0
#define IN_STREAM_OFFSET_SRC_ADDR_1 1
#define IN_STREAM_OFFSET_SRC_ADDR_2 2
#define IN_STREAM_OFFSET_SRC_ADDR_3 3
#define IN_STREAM_OFFSET_TOT_LEN_0  4
#define IN_STREAM_OFFSET_TOT_LEN_1  5
#define IN_STREAM_HDR_SIZE	6 // src_addr[4] (4 bytes) + tot_len (2 bytes)

#define UDP_IN_STATE_READ_HEADER  0
#define UDP_IN_STATE_OUT_RTPS     1
#define UDP_IN_STATE_OUT_UDP      2
#define UDP_IN_STATE_DISCARD      3

#define MAX_RAWUDP_RXBUF_PAYLOAD_LEN (RAWUDP_RXBUF_LEN - 8)

#define QUAD_UINT8(a,b,c,d) ((a) | ((b)<<8) | ((c)<<16) | ((d)<<24))

/* Cyber func=inline */
void udp_in(hls_stream<hls_uint<9>> &in,
	    hls_stream<hls_uint<9>> &out,
	    const uint8_t cpu_udp_port[2],
	    uint32_t rawudp_rxbuf[RAWUDP_RXBUF_LEN/4],
	    volatile uint8_t *rawudp_rxbuf_rel,
	    volatile uint8_t *rawudp_rxbuf_grant,
	    bool &parity_error)
{
#pragma HLS inline
	static hls_uint<2> state;
	static uint16_t offset;
	static uint16_t ram_offset;
	static uint16_t sum = PRE_CHECKSUM;
	static uint8_t src_addr[4];
	static uint8_t src_port[2];
	static uint8_t dst_port[2];
	static uint8_t tot_len[2];
	static hls_uint<2> data_pos;
	static uint8_t data_buf[4];

	hls_uint<9> x;

	uint8_t data;
	bool end;

#define READ_AND_CHECKSUM do { \
	if (!in.read_nb(x)) \
		return; \
	data = x & 0xff; \
	end = x & 0x100; \
} while (0)

	switch (state) {
	case UDP_IN_STATE_READ_HEADER:
		READ_AND_CHECKSUM;

		switch (offset) {
		case IN_STREAM_OFFSET_SRC_ADDR_0: src_addr[0] = data; break;
		case IN_STREAM_OFFSET_SRC_ADDR_1: src_addr[1] = data; break;
		case IN_STREAM_OFFSET_SRC_ADDR_2: src_addr[2] = data; break;
		case IN_STREAM_OFFSET_SRC_ADDR_3: src_addr[3] = data; break;
		case IN_STREAM_OFFSET_TOT_LEN_0:  tot_len[0] = data; break;
		case IN_STREAM_OFFSET_TOT_LEN_1:  tot_len[1] = data; break;
		case (IN_STREAM_HDR_SIZE + UDP_HDR_OFFSET_SPORT):   src_port[0] = data; break;
		case (IN_STREAM_HDR_SIZE + UDP_HDR_OFFSET_SPORT+1): src_port[1] = data; break;
		case (IN_STREAM_HDR_SIZE + UDP_HDR_OFFSET_DPORT):   dst_port[0] = data; break;
		case (IN_STREAM_HDR_SIZE + UDP_HDR_OFFSET_DPORT+1): dst_port[1] = data; break;
		}

		offset++;
		if (offset == IN_STREAM_HDR_SIZE + UDP_HDR_SIZE) {
			if (dst_port[0] == cpu_udp_port[0] && dst_port[1] == cpu_udp_port[1]) {
				uint16_t payload_len = ((tot_len[0] << 8) | tot_len[1]) - UDP_HDR_SIZE;
				if (*rawudp_rxbuf_grant == 1 && payload_len <= MAX_RAWUDP_RXBUF_PAYLOAD_LEN) {
					state = UDP_IN_STATE_OUT_UDP;
				} else {
					state = UDP_IN_STATE_DISCARD;
				}
			} else {
				state = UDP_IN_STATE_OUT_RTPS;
			}
		}
		break;
	case UDP_IN_STATE_OUT_RTPS:
		READ_AND_CHECKSUM;

		out.write(x);
		offset++;
		break;
	case UDP_IN_STATE_OUT_UDP:
		switch (ram_offset) {
		case 0:
			rawudp_rxbuf[ram_offset++] = QUAD_UINT8(src_addr[0], src_addr[1], src_addr[2], src_addr[3]);
			return;
		case 1:
			rawudp_rxbuf[ram_offset++] = QUAD_UINT8(src_port[1], src_port[0], tot_len[1], tot_len[0]);
			return;
		default:
			READ_AND_CHECKSUM;

			data_buf[data_pos] = data;
			if (data_pos == 3 || end)
				rawudp_rxbuf[ram_offset++] = QUAD_UINT8(data_buf[0], data_buf[1], data_buf[2], data_buf[3]);
			data_pos++;

			offset++;
			break;
		}
		break;
	case UDP_IN_STATE_DISCARD:
		READ_AND_CHECKSUM;

		offset++;
		break;
	}

	if (end) {
		if (sum != 0xffff)
			parity_error = true;

		if (state == UDP_IN_STATE_OUT_UDP)
			*rawudp_rxbuf_rel = 0; /*write dummy value to assert ap_vld*/

		offset = 0;
		ram_offset = 0;
		data_buf[0] = 0;
		data_buf[1] = 0;
		data_buf[2] = 0;
		data_buf[3] = 0;
		data_pos = 0;
		sum = PRE_CHECKSUM;
		state = UDP_IN_STATE_READ_HEADER;
	}
}

/* Cyber func=inline */
void udp_out(const uint8_t src_addr[4],
	     const uint8_t src_port[2],
	     const uint8_t dst_addr[4],
	     const uint8_t dst_port[2],
	     const uint8_t udp_data[],
	     const uint16_t udp_data_process_len,
	     const uint16_t udp_data_real_len,
	     uint8_t buf[])
{
#pragma HLS inline
	uint16_t tot_process_len = UDP_HDR_SIZE + udp_data_process_len;
	uint16_t tot_real_len = UDP_HDR_SIZE + udp_data_real_len;
	uint32_t sum = 0;
	uint16_t sum_n;

	uint8_t pseudo_hdr[PSEUDO_HDR_SIZE]/* Cyber array=REG */;
	uint8_t udp_hdr[UDP_HDR_SIZE]/* Cyber array=REG */;
#pragma HLS array_partition variable=pseudo_hdr complete dim=0
#pragma HLS array_partition variable=udp_hdr complete dim=0

	pseudo_hdr[0] = src_addr[0];
	pseudo_hdr[1] = src_addr[1];
	pseudo_hdr[2] = src_addr[2];
	pseudo_hdr[3] = src_addr[3];
	pseudo_hdr[4] = dst_addr[0];
	pseudo_hdr[5] = dst_addr[1];
	pseudo_hdr[6] = dst_addr[2];
	pseudo_hdr[7] = dst_addr[3];
	pseudo_hdr[8] = 0;
	pseudo_hdr[9] = PSEUDO_HDR_PROTOCOL;
	pseudo_hdr[10] = tot_real_len >> 8;
	pseudo_hdr[11] = tot_real_len & 0xff;

	udp_hdr[0] = src_port[0];
	udp_hdr[1] = src_port[1];
	udp_hdr[2] = dst_port[0];
	udp_hdr[3] = dst_port[1];
	udp_hdr[4] = tot_real_len >> 8;
	udp_hdr[5] = tot_real_len & 0xff;
	udp_hdr[6] = 0;
	udp_hdr[7] = 0;

	/* Cyber unroll_times=all */
	for (int i = 0; i < PSEUDO_HDR_SIZE; i++) {
#pragma HLS unroll
		if (i & 0x1)
			sum += pseudo_hdr[i];
		else
			sum += pseudo_hdr[i] << 8;
	}

	/* Cyber unroll_times=all */
	for (int i = 0; i < UDP_HDR_SIZE; i++) {
#pragma HLS unroll
		if (i & 0x1)
			sum += udp_hdr[i];
		else
			sum += udp_hdr[i] << 8;
	}

	/* Cyber unroll_times=all */
	for (int i = 0; i < udp_data_process_len; i++) {
#pragma HLS unroll
		if (i & 0x1)
			sum += udp_data[i];
		else
			sum += udp_data[i] << 8;
	}

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);
	sum_n = ~sum;

	udp_hdr[6] = sum_n >> 8;
	udp_hdr[7] = sum_n & 0xff;

	/* Cyber unroll_times=all */
	for (int i = 0; i < tot_process_len; i++) {
#pragma HLS unroll
		if (i < UDP_HDR_SIZE)
			buf[i] = udp_hdr[i];
		else
			buf[i] = udp_data[i-UDP_HDR_SIZE];
	}
}

/* Cyber func=inline */
void udp_set_header(const uint8_t src_port[2],
	     const uint8_t dst_port[2],
	     const uint16_t udp_data_len,
	     uint8_t udp_hdr[])
{
#pragma HLS inline
	uint16_t tot_len = UDP_HDR_SIZE + udp_data_len;

	udp_hdr[0] = src_port[0];
	udp_hdr[1] = src_port[1];
	udp_hdr[2] = dst_port[0];
	udp_hdr[3] = dst_port[1];
	udp_hdr[4] = tot_len >> 8;
	udp_hdr[5] = tot_len & 0xff;
	udp_hdr[6] = 0;
	udp_hdr[7] = 0;
}

/* Cyber func=inline */
void udp_set_checksum(uint8_t buf[])
{
#pragma HLS inline
	uint32_t sum = 0;
	uint16_t sum_n;

	uint8_t pseudo_hdr[PSEUDO_HDR_SIZE]/* Cyber array=REG */;
#pragma HLS array_partition variable=pseudo_hdr complete dim=0

	pseudo_hdr[0] = buf[IP_HDR_OFFSET_SADDR + 0];
	pseudo_hdr[1] = buf[IP_HDR_OFFSET_SADDR + 1];
	pseudo_hdr[2] = buf[IP_HDR_OFFSET_SADDR + 2];
	pseudo_hdr[3] = buf[IP_HDR_OFFSET_SADDR + 3];
	pseudo_hdr[4] = buf[IP_HDR_OFFSET_DADDR + 0];
	pseudo_hdr[5] = buf[IP_HDR_OFFSET_DADDR + 1];
	pseudo_hdr[6] = buf[IP_HDR_OFFSET_DADDR + 2];
	pseudo_hdr[7] = buf[IP_HDR_OFFSET_DADDR + 3];
	pseudo_hdr[8] = 0;
	pseudo_hdr[9] = PSEUDO_HDR_PROTOCOL;
	pseudo_hdr[10] = buf[IP_HDR_SIZE + UDP_HDR_OFFSET_ULEN + 0];
	pseudo_hdr[11] = buf[IP_HDR_SIZE + UDP_HDR_OFFSET_ULEN + 1];

	/* Cyber unroll_times=all */
	for (int i = 0; i < PSEUDO_HDR_SIZE; i++) {
#pragma HLS unroll
		if (i & 0x1)
			sum += pseudo_hdr[i];
		else
			sum += pseudo_hdr[i] << 8;
	}

	/* Cyber unroll_times=all */
	for (int i = IP_HDR_SIZE; i < MAX_TX_UDP_PAYLOAD_LEN; i++) {
#pragma HLS unroll
		if (i & 0x1)
			sum += buf[i];
		else
			sum += buf[i] << 8;
	}

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);
	sum_n = ~sum;

	buf[IP_HDR_SIZE + UDP_HDR_OFFSET_SUM + 0] = sum_n >> 8;
	buf[IP_HDR_SIZE + UDP_HDR_OFFSET_SUM + 1] = sum_n & 0xff;
}
