#include "common.hpp"

#include "checksum.hpp"
#include "ip.hpp"

/* Cyber func=inline */
void ip_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
	   bool &parity_error)
{
#pragma HLS inline
	static hls_uint<3> state;
	static uint16_t len;
	static uint16_t offset;
	static uint16_t sum;

	hls_uint<9> x;

	if (!in.read_nb(x))
		return;

	uint8_t data = x & 0xff;
	bool end = x & 0x100;

	switch (state) {
	case 0:
		checksum(sum, offset, data);
		offset++;
		if (offset == IP_HDR_OFFSET_TOT_LEN)
			state = 1;
		break;
	case 1:
		checksum(sum, offset, data);
		len = data << 8;
		offset++;
		state = 2;
		break;
	case 2:
		checksum(sum, offset, data);
		len |= data;
		len -= IP_HDR_SIZE;
		offset++;
		state = 3;
		break;
	case 3:
		checksum(sum, offset, data);
		offset++;
		if (offset == IP_HDR_OFFSET_DADDR)
			state = 4;
		break;
	case 4:
		checksum(sum, offset, data);
		out.write(x);
		offset++;
		if (offset == IP_HDR_SIZE) {
			if (sum == 0xffff) {
				out.write(len >> 8);
				out.write((end ? 0x100 : 0) | (len & 0xff));
				state = 5;
			} else {
				out.write(len >> 8);
				out.write(0x100 | (len & 0xff));
				parity_error = true;
				state = 6;
			}
		}
		break;
	case 5:
		out.write(x);
		break;
	case 6:
		; // do nothing
	}

	if (end) {
		state = 0;
		offset = 0;
		sum = 0;
	}
}

/* Cyber func=inline */
void ip_out(const uint8_t src_addr[4],
	    const uint8_t dst_addr[4],
	    const uint8_t ttl,
	    const uint8_t ip_data[],
	    const uint16_t ip_data_len,
	    uint8_t buf[])
{
#pragma HLS inline
	static uint16_t id;

	uint16_t tot_len = IP_HDR_SIZE + ip_data_len;
	uint32_t sum = 0;
	uint16_t sum_n;

	uint8_t ip_hdr[IP_HDR_SIZE]/* Cyber array=REG */;
#pragma HLS array_partition variable=ip_hdr complete dim=0

	ip_hdr[0] = IP_HDR_VERSION_IHL;
	ip_hdr[1] = IP_HDR_TOS;
	ip_hdr[2] = tot_len >> 8;
	ip_hdr[3] = tot_len & 0xff;
	ip_hdr[4] = id >> 8;
	ip_hdr[5] = id & 0xff;
	ip_hdr[6] = IP_HDR_FLAG_OFF >> 8;
	ip_hdr[7] = IP_HDR_FLAG_OFF & 0xff;
	ip_hdr[8] = ttl;
	ip_hdr[9] = IP_HDR_PROTOCOL;
	ip_hdr[10] = 0;
	ip_hdr[11] = 0;
	ip_hdr[12] = src_addr[0];
	ip_hdr[13] = src_addr[1];
	ip_hdr[14] = src_addr[2];
	ip_hdr[15] = src_addr[3];
	ip_hdr[16] = dst_addr[0];
	ip_hdr[17] = dst_addr[1];
	ip_hdr[18] = dst_addr[2];
	ip_hdr[19] = dst_addr[3];

	/* Cyber unroll_times=all */
	for (int i = 0; i < IP_HDR_SIZE; i++) {
#pragma HLS unroll
		if (i & 0x1)
			sum += ip_hdr[i];
		else
			sum += ip_hdr[i] << 8;
	}

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);
	sum_n = ~sum;

	ip_hdr[10] = sum_n >> 8;
	ip_hdr[11] = sum_n & 0xff;

	/* Cyber unroll_times=all */
	for (int i = 0; i < tot_len; i++) {
#pragma HLS unroll
		if (i < IP_HDR_SIZE)
			buf[i] = ip_hdr[i];
		else
			buf[i] = ip_data[i-IP_HDR_SIZE];
	}

	id++;
}
