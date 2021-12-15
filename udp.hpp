#ifndef UDP_HPP
#define UDP_HPP

#include <cstdint>
#include "hls.hpp"

#define UDP_HDR_SIZE	8

#define UDP_HDR_OFFSET_SPORT	0	// Source Port
#define UDP_HDR_OFFSET_DPORT	2	// Destination Port
#define UDP_HDR_OFFSET_ULEN	4	// Length
#define UDP_HDR_OFFSET_SUM	6	// Checksum

#define PSEUDO_HDR_SIZE	12

#define PSEUDO_HDR_OFFSET_SADDR		0	// Source Address
#define PSEUDO_HDR_OFFSET_DADDR		4	// Destination Address
#define PSEUDO_HDR_OFFSET_PROTOCOL	8	// Protocol
#define PSEUDO_HDR_OFFSET_ULEN		10	// Length

#define PSEUDO_HDR_PROTOCOL	0x11 // UDP

void udp_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
	    bool &parity_error);
void udp_out(const uint8_t src_addr[4],
	     const uint8_t src_port[2],
	     const uint8_t dst_addr[4],
	     const uint8_t dst_port[2],
	     const uint8_t udp_data[],
	     const uint16_t udp_data_len,
	     uint8_t buf[]);

#endif // !UDP_HPP
