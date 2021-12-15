#ifndef IP_HPP
#define IP_HPP

#include <cstdint>
#include "hls.hpp"

#define IP_HDR_SIZE	20

#define IP_HDR_OFFSET_VERSION_IHL	0	// Version, IHL
#define IP_HDR_OFFSET_TOS		1	// Type of Service
#define IP_HDR_OFFSET_TOT_LEN		2	// Total Length
#define IP_HDR_OFFSET_ID		4	// Identification
#define IP_HDR_OFFSET_FLAG_OFF		6	// Flags, Fragment Offset
#define IP_HDR_OFFSET_TTL		8	// Time to Live
#define IP_HDR_OFFSET_PROTOCOL		9	// Protocol
#define IP_HDR_OFFSET_CHECK		10	// Header Checksum
#define IP_HDR_OFFSET_SADDR		12	// Source Address
#define IP_HDR_OFFSET_DADDR		16	// Destination Address

#define IP_HDR_VERSION_IHL	0x45	// IPv4, 20 bytes
#define IP_HDR_TOS		0x00	// Unused
#define IP_HDR_FLAG_OFF		0x4000	// Don't fragment
#define IP_HDR_TTL_MULTICAST	0x01	// 1
#define IP_HDR_TTL_UNICAST	0x40	// 64
#define IP_HDR_PROTOCOL		0x11	// UDP

#define IP_MULTICAST_ADDR	{0xef, 0xff, 0x00, 0x01} // 239.255.0.1

void ip_in(hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
	   bool &parity_error);
void ip_out(const uint8_t src_addr[4],
	    const uint8_t dst_addr[4],
	    const uint8_t ttl,
	    const uint8_t ip_data[],
	    const uint16_t ip_data_len,
	    uint8_t buf[]);

#endif // !IP_HPP
