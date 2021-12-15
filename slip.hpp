#ifndef SLIP_HPP
#define SLIP_HPP

#include <cstdint>
#include "hls.hpp"

// from RFC1055
#define END	0300	// indicates end of packet
#define ESC	0333	// indicates byte stuffing
#define ESC_END	0334	// ESC ESC_END means END data byte
#define ESC_ESC	0335	// ESC ESC_ESC means ESC data byte

void slip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out);
void slip_out(hls_stream<hls_uint<9>> &in, hls_stream<uint8_t> &out);

#endif // !SLIP_HPP
