#include "pre_ip_in.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include <cstdint>

void pre_ip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out) {
#pragma HLS interface mode=ap_ctrl_none port=return
#pragma HLS interface mode=axis port=in
#pragma HLS interface mode=axis port=out
#pragma HLS pipeline style=flp
    static uint16_t offset = 0;
    static uint16_t len = 0;

    uint8_t x = in.read();
    switch (offset) {
    case IP_HDR_OFFSET_TOT_LEN:
        len = static_cast<uint16_t>(x) << 8;
        break;
    case IP_HDR_OFFSET_TOT_LEN + 1:
        len |= x;
        break;
    }

    offset++;
    if (offset == len) {
        out.write(0x100 | x);
        offset = 0;
    } else {
        out.write(x);
    }
}
