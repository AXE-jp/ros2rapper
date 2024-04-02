// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "common.hpp"
#include "hls.hpp"
#include "ip.hpp"

static const uint8_t ip_packet[]
    = {0x45, 0x00, 0x00, 0x48, 0x6e, 0x5d, 0x40, 0x00, 0x40, 0x11, 0x48, 0x25,
       0xc0, 0xa8, 0x01, 0x0a, 0xc0, 0xa8, 0x01, 0xc8, 0x8a, 0xcd, 0x04, 0xd2,
       0x00, 0x34, 0x84, 0x68, 0x54, 0x68, 0x65, 0x20, 0x71, 0x75, 0x69, 0x63,
       0x6b, 0x20, 0x62, 0x72, 0x6f, 0x77, 0x6e, 0x20, 0x66, 0x6f, 0x78, 0x20,
       0x6a, 0x75, 0x6d, 0x70, 0x73, 0x20, 0x6f, 0x76, 0x65, 0x72, 0x20, 0x74,
       0x68, 0x65, 0x20, 0x6c, 0x61, 0x7a, 0x79, 0x20, 0x64, 0x6f, 0x67, 0x2e};

void process_ip_packet(const uint8_t *packet, size_t packet_size) {
    hls_stream<hls_uint<9>> in;
    hls_stream<hls_uint<9>> out;
    hls_stream<hls_uint<9>> udpout;
    bool                    parity_error;
    bool                    udp_parity_error;
    static uint8_t
             payloadmem[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS];
    uint32_t udp_rxbuf[RAWUDP_RXBUF_LEN / 4];
    hls_uint<1>    enable = 0;
    uint8_t        udp_rxbuf_rel;
    uint8_t        udp_rxbuf_grant = 1;
    uint8_t        rx_udp_port[2] = {0x4, 0xd2};
    const uint32_t fragment_exp = 10;

    size_t i = 0;

    for (int i = 0; i < RAWUDP_RXBUF_LEN / 4; i++)
        udp_rxbuf[i] = 0;

    while (1) {
        hls_uint<9> x = packet[i];
        if (i == packet_size - 1) {
            x |= 0x100;
        }

        if (i < packet_size && in.write_nb(x))
            i++;
        ip_in(in, out, payloadmem, fragment_exp, true, parity_error);
        udp_in(out, udpout, enable, rx_udp_port, udp_rxbuf, &udp_rxbuf_rel,
               &udp_rxbuf_grant, udp_parity_error);

        hls_uint<9> y;
        if (udpout.read_nb(y)) {
            printf("%% out: %02x\n", y & 0xff);
        }

        if (i == packet_size)
            break;
    }

    for (volatile int i = 0; i < 100000; i++) {
        ip_in(in, out, payloadmem, fragment_exp, true, parity_error);
        udp_in(out, udpout, enable, rx_udp_port, udp_rxbuf, &udp_rxbuf_rel,
               &udp_rxbuf_grant, udp_parity_error);

        hls_uint<9> y;
        if (udpout.read_nb(y)) {
            printf("%% out: %02x\n", y & 0xff);
        }
    }

    printf("RECEIVED PAYLOAD:\n");
    for (int i = 0; i < RAWUDP_RXBUF_LEN / 4; i++) {
        printf("%02x %02x %02x %02x\n", udp_rxbuf[i] & 0xff,
               (udp_rxbuf[i] >> 8) & 0xff, (udp_rxbuf[i] >> 16) & 0xff,
               (udp_rxbuf[i] >> 24) & 0xff);
    }
    printf("--- --- ---\n");
    printf("%s\n", &udp_rxbuf[2]);
    printf("--- --- ---\n");
}

int main() {
    process_ip_packet(ip_packet, sizeof(ip_packet));
    process_ip_packet(ip_packet, sizeof(ip_packet));

    return 0;
}
