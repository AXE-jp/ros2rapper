// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include <cstdint>
#include <cstdio>

#include "endpoint.hpp"
#include "hls.hpp"
// #include "ros2.hpp"
#include "sedp.hpp"

/*
Link Error:  
cannot find crt1.o: No such file or directory ...
Solution:
export LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LIBRARY_PATH

HEARTBEAT Message Content:
guidPrefix: 010f702e401a134600000000
firstAvailableSeqNumber: 1
lastSeqNumber: 8
count: 1
*/

// Frame (110 bytes)
static const unsigned char pkt22[110] = {
    0x52, 0x54, 0x50, 0x53, 0x02, 0x02,             // ..RTPS..
    0x01, 0x0f, 0x01, 0x0f, 0x70, 0x2e, 0x40, 0x1a, // ....p.@.
    0x13, 0x46, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x01, // .F......
    0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, // ....7...
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, // ........
    0x1c, 0x00, 0x00, 0x00, 0x04, 0xc7, 0x00, 0x00, // ........
    0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, // ........
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, // ........
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00              // ......
};
static const uint32_t pkt22_size = sizeof(pkt22);

static void print_guid(const uint8_t guid[]) {
    int ii;
    for (ii = 0; ii < 12; ii += 4) {
        printf("%02X%02X%02X%02X ", guid[ii + 0], guid[ii + 1], guid[ii + 2],
               guid[ii + 3]);
    }
    printf("\n");
}
static void print_entity_id(const uint8_t ent_id[4]) {
    int ii;
    for (ii = 0; ii < 4; ii += 4) {
        printf("%02X%02X%02X%02X ", ent_id[ii + 0], ent_id[ii + 1],
               ent_id[ii + 2], ent_id[ii + 3]);
    }
    printf("\n");
}
static void print_ipaddr(const uint8_t ipaddr[]) {
    printf("%d.%d.%d.%d\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
}
static void print_port(const uint8_t portval[]) {
    uint16_t wval;

    wval = portval[0];
    wval <<= 8;
    wval |= portval[1];
    printf("%d\n", wval);
}

int main() {
    // hls_stream<hls_uint<9>> in;
    hls_uint<9> in;
    hls_uint<9> x;
    int         ii;
    int         n = 0;

    const uint8_t own_guid_prefix[12] = {0x01, 0x0f, 0x37, 0xad, 0xde, 0x09,
                                         0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    const uint8_t sedp_pub_reader_entity_id[4] = {0x00, 0x00, 0x03, 0xc7};
    const uint8_t sedp_sub_reader_entity_id[4] = {0x00, 0x00, 0x04, 0xc7};

    const uint8_t ip_addr[4] = {192, 168, 1, 200};
    const uint8_t subnet_mask[4] = {255, 255, 255, 0};

    sedp_endpoint sedp_reader_tbl[APP_READER_MAX];
    sedp_reader_tbl[0].guid_prefix[0] = 0x01;
    sedp_reader_tbl[0].guid_prefix[1] = 0x0f;
    sedp_reader_tbl[0].guid_prefix[2] = 0x70;
    sedp_reader_tbl[0].guid_prefix[3] = 0x2e;
    sedp_reader_tbl[0].guid_prefix[4] = 0x40;
    sedp_reader_tbl[0].guid_prefix[5] = 0x1a;
    sedp_reader_tbl[0].guid_prefix[6] = 0x13;
    sedp_reader_tbl[0].guid_prefix[7] = 0x46;
    sedp_reader_tbl[0].guid_prefix[8] = 0x00;
    sedp_reader_tbl[0].guid_prefix[9] = 0x00;
    sedp_reader_tbl[0].guid_prefix[10] = 0x00;
    sedp_reader_tbl[0].guid_prefix[11] = 0x00;
    sedp_reader_tbl[0].ip_addr[0] = 192;
    sedp_reader_tbl[0].ip_addr[1] = 168;
    sedp_reader_tbl[0].ip_addr[2] = 1;
    sedp_reader_tbl[0].ip_addr[3] = 123;
    sedp_reader_tbl[0].udp_port[0] = 45966 >> 8;
    sedp_reader_tbl[0].udp_port[1] = 0x00FF & 45966;
    sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
    sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
    sedp_reader_tbl[0].builtin_pubrd_wr_seqnum = 0;
    sedp_reader_tbl[0].builtin_subrd_wr_seqnum = 0;
    app_reader_id_t sedp_reader_cnt = (app_reader_id_t)1;

    app_endpoint    app_reader_tbl[APP_READER_MAX];
    app_reader_id_t app_reader_cnt = (app_reader_id_t)0;

    hls_uint<1> enable = 1;

    const uint16_t port_num_seed = 7400;

    const uint8_t pub_topic_name[] = "rt/fpgapubtest";
    const uint8_t pub_topic_name_len = sizeof(pub_topic_name);
    const uint8_t pub_type_name[] = "std_msgs::msg::dds_::String_";
    const uint8_t pub_type_name_len = sizeof(pub_type_name);

    const uint8_t sub_topic_name[] = "rt/fpgapubtest";
    const uint8_t sub_topic_name_len = sizeof(sub_topic_name);
    const uint8_t sub_type_name[] = "std_msgs::msg::dds_::String_";
    const uint8_t sub_type_name_len = sizeof(sub_type_name);

    /*****************************************************/

    for (ii = 0; ii < sizeof(pkt22); ii++) {
        x = pkt22[ii];
        if (ii == (sizeof(pkt22) - 1)) {
            x |= 0x100;
        }
        in.write(x);

        sedp_reader(in, sedp_reader_tbl, app_reader_cnt, app_reader_tbl, enable,
                    ip_addr, subnet_mask, port_num_seed, own_guid_prefix,
                    pub_topic_name, pub_topic_name_len, pub_type_name,
                    pub_type_name_len, sub_topic_name, sub_topic_name_len,
                    sub_type_name, sub_type_name_len);
    }

    std::cout << "reader_cnt = " << sedp_reader_cnt << std::endl;
    for (ii = 0; ii < sedp_reader_cnt; ii++) {
        printf("tbl[%d] ****\n", ii);
        printf("builtin_subrd_wr_seqnum: %d\n",
               sedp_reader_tbl[ii].builtin_subrd_wr_seqnum);
        printf("builtin_subrd_rd_seqnum: %d\n",
               sedp_reader_tbl[ii].builtin_subrd_rd_seqnum);
    }
}
