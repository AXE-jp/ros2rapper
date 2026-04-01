// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include <cstdint>
#include <cstdio>

#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"
#include "sedp.hpp"
#include "test_utils.hpp"

/*
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

static sedp_reader_tbl_t sedp_reader_tbl;
static app_reader_tbl_t  app_reader_tbl;
static uint8_t           sub_app_data_0[MAX_APP_DATA_LEN];
static uint8_t           sub_app_data_1[MAX_APP_DATA_LEN];
static uint8_t           sub_app_data_2[MAX_APP_DATA_LEN];
static uint8_t           sub_app_data_3[MAX_APP_DATA_LEN];

int test_sedp_reader_heartbeat() {
    hls_stream<hls_uint<9>> in;
    hls_uint<9>             x;
    int                     ii;
    int                     n = 0;

    const uint8_t sedp_pub_reader_entity_id[4] = {0x00, 0x00, 0x03, 0xc7};
    const uint8_t sedp_sub_reader_entity_id[4] = {0x00, 0x00, 0x04, 0xc7};

    const uint8_t  src_ip_addr[4] = {192, 168, 1, 123};
    const uint16_t src_udp_port = 45966;
    const uint8_t  src_guid_prefix[12] = {0x01, 0x0f, 0x70, 0x2e, 0x40, 0x1a,
                                          0x13, 0x46, 0x00, 0x00, 0x00, 0x00};
    uint64_t       wdata_0 = SEDP_ENDPOINT_ALIVE
                       | (static_cast<uint64_t>(src_udp_port) << 16)
                       | (static_cast<uint64_t>(0x2e700f01) << 32);
    uint64_t wdata_1 = 0x0000000046131a40;
    set_sedp_reader_tbl(wdata_0, &sedp_reader_tbl, 0, 0);
    set_sedp_reader_tbl(wdata_1, &sedp_reader_tbl, 0, 1);
    set_sedp_reader_tbl_ip_addr_and_rd_seqnums(src_ip_addr, 0, 1, 0, 1,
                                               &sedp_reader_tbl, 0);

    hls_uint<PUB_TOPICS_MAX> pub_enable = 1;
    hls_uint<SUB_TOPICS_MAX> sub_enable = 1;

    receiver_config_t conf = {
        .ip_addr = {192, 168, 1, 200},
        .subnet_mask = {255, 255, 255, 0},
        .port_num_seed = 7400,
        .guid_prefix = {0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00, 0x01,
                    0x00, 0x00, 0x00}
    };

    const uint8_t pub_topic_name_0[] = "rt/fpgapubtest";
    const uint8_t pub_type_name_0[] = "std_msgs::msg::dds_::String_";

    const uint8_t sub_topic_name_0[] = "rt/fpgapubtest";
    const uint8_t sub_type_name_0[] = "std_msgs::msg::dds_::String_";

    setup_topic_data(0, conf.pub_topic_name, conf.pub_topic_name_len,
                     conf.pub_topic_type_name, conf.pub_topic_type_name_len,
                     pub_topic_name_0, sizeof(pub_topic_name_0),
                     pub_type_name_0, sizeof(pub_type_name_0));

    for (auto id = 1; id < PUB_TOPICS_MAX; id++) {
        setup_topic_data(id, conf.pub_topic_name, conf.pub_topic_name_len,
                         conf.pub_topic_type_name, conf.pub_topic_type_name_len,
                         NULL, 0, NULL, 0);
    }

    setup_topic_data(0, conf.sub_topic_name, conf.sub_topic_name_len,
                     conf.sub_topic_type_name, conf.sub_topic_type_name_len,
                     sub_topic_name_0, sizeof(sub_topic_name_0),
                     sub_type_name_0, sizeof(sub_type_name_0));

    for (auto id = 1; id < SUB_TOPICS_MAX; id++) {
        setup_topic_data(id, conf.sub_topic_name, conf.sub_topic_name_len,
                         conf.sub_topic_type_name, conf.sub_topic_type_name_len,
                         NULL, 0, NULL, 0);
    }

    hls_uint<SUB_TOPICS_MAX> sub_app_data_req;
    hls_uint<SUB_TOPICS_MAX> sub_app_data_rel;
    hls_uint<SUB_TOPICS_MAX> sub_app_data_grant;
    hls_stream<uint64_t>     sub_app_data_recvinfo;
    /*****************************************************/

    int64_t                 timestamp_i64 = 0;
    hls_stream<rtps_data_t> stream;

    for (ii = 0; ii < sizeof(pkt22); ii++) {
        x = pkt22[ii];
        if (ii == (sizeof(pkt22) - 1)) {
            x |= hls_uint<9>(0x100);
        }
        in.write(x);
        ros2_receiver(in, stream, pub_enable, sub_enable, &sub_app_data_req,
                      &sub_app_data_rel, sub_app_data_grant, sub_app_data_0,
                      sub_app_data_1, sub_app_data_2, sub_app_data_3,
                      sub_app_data_recvinfo, conf);
    }
    call_ros2_in(stream, &sedp_reader_tbl, &app_reader_tbl, pub_enable,
                 sub_enable, timestamp_i64);

    unsigned int sedp_reader_cnt = 0;
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        if (is_sedp_endpoint_alive(&sedp_reader_tbl, j)) {
            sedp_reader_cnt++;
        }
    }
    std::cout << "reader_cnt = " << sedp_reader_cnt << std::endl;
    for (ii = 0; ii < SEDP_READER_MAX; ii++) {
        uint64_t rdata;
        get_sedp_reader_tbl(&rdata, &sedp_reader_tbl, ii, 0);
        if ((rdata & SEDP_ENDPOINT_ALIVE) != 0) {
            bool subrd_acknack_req
                = ((rdata & SEDP_ENDPOINT_SUBRD_ACKNACK_REQ) != 0);
            uint8_t tmp_ip_addr[4];
            uint8_t pubrd_wr_seqnum, pubrd_rd_seqnum, subrd_wr_seqnum,
                subrd_rd_seqnum;
            get_sedp_reader_tbl_ip_addr_and_rd_seqnums(
                tmp_ip_addr, &pubrd_wr_seqnum, &pubrd_rd_seqnum,
                &subrd_wr_seqnum, &subrd_rd_seqnum, &sedp_reader_tbl, ii);
            printf("tbl[%d] ****\n", ii);
            printf("builtin_subrd_wr_seqnum: %d\n", subrd_wr_seqnum);
            printf("builtin_subrd_rd_seqnum: %d\n", subrd_rd_seqnum);
            printf("builtin_subrd_acknack_req: %d\n", subrd_acknack_req);
        }
    }

    return 0;
}
