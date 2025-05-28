// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "sedp.hpp"
#include <cassert>

// Message from a publisher
constexpr uint8_t test_sedp_reader_pub_data[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xdb, 0xfe, 0x00, 0x00, 0x00, 0x00,
    // Submessage INFO_DST
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // Submessage INFO_TS
    0x09, 0x01, 0x08, 0x00, 0xf8, 0xe3, 0x2f, 0x68, 0xfb, 0xa4, 0x82, 0xa0,
    // Submessage DATA
    // Submessage header
    0x15, 0x05, 0x94, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0xc7,
    0x00, 0x00, 0x03, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Serialized data (PL_CDR_LE)
    0x00, 0x03, 0x00, 0x00,
    // PID_UNICAST_LOCATOR (192.168.0.2:7411)
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0xe0, 0x02,
    // PID_UNICAST_LOCATOR (192.168.0.209:7411)
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0xd1,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_TOPIC_NAME ("rt/aaa")
    0x05, 0x00, 0x0c, 0x00, 0x07, 0x00, 0x00, 0x00, 0x72, 0x74, 0x2f, 0x61,
    0x61, 0x61, 0x00, 0x00,
    // PID_TOPIC_TYPENAME ("std_msgs::msg::dds_::String_")
    0x07, 0x00, 0x24, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f,
    0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64,
    0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x5f,
    0x00, 0x00, 0x00, 0x00,
    // PID_KEY_HASH
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x03,
    // PID_ENDPOINT_GUID
    0x5a, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x03,
    // PID_TYPE_MAX_SIZE_SERIALIZED
    0x60, 0x00, 0x04, 0x00, 0x0c, 0x00, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_DURABILITY
    0x1d, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_DURABILITY_SERVICE
    0x1e, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // PID_DEADLINE
    0x23, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    // PID_LATENCY_BUDGET
    0x27, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_LIVELINESS (AUTOMATIC_LIVELINESS_QOS, lease_duration=INFINITE)
    0x1b, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f,
    0xff, 0xff, 0xff, 0xff,
    // PID_RELIABILITY
    0x1a, 0x00, 0x0c, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x9a, 0x99, 0x99, 0x19,
    // PID_LIEFSPAN
    0x2b, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    // PID_USER_DATA
    0x2c, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_TIME_BASED_FILTER
    0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_OWNERSHIP
    0x1f, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_DESTINATION_ORDER
    0x25, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_PRESENTATION
    0x21, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_PARTITION
    0x29, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_TOPIC_DATA
    0x2e, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_GROUP_DATA
    0x2d, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

static void
call_sedp_reader(sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
                 app_endpoint  app_reader_tbl[APP_READER_MAX],
                 const uint8_t ip_addr[4], const uint8_t subnet_mask[4],
                 uint16_t      port_num_seed,
                 const uint8_t guid_prefix[GUID_PREFIX_SIZE],
                 const uint8_t pub_topic_name[], uint8_t pub_topic_name_len,
                 const uint8_t pub_type_name[], uint8_t pub_type_name_len,
                 const uint8_t sub_topic_name[], uint8_t sub_topic_name_len,
                 const uint8_t sub_type_name[], uint8_t sub_type_name_len,
                 const uint8_t test_data[], size_t test_data_len) {
    for (auto j = 0; j < test_data_len; j++) {
        hls_uint<9> x = test_data[j];
        if (j == (test_data_len - 1)) {
            x |= hls_uint<9>(0x100);
        }
        sedp_reader(x, sedp_reader_tbl, app_reader_tbl, 1, ip_addr, subnet_mask,
                    port_num_seed, guid_prefix, pub_topic_name,
                    pub_topic_name_len, pub_type_name, pub_type_name_len,
                    sub_topic_name, sub_topic_name_len, sub_type_name,
                    sub_type_name_len);
    }
}

int test_sedp_reader_2() {
    sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
    app_endpoint  app_reader_tbl[APP_READER_MAX];

    constexpr uint8_t  ip_addr[4] = {192, 168, 0, 4};
    constexpr uint8_t  subnet_mask[4] = {255, 255, 255, 0};
    constexpr uint16_t port_num_seed = 7400;
    constexpr uint8_t  guid_prefix[GUID_PREFIX_SIZE] = {
        0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

    constexpr uint8_t pub_topic_name[] = "rt/bbb";
    constexpr uint8_t pub_topic_name_len = sizeof(pub_topic_name);
    constexpr uint8_t pub_type_name[] = "std_msgs::msg::dds_::String_";
    constexpr uint8_t pub_type_name_len = sizeof(pub_type_name);

    constexpr uint8_t sub_topic_name[] = "rt/aaa";
    constexpr uint8_t sub_topic_name_len = sizeof(pub_topic_name);
    constexpr uint8_t sub_type_name[] = "std_msgs::msg::dds_::String_";
    constexpr uint8_t sub_type_name_len = sizeof(pub_type_name);

    // Test whether sedp_reader ignores dead participants.
    // Setup sedp_reader_tbl.
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
        sedp_reader_tbl[j].children = 0;
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
            sedp_reader_tbl[j].guid_prefix[k]
                = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
        }
        sedp_reader_tbl[j].builtin_pubrd_rd_seqnum = 1;
        sedp_reader_tbl[j].builtin_subrd_rd_seqnum = 1;
    }
    // Setup app_reader_tbl.
    for (auto j = 0; j < APP_READER_MAX; j++) {
        app_reader_tbl[j].alive = false;
    }
    // sedp_reader gets a message from a known and dead participant.
    call_sedp_reader(sedp_reader_tbl, app_reader_tbl, ip_addr, subnet_mask,
                     port_num_seed, guid_prefix, pub_topic_name,
                     pub_topic_name_len, pub_type_name, pub_type_name_len,
                     sub_topic_name, sub_topic_name_len, sub_type_name,
                     sub_type_name_len, test_sedp_reader_pub_data,
                     sizeof(test_sedp_reader_pub_data));
    // sedp_reader should not create app_endpoint.
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        assert(sedp_reader_tbl[j].children == 0);
    }
    assert(find_living_app_endpoints(app_reader_tbl) == 0);

    // Test whether sedp_reader overwrites when app_reader_tbl is full.
    // Setup sedp_reader_tbl.
    sedp_reader_tbl[0].alive = true;
    sedp_reader_tbl[0].children = (1 << APP_READER_MAX) - 1;
    sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
    sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
    for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
        sedp_reader_tbl[0].guid_prefix[k]
            = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
    }
    // Setup app_reader_tbl.
    for (auto j = 0; j < APP_READER_MAX; j++) {
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
            app_reader_tbl[j].guid_prefix[k] = 0;
        }
        app_reader_tbl[j].alive = true;
    }
    // sedp_reader gets a message from a known participant, but app_reader_tbl
    // is full.
    call_sedp_reader(sedp_reader_tbl, app_reader_tbl, ip_addr, subnet_mask,
                     port_num_seed, guid_prefix, pub_topic_name,
                     pub_topic_name_len, pub_type_name, pub_type_name_len,
                     sub_topic_name, sub_topic_name_len, sub_type_name,
                     sub_type_name_len, test_sedp_reader_pub_data,
                     sizeof(test_sedp_reader_pub_data));
    // Check app_reader_tbl.
    for (auto j = 0; j < APP_READER_MAX; j++) {
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
            // sedp_reader should not change app_reader_tbl.
            assert(app_reader_tbl[j].guid_prefix[k] == 0);
        }
    }

    // Test whether sedp_reader ignores known endpoints.
    constexpr unsigned int n_endpoint_patterns = (1 << APP_READER_MAX);
    for (auto known_endpoints = 1; known_endpoints < n_endpoint_patterns;
         known_endpoints++) {
        // Setup reader_tbl.
        sedp_reader_tbl[0].alive = true;
        sedp_reader_tbl[0].children = known_endpoints;
        sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
        sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
            sedp_reader_tbl[0].guid_prefix[k]
                = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
        }
        for (auto j = 1; j < SEDP_READER_MAX; j++) {
            sedp_reader_tbl[j].alive = false;
        }
        // Setup app_reader_tbl.
        for (auto j = 0; j < APP_READER_MAX; j++) {
            bool known = ((1 << j) & known_endpoints);
            for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
                app_reader_tbl[j].guid_prefix[k]
                    = known ? test_sedp_reader_pub_data
                                  [k + RTPS_HDR_OFFSET_GUID_PREFIX]
                            : 0;
            }
            app_reader_tbl[j].entity_id[0] = 0x00;
            app_reader_tbl[j].entity_id[1] = 0x00;
            app_reader_tbl[j].entity_id[2] = 0x11;
            app_reader_tbl[j].entity_id[3] = 0x03;
            app_reader_tbl[j].alive = known;
        }
        assert(find_living_app_endpoints(app_reader_tbl) == known_endpoints);
        // sedp_reader gets a message from a known endpoint.
        call_sedp_reader(sedp_reader_tbl, app_reader_tbl, ip_addr, subnet_mask,
                         port_num_seed, guid_prefix, pub_topic_name,
                         pub_topic_name_len, pub_type_name, pub_type_name_len,
                         sub_topic_name, sub_topic_name_len, sub_type_name,
                         sub_type_name_len, test_sedp_reader_pub_data,
                         sizeof(test_sedp_reader_pub_data));
        // sedp_reader should not create a new app_endpoint.
        assert(sedp_reader_tbl[0].children == known_endpoints);
        assert(find_living_app_endpoints(app_reader_tbl) == known_endpoints);
    }

    // Test whether sedp_reader create a new app_endpoint correctly.
    // sedp_reader_tbl[1] is a living participant and sends a test message.
    // sedp_reader_tbl[0] is another living  participant.
    // Others are dead endpoints.
    sedp_reader_tbl[0].alive = true;
    sedp_reader_tbl[1].alive = true;
    for (auto j = 2; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
    }
    for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
        sedp_reader_tbl[0].guid_prefix[k] = 0;
        sedp_reader_tbl[1].guid_prefix[k]
            = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
    }
    for (auto first_n_app_endpoints_alive = 0;
         first_n_app_endpoints_alive < APP_READER_MAX;
         first_n_app_endpoints_alive++) {
        hls_uint<APP_READER_MAX> expected
            = (1 << first_n_app_endpoints_alive) - 1;
        // Setup sedp_reader_tbl.
        sedp_reader_tbl[0].children = expected;
        sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
        sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
        sedp_reader_tbl[1].children = 0;
        sedp_reader_tbl[1].builtin_pubrd_rd_seqnum = 1;
        sedp_reader_tbl[1].builtin_subrd_rd_seqnum = 1;
        // Setup app_reader_tbl.
        for (auto j = 0; j < APP_READER_MAX; j++) {
            for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
                app_reader_tbl[j].guid_prefix[k] = 0;
            }
            app_reader_tbl[j].alive = (j < first_n_app_endpoints_alive);
        }
        assert(find_living_app_endpoints(app_reader_tbl) == expected);
        call_sedp_reader(sedp_reader_tbl, app_reader_tbl, ip_addr, subnet_mask,
                         port_num_seed, guid_prefix, pub_topic_name,
                         pub_topic_name_len, pub_type_name, pub_type_name_len,
                         sub_topic_name, sub_topic_name_len, sub_type_name,
                         sub_type_name_len, test_sedp_reader_pub_data,
                         sizeof(test_sedp_reader_pub_data));
        // sedp_reader should create a new app_endpoint.
        assert(sedp_reader_tbl[0].children == expected);
        assert(sedp_reader_tbl[1].children
               == (1 << first_n_app_endpoints_alive));
        assert(find_living_app_endpoints(app_reader_tbl)
               == (expected | (1 << first_n_app_endpoints_alive)));
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
            assert(
                app_reader_tbl[first_n_app_endpoints_alive].guid_prefix[k]
                == test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX]);
        }
    }

    // Test whether ros2rapper uses valid endpoint's data and does not use
    // invalid endpoint data. Setup sedp_reader_tbl and app_reader_tbl.
    // - sedp_reader_tbl[0] is a known and dead endpoint.
    // - sedp_reader_tbl[1] is a known and living endpoint.
    // - sedp_reader_tbl[0] and sedp_reader_tbl[1] have the same GUID prefix.
    // - app_reader_tbl is empty.
    // When ros2rapper gets a RTPS message from sedp_reader_tbl[1],
    // ros2rapper shoud use sedp_reader_tbl[1] and should not use
    // sedp_reader_tbl[0].
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = (j == 1);
    }
    sedp_reader_tbl[0].children = 0;
    sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
    sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
    sedp_reader_tbl[1].children = 0;
    sedp_reader_tbl[1].builtin_pubrd_rd_seqnum = 1;
    sedp_reader_tbl[1].builtin_subrd_rd_seqnum = 1;
    for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
        sedp_reader_tbl[0].guid_prefix[k]
            = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
        sedp_reader_tbl[1].guid_prefix[k]
            = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
    }
    // Setup app_reader_tbl.
    for (auto j = 0; j < APP_READER_MAX; j++) {
        app_reader_tbl[j].alive = false;
    }
    call_sedp_reader(sedp_reader_tbl, app_reader_tbl, ip_addr, subnet_mask,
                     port_num_seed, guid_prefix, pub_topic_name,
                     pub_topic_name_len, pub_type_name, pub_type_name_len,
                     sub_topic_name, sub_topic_name_len, sub_type_name,
                     sub_type_name_len, test_sedp_reader_pub_data,
                     sizeof(test_sedp_reader_pub_data));
    // Check
    assert(sedp_reader_tbl[0].children == 0);
    assert(sedp_reader_tbl[1].children == 1);
    assert(app_reader_tbl[0].alive);

    return 0;
}
