// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cassert>

#include "ros2.hpp"
#include "sedp.hpp"
#include "test_sedp_reader.hpp"

void setup_topic_data(int id, uint8_t topic_name[][MAX_TOPIC_NAME_LEN],
                      uint8_t topic_name_len[],
                      uint8_t type_name[][MAX_TOPIC_TYPE_NAME_LEN],
                      uint8_t type_name_len[], const uint8_t topic_name_0[],
                      uint8_t topic_name_len_0, const uint8_t type_name_0[],
                      uint8_t type_name_len_0) {
    for (auto j = 0; j < topic_name_len_0; j++) {
        topic_name[id][j] = topic_name_0[j];
    }
    for (auto j = topic_name_len_0; j < MAX_TOPIC_NAME_LEN; j++) {
        topic_name[id][j] = 0;
    }

    topic_name_len[id] = topic_name_len_0;

    for (auto j = 0; j < type_name_len_0; j++) {
        type_name[id][j] = type_name_0[j];
    }
    for (auto j = type_name_len_0; j < MAX_TOPIC_TYPE_NAME_LEN; j++) {
        type_name[id][j] = 0;
    }

    type_name_len[id] = type_name_len_0;
}

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
    0xc0, 0xa8, 0x00, 0x02,
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

// Message from a subscriber
constexpr uint8_t test_sedp_reader_sub_data[] = {
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
    0x15, 0x05, 0x94, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0xc7,
    0x00, 0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Serialized data (PL_CDR_LE)
    0x00, 0x03, 0x00, 0x00,
    // PID_UNICAST_LOCATOR (192.168.0.2:7411)
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_UNICAST_LOCATOR (192.168.0.209:7411)
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0xd1,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_TOPIC_NAME ("rt/bbb")
    0x05, 0x00, 0x0c, 0x00, 0x07, 0x00, 0x00, 0x00, 0x72, 0x74, 0x2f, 0x62,
    0x62, 0x62, 0x00, 0x00,
    // PID_TOPIC_TYPENAME ("std_msgs::msg::dds_::String_")
    0x07, 0x00, 0x24, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f,
    0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64,
    0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x5f,
    0x00, 0x00, 0x00, 0x00,
    // PID_KEY_HASH
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x04,
    // PID_ENDPOINT_GUID
    0x5a, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x04,
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

// Message from a subscriber
constexpr uint8_t test_sedp_reader_sub_data_with_padding[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xdb, 0xfe, 0x00, 0x00, 0x00, 0x00,
    // Submessage INFO_DST
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // Submessage INFO_TS
    0x09, 0x01, 0x08, 0x00, 0x15, 0x05, 0x00, 0x00, 0x67, 0x0d, 0x69, 0x44,
    // Submessage DATA
    0x15, 0x05, 0x60, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0xc7,
    0x00, 0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Payload
    0x00, 0x03, 0x00, 0x00,
    // PID_UNICAST_LOCATOR (192.168.0.2:7411)
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_TOPIC_NAME ("rt/bbb")
    0x05, 0x00, 0x0c, 0x00, 0x07, 0x00, 0x00, 0x00, 0x72, 0x74, 0x2f, 0x62,
    0x62, 0x62, 0x00, 0x00,
    // Padding
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_TYPE_NAME ("std_msgs::msg::dds_::String_")
    0x07, 0x00, 0x24, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f,
    0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64,
    0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x5f,
    0x00, 0x00, 0x00, 0x00,
    // Padding
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_KEY_HASH
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x04,
    // PID_ENDPOINT_GUID
    0x5a, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x04,
    // PID_TYPE_MAX_SIZE_SERIALIZED
    0x60, 0x00, 0x04, 0x00, 0x54, 0x00, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_DURABILITY
    0x1d, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_DEADLINE
    0x23, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    // PID_LATENCY_BUDGET
    0x27, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_LIVELINESS
    0x1b, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f,
    0xff, 0xff, 0xff, 0xff,
    // PID_RELIABILITY
    0x1a, 0x00, 0x0c, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x9a, 0x99, 0x99, 0x19,
    // PID_LIFESPAN
    0x2b, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    // PID_OWNERSHIP
    0x1f, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_OWNERSHIP_STRENGTH
    0x06, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_DESTINATION_ORDER
    0x25, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// Message from a subscriber
constexpr uint8_t test_sedp_reader_sub_data_with_nonzero_padding[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xdb, 0xfe, 0x00, 0x00, 0x00, 0x00,
    // Submessage INFO_DST
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // Submessage INFO_TS
    0x09, 0x01, 0x08, 0x00, 0x15, 0x05, 0x00, 0x00, 0x67, 0x0d, 0x69, 0x44,
    // Submessage DATA
    0x15, 0x05, 0x60, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0xc7,
    0x00, 0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Payload
    0x00, 0x03, 0x00, 0x00,
    // PID_UNICAST_LOCATOR (192.168.0.2:7411)
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_TOPIC_NAME ("rt/bbb")
    0x05, 0x00, 0x0c, 0x00, 0x07, 0x00, 0x00, 0x00, 0x72, 0x74, 0x2f, 0x62,
    0x62, 0x62, 0x00, 0x00,
    // Padding
    0x00, 0x00, 0x14, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // PID_TYPE_NAME ("std_msgs::msg::dds_::String_")
    0x07, 0x00, 0x24, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f,
    0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64,
    0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x5f,
    0x00, 0x00, 0x00, 0x00,
    // Padding
    0x00, 0x00, 0x1c, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // PID_KEY_HASH
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x04,
    // PID_ENDPOINT_GUID
    0x5a, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xdb, 0xfe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x04,
    // PID_TYPE_MAX_SIZE_SERIALIZED
    0x60, 0x00, 0x04, 0x00, 0x54, 0x00, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_DURABILITY
    0x1d, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_DEADLINE
    0x23, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    // PID_LATENCY_BUDGET
    0x27, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_LIVELINESS
    0x1b, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f,
    0xff, 0xff, 0xff, 0xff,
    // PID_RELIABILITY
    0x1a, 0x00, 0x0c, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x9a, 0x99, 0x99, 0x19,
    // PID_LIFESPAN
    0x2b, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    // PID_OWNERSHIP
    0x1f, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_OWNERSHIP_STRENGTH
    0x06, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_DESTINATION_ORDER
    0x25, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

static void call_sedp_reader(
    sedp_endpoint            sedp_reader_tbl[SEDP_READER_MAX],
    app_endpoint             app_reader_tbl[APP_READER_MAX],
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    const uint8_t ip_addr[4], const uint8_t subnet_mask[4],
    uint16_t port_num_seed, const uint8_t guid_prefix[GUID_PREFIX_SIZE],
    const uint8_t pub_topic_name[PUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t pub_topic_name_len[PUB_TOPICS_MAX],
    const uint8_t pub_type_name[PUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t pub_type_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_topic_name[SUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t sub_topic_name_len[SUB_TOPICS_MAX],
    const uint8_t sub_type_name[SUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t sub_type_name_len[SUB_TOPICS_MAX], const uint8_t test_data[],
    size_t test_data_len) {
    for (auto j = 0; j < test_data_len; j++) {
        hls_uint<9> x = test_data[j];
        if (j == (test_data_len - 1)) {
            x |= hls_uint<9>(0x100);
        }
        sedp_reader(x, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
                    ip_addr, subnet_mask, port_num_seed, guid_prefix,
                    pub_topic_name, pub_topic_name_len, pub_type_name,
                    pub_type_name_len, sub_topic_name, sub_topic_name_len,
                    sub_type_name, sub_type_name_len);
    }
}

static void setup_topic_data_all(
    config_t *conf, int pub_id, const uint8_t pub_topic_name_0[],
    uint8_t pub_topic_name_len_0, const uint8_t pub_type_name_0[],
    uint8_t pub_type_name_len_0, int sub_id, const uint8_t sub_topic_name_0[],
    uint8_t sub_topic_name_len_0, const uint8_t sub_type_name_0[],
    uint8_t sub_type_name_len_0) {
    for (auto j = 0; j < PUB_TOPICS_MAX; j++) {
        if (j == pub_id) {
            // Set designated data.
            setup_topic_data(j, conf->pub_topic_name, conf->pub_topic_name_len,
                             conf->pub_topic_type_name,
                             conf->pub_topic_type_name_len, pub_topic_name_0,
                             pub_topic_name_len_0, pub_type_name_0,
                             pub_type_name_len_0);
        } else {
            // Set empty data.
            setup_topic_data(j, conf->pub_topic_name, conf->pub_topic_name_len,
                             conf->pub_topic_type_name,
                             conf->pub_topic_type_name_len, NULL, 0, NULL, 0);
        }
    }
    for (auto j = 0; j < SUB_TOPICS_MAX; j++) {
        if (j == sub_id) {
            // Set designated data.
            setup_topic_data(j, conf->sub_topic_name, conf->sub_topic_name_len,
                             conf->sub_topic_type_name,
                             conf->sub_topic_type_name_len, sub_topic_name_0,
                             sub_topic_name_len_0, sub_type_name_0,
                             sub_type_name_len_0);
        } else {
            // Set empty data.
            setup_topic_data(j, conf->sub_topic_name, conf->sub_topic_name_len,
                             conf->sub_topic_type_name,
                             conf->sub_topic_type_name_len, NULL, 0, NULL, 0);
        }
    }
}

#define SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0,        \
                             sub_id, sub_topic_name_0, sub_type_name_0)        \
    setup_topic_data_all(                                                      \
        &conf, pub_id, pub_topic_name_0, sizeof(pub_topic_name_0),             \
        pub_type_name_0, sizeof(pub_type_name_0), sub_id, sub_topic_name_0,    \
        sizeof(sub_topic_name_0), sub_type_name_0, sizeof(sub_type_name_0))

static void setup_reader_tables_with_default_value(
    sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
    app_endpoint  app_reader_tbl[APP_READER_MAX]) {
    // Setup sedp_reader_tbl[0].
    sedp_reader_tbl[0].alive = true;
    reset_sedp_endpoint_children(sedp_reader_tbl[0].children);
    sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
    sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
    for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
        sedp_reader_tbl[0].guid_prefix[k]
            = test_sedp_reader_pub_data[k + RTPS_HDR_OFFSET_GUID_PREFIX];
    }
    // Disable the rest endpoints in sedp_reader_tbl.
    for (auto j = 1; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
    }
    // Disable all endpoints in app_reader_tbl.
    for (auto j = 0; j < APP_READER_MAX; j++) {
        app_reader_tbl[j].alive = false;
    }
}

#define CALL_SEDP_READER(pub_enable, sub_enable, test_data)                    \
    call_sedp_reader(sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,  \
                     conf.ip_addr, conf.subnet_mask, conf.port_num_seed,       \
                     conf.guid_prefix, conf.pub_topic_name,                    \
                     conf.pub_topic_name_len, conf.pub_topic_type_name,        \
                     conf.pub_topic_type_name_len, conf.sub_topic_name,        \
                     conf.sub_topic_name_len, conf.sub_topic_type_name,        \
                     conf.sub_topic_type_name_len, test_data,                  \
                     sizeof(test_data))

#define CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_data)                          \
    do {                                                                       \
        SETUP_TOPIC_DATA_ALL(0, pub_topic_name_0, pub_type_name_0, 0,          \
                             sub_topic_name_0, sub_type_name_0);               \
        CALL_SEDP_READER(1, 1, test_data);                                     \
    } while (0)

static sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
static app_endpoint  app_reader_tbl[APP_READER_MAX];

int test_sedp_reader_2() {
    config_t conf = {
        .ip_addr = {192, 168, 0, 4},
        .subnet_mask = {255, 255, 255, 0},
        .port_num_seed = 7400,
        .guid_prefix = {0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00, 0x01,
                    0x00, 0x00, 0x00}
    };

    constexpr uint8_t pub_topic_name_0[] = "rt/bbb";
    constexpr uint8_t pub_type_name_0[] = "std_msgs::msg::dds_::String_";

    constexpr uint8_t sub_topic_name_0[] = "rt/aaa";
    constexpr uint8_t sub_type_name_0[] = "std_msgs::msg::dds_::String_";

    constexpr uint8_t empty_topic_name[] = {};
    constexpr uint8_t empty_type_name[] = {};

    constexpr uint8_t wrong_topic_name_1[] = "rt/aaaa";
    constexpr uint8_t wrong_topic_name_2[] = "rt/ccc";
    constexpr uint8_t wrong_type_name_1[] = "std::msgs::msg::dds_::Uint8_";
    constexpr uint8_t wrong_type_name_2[] = "std::msgs::msg::dds_::Uint16_";

    // Test whether sedp_reader ignores dead participants.
    // Setup sedp_reader_tbl.
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
        reset_sedp_endpoint_children(sedp_reader_tbl[j].children);
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
    CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_sedp_reader_pub_data);
    // sedp_reader should not create app_endpoint.
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        for (auto k = 0; k < APP_READER_MAX; k++) {
            assert(!sedp_reader_tbl[j].children[k]);
        }
    }
    for (auto j = 0; j < APP_READER_MAX; j++) {
        assert(!app_reader_tbl[j].alive);
    }

    // Test whether sedp_reader overwrites when app_reader_tbl is full.
    // Setup sedp_reader_tbl.
    sedp_reader_tbl[0].alive = true;
    for (auto j = 0; j < APP_READER_MAX; j++) {
        sedp_reader_tbl[0].children[j] = true;
    }
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
    CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_sedp_reader_pub_data);
    // Check app_reader_tbl.
    for (auto j = 0; j < APP_READER_MAX; j++) {
        for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
            // sedp_reader should not change app_reader_tbl.
            assert(app_reader_tbl[j].guid_prefix[k] == 0);
        }
    }

    // Test whether sedp_reader ignores known endpoints.
    constexpr unsigned int n_endpoint_patterns = (1 << MIN(APP_READER_MAX, 8));
    for (auto known_endpoints = 1; known_endpoints < n_endpoint_patterns;
         known_endpoints++) {
        // Setup reader_tbl.
        sedp_reader_tbl[0].alive = true;
        for (auto j = 0; j < APP_READER_MAX; j++) {
            sedp_reader_tbl[0].children[j] = (known_endpoints & (1 << j));
        }
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
        // sedp_reader gets a message from a known endpoint.
        CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_sedp_reader_pub_data);
        // sedp_reader should not create a new app_endpoint.
        for (auto j = 0; j < APP_READER_MAX; j++) {
            bool known = (known_endpoints & (1 << j));
            assert(sedp_reader_tbl[0].children[j] == known);
            assert(app_reader_tbl[j].alive == known);
        }
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
        // Setup sedp_reader_tbl.
        for (auto j = 0; j < APP_READER_MAX; j++) {
            sedp_reader_tbl[0].children[j] = (j < first_n_app_endpoints_alive);
        }
        sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
        sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
        reset_sedp_endpoint_children(sedp_reader_tbl[1].children);
        sedp_reader_tbl[1].builtin_pubrd_rd_seqnum = 1;
        sedp_reader_tbl[1].builtin_subrd_rd_seqnum = 1;
        // Setup app_reader_tbl.
        for (auto j = 0; j < APP_READER_MAX; j++) {
            for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
                app_reader_tbl[j].guid_prefix[k] = 0;
            }
            app_reader_tbl[j].alive = (j < first_n_app_endpoints_alive);
        }
        CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_sedp_reader_pub_data);
        // sedp_reader should create a new app_endpoint.
        for (auto j = 0; j < APP_READER_MAX; j++) {
            assert(sedp_reader_tbl[0].children[j]
                   == (j < first_n_app_endpoints_alive));
            assert(sedp_reader_tbl[1].children[j]
                   == (j == first_n_app_endpoints_alive));
            assert(app_reader_tbl[j].alive
                   == (j <= first_n_app_endpoints_alive));
        }
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
    reset_sedp_endpoint_children(sedp_reader_tbl[0].children);
    sedp_reader_tbl[0].builtin_pubrd_rd_seqnum = 1;
    sedp_reader_tbl[0].builtin_subrd_rd_seqnum = 1;
    reset_sedp_endpoint_children(sedp_reader_tbl[1].children);
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
    CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_sedp_reader_pub_data);
    // Check
    for (auto j = 0; j < APP_READER_MAX; j++) {
        assert(!sedp_reader_tbl[0].children[j]);
        assert(sedp_reader_tbl[1].children[j] == (j == 0));
    }
    assert(app_reader_tbl[0].alive);

    // Test whether sedp_reader finds a new subscriber when ROS2rapper gets a
    // message with padding.
    setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
    CALL_SEDP_READER_WITH_DEFAULT_ARGS(test_sedp_reader_sub_data_with_padding);
    assert(app_reader_tbl[0].alive);

    setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
    CALL_SEDP_READER_WITH_DEFAULT_ARGS(
        test_sedp_reader_sub_data_with_nonzero_padding);
    assert(app_reader_tbl[0].alive);

    // Test multi-topic publication
    for (auto pub_id = 0; pub_id < PUB_TOPICS_MAX; pub_id++) {
        int                      sub_id = 0;
        hls_uint<PUB_TOPICS_MAX> pub_enable = (1 << pub_id);
        hls_uint<SUB_TOPICS_MAX> sub_enable = (1 << sub_id);

        // Test whether ROS2rapper finds a new subscriber (case 1).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        // Topic name and its length are different from the topic name of the
        // message.
        SETUP_TOPIC_DATA_ALL(pub_id, wrong_topic_name_1, pub_type_name_0,
                             sub_id, sub_topic_name_0, sub_type_name_0);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_sub_data);
        // ROS2rapper should not find a subscriber.
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new subscriber (case 2).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        // Topic name is different from the topic name of the message, but their
        // lengths are the same.
        SETUP_TOPIC_DATA_ALL(pub_id, wrong_topic_name_2, pub_type_name_0,
                             sub_id, sub_topic_name_0, sub_type_name_0);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_sub_data);
        // ROS2rapper should not find a subscriber.
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new subscriber (case 3).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        // Topic type name and its length are different from the topic type name
        // of the message.
        SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, wrong_type_name_1,
                             sub_id, sub_topic_name_0, sub_type_name_0);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_sub_data);
        // ROS2rapper should not find a subscriber.
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new subscriber (case 4).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        // Topic type name is different from the topic type name of the message,
        // but their lengths are the same.
        SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, wrong_type_name_2,
                             sub_id, sub_topic_name_0, sub_type_name_0);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_sub_data);
        // ROS2rapper should not find a subscriber.
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new subscriber (case 5).
        unsigned int n_pub_enable_patterns = (1 << PUB_TOPICS_MAX);
        for (unsigned int pub_enable_pattern = 0;
             pub_enable_pattern < n_pub_enable_patterns; pub_enable_pattern++) {
            setup_reader_tables_with_default_value(sedp_reader_tbl,
                                                   app_reader_tbl);
            // Topic name and topic type name is matched.
            SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0,
                                 sub_id, sub_topic_name_0, sub_type_name_0);
            CALL_SEDP_READER(pub_enable_pattern, sub_enable,
                             test_sedp_reader_sub_data);
            // Check
            if (pub_enable_pattern & (1 << pub_id)) {
                // ROS2rapper should find a subscriber.
                assert(app_reader_tbl[0].alive);
                assert(app_reader_tbl[0].app_ep_type & APP_EP_PUB);
                assert(app_reader_tbl[0].topic_id == pub_id);
            } else {
                // ROS2rapper should ignore disabled topics.
                assert(!app_reader_tbl[0].alive);
            }
        }
    }

    // Test multi-topic subscription
    for (auto sub_id = 0; sub_id < SUB_TOPICS_MAX; sub_id++) {
        int                      pub_id = 0;
        hls_uint<PUB_TOPICS_MAX> pub_enable = (1 << pub_id);
        hls_uint<SUB_TOPICS_MAX> sub_enable = (1 << sub_id);

        // Test whether ROS2rapper finds a new publisher (case 1).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0, sub_id,
                             wrong_topic_name_1, sub_type_name_0);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_pub_data);
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new publisher (case 2).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0, sub_id,
                             wrong_topic_name_2, sub_type_name_0);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_pub_data);
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new publisher (case 3).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0, sub_id,
                             sub_topic_name_0, wrong_type_name_1);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_pub_data);
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new publisher (case 4).
        setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
        SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0, sub_id,
                             sub_topic_name_0, wrong_type_name_2);
        CALL_SEDP_READER(pub_enable, sub_enable, test_sedp_reader_pub_data);
        assert(!app_reader_tbl[0].alive);

        // Test whether ROS2rapper finds a new publisher (case 5).
        unsigned int n_sub_enable_patterns = (1 << SUB_TOPICS_MAX);
        for (unsigned int sub_enable_pattern = 0;
             sub_enable_pattern < n_sub_enable_patterns; sub_enable_pattern++) {
            setup_reader_tables_with_default_value(sedp_reader_tbl,
                                                   app_reader_tbl);
            SETUP_TOPIC_DATA_ALL(pub_id, pub_topic_name_0, pub_type_name_0,
                                 sub_id, sub_topic_name_0, sub_type_name_0);
            CALL_SEDP_READER(pub_enable, sub_enable_pattern,
                             test_sedp_reader_pub_data);
            if (sub_enable_pattern & (1 << sub_id)) {
                assert(app_reader_tbl[0].alive);
                assert(app_reader_tbl[0].app_ep_type & APP_EP_SUB);
            } else {
                assert(!app_reader_tbl[0].alive);
            }
        }
    }

    // Test whether ROS2rapper ignores a subscriber
    // - whose topic name is the same as pub_topic_name[0] and different from
    //   pub_topic_name[1],
    // - and whose type name is the same as pub_topic_type_name[1] and different
    //   from pub_topic_type_name[0].
    setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
    SETUP_TOPIC_DATA_ALL(0, pub_topic_name_0, pub_type_name_0, 0,
                         sub_topic_name_0, sub_type_name_0);
    setup_topic_data(0, conf.pub_topic_name, conf.pub_topic_name_len,
                     conf.pub_topic_type_name, conf.pub_topic_type_name_len,
                     pub_topic_name_0, sizeof(pub_topic_name_0), NULL, 0);
    setup_topic_data(1, conf.pub_topic_name, conf.pub_topic_name_len,
                     conf.pub_topic_type_name, conf.pub_topic_type_name_len,
                     NULL, 0, pub_type_name_0, sizeof(pub_type_name_0));
    CALL_SEDP_READER(3, 1, test_sedp_reader_sub_data);
    assert(!app_reader_tbl[0].alive);

    // Test whether ROS2rapper ignores a publisher
    // - whose topic name is the same as sub_topic_name[0] and different from
    //   sub_topic_name[1],
    // - and whose type name is the same as sub_topic_type_name[1] and different
    //   from sub_topic_type_name[0].
    setup_reader_tables_with_default_value(sedp_reader_tbl, app_reader_tbl);
    SETUP_TOPIC_DATA_ALL(0, pub_topic_name_0, pub_type_name_0, 0,
                         sub_topic_name_0, sub_type_name_0);
    setup_topic_data(0, conf.sub_topic_name, conf.sub_topic_name_len,
                     conf.sub_topic_type_name, conf.sub_topic_type_name_len,
                     sub_topic_name_0, sizeof(sub_topic_name_0), NULL, 0);
    setup_topic_data(1, conf.sub_topic_name, conf.sub_topic_name_len,
                     conf.sub_topic_type_name, conf.sub_topic_type_name_len,
                     NULL, 0, sub_type_name_0, sizeof(sub_type_name_0));
    CALL_SEDP_READER(1, 3, test_sedp_reader_pub_data);
    assert(!app_reader_tbl[0].alive);

    return 0;
}
