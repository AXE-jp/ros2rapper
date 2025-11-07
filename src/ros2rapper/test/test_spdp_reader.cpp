// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"
#include "duration.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"
#include "spdp.hpp"
#include <cassert>

// SPDP message sample with PID_PARTICIPANT_LEASE_DURATION.
constexpr uint8_t test_spdp_reader_data_1[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xcf, 0xe4, 0x00, 0x00, 0x00, 0x00,
    // INFO_TS submessage
    0x09, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // DATA submessage
    // Submessage header
    0x15, 0x05, 0xd8, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Serialized data
    0x00, 0x03, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xcf, 0xe4,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_METATRAFFIC_UNICAST_LOCATOR (192.168.0.2:7410)
    0x32, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_DEFAULT_UNICAST_LOCATOR (192.168.0.2:7411)
    0x31, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_PARTICIPANT_LEASE_DULATION (20sec)
    0x02, 0x00, 0x08, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_BUILTIN_ENDPOINT_SET
    0x58, 0x00, 0x04, 0x00, 0x3f, 0x0c, 0x0f, 0x00,
    // PID_ENTITY_NAME
    0x62, 0x00, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00,
    // PID_USER_DATA
    0x2c, 0x00, 0x10, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x65, 0x6e, 0x63, 0x6c,
    0x61, 0x76, 0x65, 0x3d, 0x2f, 0x3b, 0x00, 0x00,
    // PID_PROPERTY_LIST
    0x59, 0x00, 0x28, 0x00, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x50, 0x41, 0x52, 0x54, 0x49, 0x43, 0x49, 0x50, 0x41, 0x4e, 0x54, 0x5f,
    0x54, 0x59, 0x50, 0x45, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    0x53, 0x49, 0x4d, 0x50, 0x4c, 0x45, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// SPDP message sample without PID_PARTICIPANT_LEASE_DURATION.
constexpr uint8_t test_spdp_reader_data_2[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xcf, 0xe4, 0x00, 0x00, 0x00, 0x00,
    // INFO_TS submessage
    0x09, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // DATA submessage
    // Submessage header
    0x15, 0x05, 0xcc, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Serialized data
    0x00, 0x03, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xcf, 0xe4,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_METATRAFFIC_UNICAST_LOCATOR (192.168.0.2:7410)
    0x32, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_DEFAULT_UNICAST_LOCATOR (192.168.0.2:7411)
    0x31, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf3, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_BUILTIN_ENDPOINT_SET
    0x58, 0x00, 0x04, 0x00, 0x3f, 0x0c, 0x0f, 0x00,
    // PID_ENTITY_NAME
    0x62, 0x00, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00,
    // PID_USER_DATA
    0x2c, 0x00, 0x10, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x65, 0x6e, 0x63, 0x6c,
    0x61, 0x76, 0x65, 0x3d, 0x2f, 0x3b, 0x00, 0x00,
    // PID_PROPERTY_LIST
    0x59, 0x00, 0x28, 0x00, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x50, 0x41, 0x52, 0x54, 0x49, 0x43, 0x49, 0x50, 0x41, 0x4e, 0x54, 0x5f,
    0x54, 0x59, 0x50, 0x45, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    0x53, 0x49, 0x4d, 0x50, 0x4c, 0x45, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// SPDP Message with padding
constexpr uint8_t test_spdp_reader_data_3[] = {
    // RTPS Header (GUID prefix:
    // 0x01,0x0f,0x37,0xad,0xde,0x09,0x00,0x00,0x01,0x00,0x00,0x00)
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x37, 0xad,
    0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Submessage INFO_TS
    0x09, 0x01, 0x08, 0x00, 0x32, 0x05, 0x00, 0x00, 0xee, 0x60, 0xdc, 0x8e,
    // Submessage DATA
    0x15, 0x05, 0xb4, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Encapsulation kind and options
    0x00, 0x03, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_METATRAFIC_UNICAST_LOCATOR (192.168.0.2:7412)
    0x32, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf4, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_DEFAULT_UNICAST_LOCATOR (192.168.0.2:7413)
    0x31, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf5, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_PARTICIPANT_LEASE_DURATION
    0x02, 0x00, 0x08, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_BUILTIN_ENDPOINT_SET
    0x58, 0x00, 0x04, 0x00, 0x3f, 0x0c, 0x00, 0x00,
    // PID_ENTITY_NAME
    0x62, 0x00, 0x18, 0x00, 0x13, 0x00, 0x00, 0x00, 0x72, 0x6f, 0x73, 0x32,
    0x72, 0x61, 0x70, 0x70, 0x65, 0x72, 0x5f, 0x65, 0x78, 0x61, 0x6d, 0x70,
    0x6c, 0x65, 0x00, 0x00,
    // Padding
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// SPDP Message with non-zero padding
constexpr uint8_t test_spdp_reader_data_4[] = {
    // RTPS Header (GUID prefix:
    // 0x01,0x0f,0x37,0xad,0xde,0x09,0x00,0x00,0x01,0x00,0x00,0x00)
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x37, 0xad,
    0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Submessage INFO_TS
    0x09, 0x01, 0x08, 0x00, 0x32, 0x05, 0x00, 0x00, 0xee, 0x60, 0xdc, 0x8e,
    // Submessage DATA
    0x15, 0x05, 0xb4, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Encapsulation kind and options
    0x00, 0x03, 0x00, 0x00,
    // PID_PROTOCOL_VERSION
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00,
    // PID_VENDOR_ID
    0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    // PID_PARTICIPANT_GUID
    0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_METATRAFIC_UNICAST_LOCATOR (192.168.0.2:7412)
    0x32, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf4, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_DEFAULT_UNICAST_LOCATOR (192.168.0.2:7413)
    0x31, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf5, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8, 0x00, 0x02,
    // PID_PARTICIPANT_LEASE_DURATION
    0x02, 0x00, 0x08, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_BUILTIN_ENDPOINT_SET
    0x58, 0x00, 0x04, 0x00, 0x3f, 0x0c, 0x00, 0x00,
    // PID_ENTITY_NAME
    0x62, 0x00, 0x18, 0x00, 0x13, 0x00, 0x00, 0x00, 0x72, 0x6f, 0x73, 0x32,
    0x72, 0x61, 0x70, 0x70, 0x65, 0x72, 0x5f, 0x65, 0x78, 0x61, 0x6d, 0x70,
    0x6c, 0x65, 0x00, 0x00,
    // Padding
    0x00, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

constexpr uint8_t SOURCE_GUID_PREFIX[GUID_PREFIX_SIZE]
    = {0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0xcf, 0xe4, 0x00, 0x00, 0x00, 0x00};

static void call_spdp_reader(hls_stream<rtps_data_t> &out,
                             const uint8_t            ip_addr[4],
                             const uint8_t            subnet_mask[4],
                             uint16_t port_num_seed, const uint8_t test_data[],
                             size_t test_data_size) {
    for (auto j = 0; j < test_data_size; j++) {
        hls_uint<9> data = test_data[j];
        if (j == (test_data_size - 1)) {
            data |= hls_uint<9>(0x100);
        }
        spdp_reader(data, out, true, ip_addr, subnet_mask, port_num_seed);
    }
}

#define CALL_SPDP_READER(out, ip_addr, subnet_mask, port_num_seed, test_data)  \
    call_spdp_reader(out, ip_addr, subnet_mask, port_num_seed, test_data,      \
                     sizeof(test_data))

static int test_spdp_reader_0() {
    // Test spdp_reader
    constexpr uint8_t       ip_addr[4] = {192, 168, 0, 3};
    constexpr uint8_t       subnet_mask[4] = {255, 255, 255, 0};
    constexpr uint16_t      port_num_seed = 7400;
    hls_stream<rtps_data_t> out;
    rtps_data_t             rtps_data;

    CALL_SPDP_READER(out, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_1);
    assert(!out.empty());
    rtps_data = out.read();
    assert(out.empty());
    assert(rtps_data.type == RTPS_TYPE_SPDP);
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
        assert(rtps_data.guid_prefix[j]
               == test_spdp_reader_data_1[j + RTPS_HDR_OFFSET_GUID_PREFIX]);
    }
    // Check IP address
    assert(rtps_data.data[0] == 192);
    assert(rtps_data.data[1] == 168);
    assert(rtps_data.data[2] == 0);
    assert(rtps_data.data[3] == 2);
    // Check UDP port
    assert(rtps_data.data[4] == (7410 >> 8));
    assert(rtps_data.data[5] == (7410 & 0xff));
    // Check participant lease duration
    for (auto j = 0; j < 8; j++) {
        if (j == 4) {
            assert(rtps_data.data[j + 6] == 20);
        } else {
            assert(rtps_data.data[j + 6] == 0);
        }
    }

    CALL_SPDP_READER(out, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_2);
    assert(out.read_nb(rtps_data));
    assert(out.empty());
    assert(rtps_data.type == RTPS_TYPE_SPDP);
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
        assert(rtps_data.guid_prefix[j]
               == test_spdp_reader_data_2[j + RTPS_HDR_OFFSET_GUID_PREFIX]);
    }
    // Check IP address
    assert(rtps_data.data[0] == 192);
    assert(rtps_data.data[1] == 168);
    assert(rtps_data.data[2] == 0);
    assert(rtps_data.data[3] == 2);
    // Check UDP port
    assert(rtps_data.data[4] == (7410 >> 8));
    assert(rtps_data.data[5] == (7410 & 0xff));
    // Check participant lease duration
    for (auto j = 0; j < 8; j++) {
        if (j == 4) {
            assert(rtps_data.data[j + 6] == 100);
        } else {
            assert(rtps_data.data[j + 6] == 0);
        }
    }

    CALL_SPDP_READER(out, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_3);
    assert(out.read_nb(rtps_data));
    assert(out.empty());
    assert(rtps_data.type == RTPS_TYPE_SPDP);
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
        assert(rtps_data.guid_prefix[j]
               == test_spdp_reader_data_3[j + RTPS_HDR_OFFSET_GUID_PREFIX]);
    }
    // Check IP address
    assert(rtps_data.data[0] == 192);
    assert(rtps_data.data[1] == 168);
    assert(rtps_data.data[2] == 0);
    assert(rtps_data.data[3] == 2);
    // Check UDP port
    assert(rtps_data.data[4] == (7412 >> 8));
    assert(rtps_data.data[5] == (7412 & 0xff));
    // Check participant lease duration
    for (auto j = 0; j < 8; j++) {
        if (j == 4) {
            assert(rtps_data.data[j + 6] == 20);
        } else {
            assert(rtps_data.data[j + 6] == 0);
        }
    }

    CALL_SPDP_READER(out, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_4);
    assert(out.read_nb(rtps_data));
    assert(out.empty());
    assert(rtps_data.type == RTPS_TYPE_SPDP);
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
        assert(rtps_data.guid_prefix[j]
               == test_spdp_reader_data_4[j + RTPS_HDR_OFFSET_GUID_PREFIX]);
    }
    // Check IP address
    assert(rtps_data.data[0] == 192);
    assert(rtps_data.data[1] == 168);
    assert(rtps_data.data[2] == 0);
    assert(rtps_data.data[3] == 2);
    // Check UDP port
    assert(rtps_data.data[4] == (7412 >> 8));
    assert(rtps_data.data[5] == (7412 & 0xff));
    // Check participant lease duration
    for (auto j = 0; j < 8; j++) {
        if (j == 4) {
            assert(rtps_data.data[j + 6] == 20);
        } else {
            assert(rtps_data.data[j + 6] == 0);
        }
    }

    return 0;
}

static sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
static app_endpoint  app_reader_tbl[APP_READER_MAX];

static int test_spdp_reader_1() {
    constexpr uint8_t      ip_addr[4] = {192, 168, 0, 3};
    constexpr uint8_t      subnet_mask[4] = {255, 255, 255, 0};
    constexpr uint16_t     port_num_seed = 7400;
    constexpr int64_t      timestamp_i64 = 0;
    constexpr unsigned int n_patterns = (1 << MIN(SEDP_READER_MAX, 8));

    const hls_uint<PUB_TOPICS_MAX> pub_enable = 1;
    const hls_uint<SUB_TOPICS_MAX> sub_enable = 1;

    hls_stream<rtps_data_t> stream;
    CALL_SPDP_READER(stream, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_1);
    rtps_data_t rtps_data_1 = stream.read();

    // Test whether spdp_reader finds a new participant correctly.
    for (auto first_n_alive = 0; first_n_alive < SEDP_READER_MAX;
         first_n_alive++) {
        // Initialize sedp_reader_tbl
        for (auto j = 0; j < SEDP_READER_MAX; j++) {
            // Set liveliness
            sedp_reader_tbl[j].alive = (j < first_n_alive);
            // Set GUID prefix
            for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
                // If sedp_reader_tbl[j] is alive, set
                // sedp_reader_tbl[j].guid_prefix UNKNOWN. If sedp_reader_tbl[j]
                // is dead, set sedp_reader_tbl[j].guid_prefix
                // SOURCE_GUID_PREFIX. Test whether spdp_reader find a new
                // participant whose GUID prefix is the same as dead
                // participants'.
                sedp_reader_tbl[j].guid_prefix[k]
                    = (j < first_n_alive) ? 0 : SOURCE_GUID_PREFIX[k];
            }
        }
        // Update sedp_reader_tbl
        stream.write(rtps_data_1);
        ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
                timestamp_i64);
        // Check sedp_reader_tbl
        for (auto j = 0; j < SEDP_READER_MAX; j++) {
            // spdp_reader should find a new participant, and
            // sedp_reader_tbl[first_n_alive] should become alive.
            assert(sedp_reader_tbl[j].alive == (j <= first_n_alive));
        }
    }

    // Test whether spdp_reader ignore known participants collectly.
    for (auto known_participants = 1; known_participants < n_patterns;
         known_participants++) {
        // Initialize sedp_reader_tbl
        for (auto j = 0; j < SEDP_READER_MAX; j++) {
            bool known = ((1 << j) & known_participants);
            // Set liveliness
            sedp_reader_tbl[j].alive = known;
            // Set GUID prefix
            for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
                // If sedp_reader_tbl[j] is alive, set
                // sedp_reader_tbl[j].guid_prefix SOURCE_GUID_PREFIX. If
                // sedp_reader_tbl[j] is dead, set
                // sedp_reader_tbl[j].guid_prefix UNKNOWN.
                sedp_reader_tbl[j].guid_prefix[k]
                    = known ? SOURCE_GUID_PREFIX[k] : 0;
            }
        }
        // Update sedp_reader_tbl
        stream.write(rtps_data_1);
        ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
                timestamp_i64);
        // Check sedp_reader_tbl
        for (auto j = 0; j < SEDP_READER_MAX; j++) {
            bool known = ((1 << j) & known_participants);
            assert(sedp_reader_tbl[j].alive == known);
        }
    }

    return 0;
}

static int test_spdp_reader_2() {
    constexpr uint8_t  ip_addr[4] = {192, 168, 0, 3};
    constexpr uint8_t  subnet_mask[4] = {255, 255, 255, 0};
    constexpr uint16_t port_num_seed = 7400;

    const hls_uint<PUB_TOPICS_MAX> pub_enable = 1;
    const hls_uint<SUB_TOPICS_MAX> sub_enable = 1;

    hls_stream<rtps_data_t> stream;
    CALL_SPDP_READER(stream, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_1);
    rtps_data_t rtps_data_1 = stream.read();
    CALL_SPDP_READER(stream, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_2);
    rtps_data_t rtps_data_2 = stream.read();

    int64_t timestamp_i64;
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
    }

    // Test whether spdp_reader reads PID_PARTICIPANT_LEASE_DURATION and sets
    // timestamp correctly.
    // Initialize sedp_reader_tbl.
    timestamp_i64 = 0x0a00000000;
    sedp_reader_tbl[0].lease_duration = 0;
    sedp_reader_tbl[0].timestamp = 0;
    sedp_reader_tbl[0].alive = false;
    // Update sedp_reader_tbl
    stream.write(rtps_data_1);
    ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
            timestamp_i64);
    // Check sedp_reader_tbl.
    assert(sedp_reader_tbl[0].alive);
    assert(sedp_reader_tbl[0].lease_duration == 0x1400000000);
    assert(sedp_reader_tbl[0].timestamp == timestamp_i64);

    // Test whether spdp_reader reset lease_duration before read from the
    // parameters.
    // Initialize sedp_reader_tbl.
    timestamp_i64 = 0x0a00000000;
    sedp_reader_tbl[0].lease_duration = 0x7fffffffffffffff;
    sedp_reader_tbl[0].timestamp = 0;
    sedp_reader_tbl[0].alive = false;
    // Update sedp_reader_tbl
    stream.write(rtps_data_1);
    ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
            timestamp_i64);
    // Check sedp_reader_tbl.
    assert(sedp_reader_tbl[0].alive);
    assert(sedp_reader_tbl[0].lease_duration == 0x1400000000);
    assert(sedp_reader_tbl[0].timestamp == timestamp_i64);

    // Test whether spdp_reader sets default lease_duration value if SPDP
    // message does not have PID_PARTICIPANT_LEASE_DURATION.
    // Initialize sedp_reader_tbl.
    timestamp_i64 = 0x3200000000;
    sedp_reader_tbl[0].lease_duration = 0;
    sedp_reader_tbl[0].timestamp = 0;
    sedp_reader_tbl[0].alive = false;
    // Update sedp_reader_tbl
    stream.write(rtps_data_2);
    ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
            timestamp_i64);
    // Check sedp_reader_tbl.
    assert(sedp_reader_tbl[0].alive);
    assert(sedp_reader_tbl[0].lease_duration == SPDP_LEASE_DURATION_DEFAULT);
    assert(sedp_reader_tbl[0].timestamp == timestamp_i64);

    return 0;
}

static int test_spdp_reader_3() {
    // Test whether spdp_reader finds a new participant when ROS2rapper gets a
    // message with padding.
    constexpr uint8_t  ip_addr[4] = {192, 168, 0, 3};
    constexpr uint8_t  subnet_mask[4] = {255, 255, 255, 0};
    constexpr uint16_t port_num_seed = 7400;

    const hls_uint<PUB_TOPICS_MAX> pub_enable = 1;
    const hls_uint<SUB_TOPICS_MAX> sub_enable = 1;

    hls_stream<rtps_data_t> stream;
    CALL_SPDP_READER(stream, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_3);
    rtps_data_t rtps_data_3 = stream.read();
    CALL_SPDP_READER(stream, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_4);
    rtps_data_t rtps_data_4 = stream.read();

    int64_t timestamp_i64 = 0;

    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
    }
    // Update sedp_reader_tbl
    stream.write(rtps_data_3);
    ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
            timestamp_i64);
    // Test
    assert(sedp_reader_tbl[0].alive);

    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        sedp_reader_tbl[j].alive = false;
    }
    // Update sedp_reader_tbl
    stream.write(rtps_data_4);
    ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
            timestamp_i64);
    // Test
    assert(sedp_reader_tbl[0].alive);

    return 0;
}

static int test_update_timestamp() {
    // Test whether ros2_in updates timestamps in sedp_reader_tbl
    // correctly.
    constexpr uint8_t  ip_addr[4] = {192, 168, 0, 3};
    constexpr uint8_t  subnet_mask[4] = {255, 255, 255, 0};
    constexpr uint16_t port_num_seed = 7400;

    const hls_uint<PUB_TOPICS_MAX> pub_enable = 1;
    const hls_uint<SUB_TOPICS_MAX> sub_enable = 1;

    hls_stream<rtps_data_t> stream;
    CALL_SPDP_READER(stream, ip_addr, subnet_mask, port_num_seed,
                     test_spdp_reader_data_1);
    rtps_data_t rtps_data_1 = stream.read();

    for (auto target = 0; target < SEDP_READER_MAX; target++) {
        int64_t timestamp_i64_orig = 0;
        int64_t timestamp_i64_new = static_cast<int64_t>(target + 1) << 32;
        // Initialize sedp_reader_tbl.
        for (auto j = 0; j < SEDP_READER_MAX; j++) {
            sedp_reader_tbl[j].timestamp = timestamp_i64_orig;
            sedp_reader_tbl[j].alive = true;
            for (auto k = 0; k < GUID_PREFIX_SIZE; k++) {
                sedp_reader_tbl[j].guid_prefix[k]
                    = (j == target) ? test_spdp_reader_data_1
                                          [k + RTPS_HDR_OFFSET_GUID_PREFIX]
                                    : 0;
            }
        }
        // Update sedp_reader_tbl
        stream.write(rtps_data_1);
        ros2_in(stream, sedp_reader_tbl, app_reader_tbl, pub_enable, sub_enable,
                timestamp_i64_new);
        // Check sedp_reader_tbl.
        for (auto j = 0; j < SEDP_READER_MAX; j++) {
            if (j == target) {
                assert(sedp_reader_tbl[j].timestamp == timestamp_i64_new);
            } else {
                assert(sedp_reader_tbl[j].timestamp == timestamp_i64_orig);
            }
        }
    }
    return 0;
}

int test_spdp_reader() {
    assert(test_spdp_reader_0() == 0);
    assert(test_spdp_reader_1() == 0);
    assert(test_spdp_reader_2() == 0);
    assert(test_spdp_reader_3() == 0);
    assert(test_update_timestamp() == 0);
    return 0;
}
