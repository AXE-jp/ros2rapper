// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove_endpoints.hpp"
#include <cassert>
#include <cstdio>

// Heartbeat
constexpr uint8_t test_update_liveliness_alive_data_1[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x37, 0xad,
    0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Submessage (INFO_DST)
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00,
    // Submessage (HEARTBEAT)
    0x07, 0x01, 0x1c, 0x00, 0x00, 0x00, 0x03, 0xc7, 0x00, 0x00, 0x03, 0xc2,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

// Data
constexpr uint8_t test_update_liveliness_alive_data_2[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x37, 0xad,
    0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Submessage (INFO_DST)
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_TS)
    0x09, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // Submessage (DATA)
    0x15, 0x05, 0x34, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x11, 0x04,
    0x00, 0x00, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x4d, 0x65, 0x73, 0x73,
    0x61, 0x67, 0x65, 0x20, 0x46, 0x72, 0x6f, 0x6d, 0x20, 0x46, 0x50, 0x47,
    0x41, 0x20, 0x2d, 0x20, 0x35, 0x00, 0x00, 0x00};

// Inline QoS with alive status info
constexpr uint8_t test_update_liveliness_alive_data_3[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0x03, 0x5b, 0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_TS)
    0x09, 0x01, 0x08, 0x00, 0xc5, 0xd2, 0x26, 0x68, 0x4f, 0xfc, 0xb3, 0xa9,
    // Submessage (DATA)
    0x15, 0x03, 0x50, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    // Parameter List
    // UNKNOWN
    0x0f, 0x80, 0x18, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // PID_KEY_HASH (wrong?)
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_STATUS_INFO
    0x71, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// Inline QoS with dead status info without INFO_DST to another endpoint
constexpr uint8_t test_update_liveliness_alive_data_4[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0x03, 0x5b, 0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_DST)
    0x0e, 0x01, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_TS)
    0x09, 0x01, 0x08, 0x00, 0xc5, 0xd2, 0x26, 0x68, 0x4f, 0xfc, 0xb3, 0xa9,
    // Submessage (DATA)
    0x15, 0x03, 0x50, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    // Parameter List
    // UNKNOWN
    0x0f, 0x80, 0x18, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // PID_KEY_HASH (wrong?)
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_STATUS_INFO
    0x71, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// Inline QoS with dead status info without INFO_DST
constexpr uint8_t test_update_liveliness_dead_data_1[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0x03, 0x5b, 0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_TS)
    0x09, 0x01, 0x08, 0x00, 0xc5, 0xd2, 0x26, 0x68, 0x4f, 0xfc, 0xb3, 0xa9,
    // Submessage (DATA)
    0x15, 0x03, 0x50, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7,
    0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    // Parameter List
    // UNKNOWN
    0x0f, 0x80, 0x18, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // PID_KEY_HASH
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_STATUS_INFO
    0x71, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// Inline QoS with dead status info to the ros2rapper
constexpr uint8_t test_update_liveliness_dead_data_2[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0x03, 0x5b, 0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_DST)
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_TS)
    0x09, 0x01, 0x08, 0x00, 0xc5, 0xd2, 0x26, 0x68, 0x4f, 0xfc, 0xb3, 0xa9,
    // Submessage (DATA)
    0x15, 0x03, 0x34, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0xc7,
    0x00, 0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    // Parameter List
    // PID_KEY_HASH (wrong?)
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_STATUS_INFO
    0x71, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00};

// Inline QoS with dead status info to another endpoint,
// and INFO_DST appears after inline QoS.
constexpr uint8_t test_update_liveliness_dead_data_3[] = {
    // RTPS Header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0x03, 0x5b, 0x00, 0x00, 0x00, 0x00,
    // Submessage (INFO_TS)
    0x09, 0x01, 0x08, 0x00, 0xc5, 0xd2, 0x26, 0x68, 0x4f, 0xfc, 0xb3, 0xa9,
    // Submessage (DATA)
    0x15, 0x03, 0x34, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0xc7,
    0x00, 0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    // Parameter List
    // PID_KEY_HASH (wrong?)
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    // PID_STATUS_INFO
    0x71, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03,
    // PID_SENTINEL
    0x01, 0x00, 0x00, 0x00,
    // Submessage (INFO_DST)
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b,
    0x00, 0x00, 0x00, 0x00};

constexpr uint8_t test_update_liveliness_guid_prefix[GUID_PREFIX_SIZE]
    = {0x01, 0x0f, 0x9c, 0x9d, 0x4a, 0x00, 0x03, 0x5b, 0x00, 0x00, 0x00, 0x00};

void setup_sedp_reader_tbl(unsigned int mask, const uint8_t test_data[],
                           sedp_endpoint tbl[SEDP_READER_MAX]) {
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        bool jth_selected = ((1 << j) & mask);
        // setup GUID prefix
        for (auto k = RTPS_HDR_OFFSET_GUID_PREFIX; k < RTPS_HDR_SIZE; k++) {
            uint8_t data;
            if (jth_selected) {
                // Copy GUID prefix from test_data.
                data = test_data[k];
            } else {
                // set GUID prefix to UNKNOWN.
                data = 0;
            }
            tbl[j].guid_prefix[k - RTPS_HDR_OFFSET_GUID_PREFIX] = data;
        }
        // setup liveliness
        tbl[j].alive = true;
    }
}

int check_sedp_reader_tbl_liveliness(unsigned int  mask,
                                     sedp_endpoint tbl[SEDP_READER_MAX]) {
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        bool jth_selected = ((1 << j) & mask);
        if (jth_selected) {
            // tbl[j] shoud be alive.
            if (tbl[j].alive != true) {
                printf("check_sedp_reader_tbl_alive: %d is not alive.\n", j);
                return 1;
            }
        } else {
            // tbl[j] should be dead.
            if (tbl[j].alive != false) {
                printf("check_sedp_reader_tbl_alive: %d is not dead.\n", j);
                return 1;
            }
        }
    }
    return 0;
}

void setup_app_reader_tbl(unsigned int mask, const uint8_t test_data[],
                          app_endpoint tbl[APP_READER_MAX]) {
    for (auto j = 0; j < APP_READER_MAX; j++) {
        bool jth_selected = ((1 << j) & mask);
        // setup GUID prefix
        for (auto k = RTPS_HDR_OFFSET_GUID_PREFIX; k < RTPS_HDR_SIZE; k++) {
            uint8_t data;
            if (jth_selected) {
                // Copy GUID prefix from test_data.
                data = test_data[k];
            } else {
                // set GUID prefix to UNKNOWN.
                data = 0;
            }
            tbl[j].guid_prefix[k - RTPS_HDR_OFFSET_GUID_PREFIX] = data;
        }
        // setup liveliness
        tbl[j].alive = true;
    }
}

int check_app_reader_tbl_liveliness(unsigned int mask,
                                    app_endpoint tbl[APP_READER_MAX]) {
    for (auto j = 0; j < APP_READER_MAX; j++) {
        bool jth_selected = ((1 << j) & mask);
        if (jth_selected) {
            // tbl[j] shoud be alive.
            if (tbl[j].alive != true) {
                printf("check_app_reader_tbl_alive: %d is not alive.\n", j);
                return 1;
            }
        } else {
            // tbl[j] should be dead.
            if (tbl[j].alive != false) {
                printf("check_app_reader_tbl_alive: %d is not dead.\n", j);
                return 1;
            }
        }
    }
    return 0;
}

int call_update_liveliness(const config_t *conf,
                           sedp_endpoint   sedp_reader_tbl[SEDP_READER_MAX],
                           app_endpoint    app_reader_tbl[APP_READER_MAX],
                           const uint8_t test_data[], size_t length) {
    bool reading_rtps_message = false;
    for (auto j = 0; j < length; j++) {
        hls_uint<9> data = test_data[j];
        if (j == (length - 1)) {
            data |= hls_uint<9>(0x100);
        }
        update_liveliness(data, conf, sedp_reader_tbl, app_reader_tbl,
                          &reading_rtps_message);
        // check reading_rtps_message
        if (j < RTPS_HDR_OFFSET_GUID_PREFIX) {
            if (reading_rtps_message != false) {
                puts("reading_rtps_message becomes true before the message "
                     "reaches the GUID prefix.");
                return 1;
            }
        } else if (j < (length - 1)) {
            if (reading_rtps_message != true) {
                puts("reading_rtps_message becomes false while reading RTPS "
                     "message.");
                return 1;
            }
        } else {
            if (reading_rtps_message != false) {
                puts("reading_rtps_message does not become false at the end of "
                     "the RTPS message.");
                return 1;
            }
        }
    }
    return 0;
}

int test_update_liveliness_1(const config_t *conf, unsigned int sedp_set_mask,
                             unsigned int  sedp_check_mask,
                             unsigned int  app_set_mask,
                             unsigned int  app_check_mask,
                             const uint8_t test_data[], size_t test_data_length,
                             const char *test_data_name) {
    // setup reader tables
    sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
    setup_sedp_reader_tbl(sedp_set_mask, test_data, sedp_reader_tbl);
    app_endpoint app_reader_tbl[APP_READER_MAX];
    setup_app_reader_tbl(app_set_mask, test_data, app_reader_tbl);

    // process test_data
    assert(call_update_liveliness(conf, sedp_reader_tbl, app_reader_tbl,
                                  test_data, test_data_length)
           == 0);

    // check liveliness
    int result[2];
    result[0]
        = check_sedp_reader_tbl_liveliness(sedp_check_mask, sedp_reader_tbl);
    result[1] = check_app_reader_tbl_liveliness(app_check_mask, app_reader_tbl);
    if ((result[0] == 0) && (result[1] == 0)) {
        return 0;
    } else {
        printf("%s: %x %x %x %x\n", test_data_name, sedp_set_mask,
               sedp_check_mask, app_set_mask, app_check_mask);
        return 1;
    }
}

#define TEST_UPDATE_LIVELINESS(conf, sedp_set_mask, sedp_check_mask,           \
                               app_set_mask, app_check_mask, test_data)        \
    test_update_liveliness_1(conf, sedp_set_mask, sedp_check_mask,             \
                             app_set_mask, app_check_mask, test_data,          \
                             sizeof(test_data), #test_data)

int test_update_liveliness() {
    config_t conf;
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
        conf.guid_prefix[j] = test_update_liveliness_guid_prefix[j];
    }
    unsigned int n_sedp_patterns = (1 << SEDP_READER_MAX);
    unsigned int n_app_patterns = (1 << APP_READER_MAX);
    for (unsigned int sedp_mask = 0; sedp_mask < n_sedp_patterns; sedp_mask++) {
        for (unsigned int app_mask = 0; app_mask < n_app_patterns; app_mask++) {
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~0, app_mask, ~0,
                                   test_update_liveliness_alive_data_1);
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~0, app_mask, ~0,
                                   test_update_liveliness_alive_data_2);
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~0, app_mask, ~0,
                                   test_update_liveliness_alive_data_3);
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~0, app_mask, ~0,
                                   test_update_liveliness_alive_data_4);
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~sedp_mask, app_mask,
                                   ~app_mask,
                                   test_update_liveliness_dead_data_1);
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~sedp_mask, app_mask,
                                   ~app_mask,
                                   test_update_liveliness_dead_data_2);
            TEST_UPDATE_LIVELINESS(&conf, sedp_mask, ~sedp_mask, app_mask,
                                   ~app_mask,
                                   test_update_liveliness_dead_data_3);
        }
    }
    return 0;
}

int test_collect_dead_endpoint_1(unsigned int n_sedp_readers_allocated,
                                 unsigned int first_n_sedp_readers_alive,
                                 unsigned int n_app_readers_allocated,
                                 unsigned int first_n_app_readers_alive) {
    // setup sedp_reader_tbl and sedp_{pub,sub}_{heartbeat,acknack}_cnt
    sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
    uint32_t      sedp_pub_heartbeat_cnt[SEDP_READER_MAX];
    uint32_t      sedp_sub_heartbeat_cnt[SEDP_READER_MAX];
    uint32_t      sedp_pub_acknack_cnt[SEDP_READER_MAX];
    uint32_t      sedp_sub_acknack_cnt[SEDP_READER_MAX];
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        // sedp_endpoint
        sedp_reader_tbl[j].ip_addr[0] = j;
        sedp_reader_tbl[j].alive = (j < first_n_sedp_readers_alive);
        // counters
        sedp_pub_heartbeat_cnt[j] = j;
        sedp_sub_heartbeat_cnt[j] = j + SEDP_READER_MAX;
        sedp_pub_acknack_cnt[j] = j + 2 * SEDP_READER_MAX;
        sedp_sub_acknack_cnt[j] = j + 3 * SEDP_READER_MAX;
    }
    // setup app_reader_tbl
    app_endpoint app_reader_tbl[APP_READER_MAX];
    for (auto j = 0; j < APP_READER_MAX; j++) {
        app_reader_tbl[j].ip_addr[0] = j;
        app_reader_tbl[j].alive = (j < first_n_app_readers_alive);
    }

    // call
    sedp_reader_id_t sedp_reader_cnt = n_sedp_readers_allocated;
    app_reader_id_t  app_reader_cnt = n_app_readers_allocated;
    collect_dead_endpoint(0, sedp_reader_cnt, sedp_reader_tbl, app_reader_cnt,
                          app_reader_tbl, sedp_pub_heartbeat_cnt,
                          sedp_sub_heartbeat_cnt, sedp_pub_acknack_cnt,
                          sedp_sub_acknack_cnt);
    collect_dead_endpoint(1, sedp_reader_cnt, sedp_reader_tbl, app_reader_cnt,
                          app_reader_tbl, sedp_pub_heartbeat_cnt,
                          sedp_sub_heartbeat_cnt, sedp_pub_acknack_cnt,
                          sedp_sub_acknack_cnt);
    collect_dead_endpoint(2, sedp_reader_cnt, sedp_reader_tbl, app_reader_cnt,
                          app_reader_tbl, sedp_pub_heartbeat_cnt,
                          sedp_sub_heartbeat_cnt, sedp_pub_acknack_cnt,
                          sedp_sub_acknack_cnt);
    collect_dead_endpoint(3, sedp_reader_cnt, sedp_reader_tbl, app_reader_cnt,
                          app_reader_tbl, sedp_pub_heartbeat_cnt,
                          sedp_sub_heartbeat_cnt, sedp_pub_acknack_cnt,
                          sedp_sub_acknack_cnt);

    // check sedp_reader_cnt
    if (first_n_sedp_readers_alive >= n_sedp_readers_allocated) {
        if (sedp_reader_cnt != n_sedp_readers_allocated) {
            puts("collect_dead_endpoint: sedp_reader_cnt should not change.");
            return 1;
        }
    } else {
        if (sedp_reader_cnt != (n_sedp_readers_allocated - 1)) {
            puts("collect_dead_endpoint: sedp_reader_cnt should be "
                 "decremented.");
            return 1;
        }
    }
    // check app_reader_cnt
    if (first_n_app_readers_alive >= n_app_readers_allocated) {
        if (app_reader_cnt != n_app_readers_allocated) {
            puts("collect_dead_endpoint: app_reader_cnt should not change.");
            return 1;
        }
    } else {
        if (app_reader_cnt != (n_app_readers_allocated - 1)) {
            puts(
                "collect_dead_endpoint: app_reader_cnt should be decremented.");
            return 1;
        }
    }
    // check sedp_reader_tbl and sedp_{pub,sub}_{heartbeat,acknack}_cnt
    for (auto j = 0; j < SEDP_READER_MAX; j++) {
        uint32_t expected;
        if ((first_n_sedp_readers_alive <= j)
            && ((j + 1) < n_sedp_readers_allocated)) {
            expected = j + 1;
        } else {
            expected = j;
        }
        if ((sedp_reader_tbl[j].ip_addr[0] != expected)
            || (sedp_pub_heartbeat_cnt[j] != expected)
            || (sedp_sub_heartbeat_cnt[j] != (expected + SEDP_READER_MAX))
            || (sedp_pub_acknack_cnt[j] != (expected + 2 * SEDP_READER_MAX))
            || (sedp_sub_acknack_cnt[j] != (expected + 3 * SEDP_READER_MAX))) {
            puts("collect_dead_endpoint: fail to copy sedp tables.");
            return 1;
        }
    }
    // check app_reader_tbl
    for (auto j = 0; j < APP_READER_MAX; j++) {
        uint32_t expected;
        if ((first_n_app_readers_alive <= j)
            && ((j + 1) < n_app_readers_allocated)) {
            expected = j + 1;
        } else {
            expected = j;
        }
        if (app_reader_tbl[j].ip_addr[0] != expected) {
            puts("collect_dead_endpoint: fail to copy app_reader_tbl.");
            return 1;
        }
    }
    return 0;
}

int test_collect_dead_endpoint() {
    for (unsigned int n_sedp_readers_allocated = 0;
         n_sedp_readers_allocated <= SEDP_READER_MAX;
         n_sedp_readers_allocated++) {
        for (unsigned int first_n_sedp_readers_alive = 0;
             first_n_sedp_readers_alive <= n_sedp_readers_allocated;
             first_n_sedp_readers_alive++) {
            for (unsigned int n_app_readers_allocated = 0;
                 n_app_readers_allocated <= APP_READER_MAX;
                 n_app_readers_allocated++) {
                for (unsigned int first_n_app_readers_alive = 0;
                     first_n_app_readers_alive <= n_app_readers_allocated;
                     first_n_app_readers_alive++) {
                    assert(
                        test_collect_dead_endpoint_1(n_sedp_readers_allocated,
                                                     first_n_sedp_readers_alive,
                                                     n_app_readers_allocated,
                                                     first_n_app_readers_alive)
                        == 0);
                }
            }
        }
    }
    return 0;
}

int main() {
    assert(test_update_liveliness() == 0);
    assert(test_collect_dead_endpoint() == 0);
    return 0;
}
