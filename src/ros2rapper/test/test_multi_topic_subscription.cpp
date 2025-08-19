#include "app.hpp"
#include <cassert>
#include <iostream>

// The recipient's reader entity ID: {0x00, 0x00, 0x10, 0x04}.
constexpr uint8_t app_reader_test_data_0[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xa9, 0x37, 0x00, 0x00, 0x00, 0x00,
    // INFO_DST submessage
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // INFO_TS submessage
    0x09, 0x01, 0x08, 0x00, 0x5e, 0xce, 0x37, 0x68, 0x6e, 0x59, 0x81, 0x0c,
    // INFO_DATA submessage
    0x15, 0x05, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00,
    // Reader entity ID
    0x00, 0x00, 0x10, 0x04,
    // Writer entity ID
    0x00, 0x00, 0x10, 0x03,
    // Sequence number
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Payload header
    0x00, 0x01, 0x00, 0x00,
    // Payload
    0x02, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00};

// The recipient's reader entity ID: {0x00, 0x00, 0x11, 0x04}.
constexpr uint8_t app_reader_test_data_1[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xa9, 0x37, 0x00, 0x00, 0x00, 0x00,
    // INFO_DST submessage
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // INFO_TS submessage
    0x09, 0x01, 0x08, 0x00, 0x5e, 0xce, 0x37, 0x68, 0x6e, 0x59, 0x81, 0x0c,
    // INFO_DATA submessage
    0x15, 0x05, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00,
    // Reader entity ID
    0x00, 0x00, 0x11, 0x04,
    // Writer entity ID
    0x00, 0x00, 0x11, 0x03,
    // Sequence number
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Payload header
    0x00, 0x01, 0x00, 0x00,
    // Payload
    0x03, 0x00, 0x00, 0x00, 0x42, 0x42, 0x00, 0x00};

// The recipient's reader entity ID: {0x00, 0x00, 0x12, 0x04}.
constexpr uint8_t app_reader_test_data_2[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xa9, 0x37, 0x00, 0x00, 0x00, 0x00,
    // INFO_DST submessage
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // INFO_TS submessage
    0x09, 0x01, 0x08, 0x00, 0x5e, 0xce, 0x37, 0x68, 0x6e, 0x59, 0x81, 0x0c,
    // INFO_DATA submessage
    0x15, 0x05, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00,
    // Reader entity ID
    0x00, 0x00, 0x12, 0x04,
    // Writer entity ID
    0x00, 0x00, 0x12, 0x03,
    // Sequence number
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Payload header
    0x00, 0x01, 0x00, 0x00,
    // Payload
    0x04, 0x00, 0x00, 0x00, 0x43, 0x43, 0x43, 0x00};

// The recipient's reader entity ID: {0x00, 0x00, 0x13, 0x04}.
constexpr uint8_t app_reader_test_data_3[] = {
    // RTPS header
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x9c, 0x9d,
    0x4a, 0x00, 0xa9, 0x37, 0x00, 0x00, 0x00, 0x00,
    // INFO_DST submessage
    0x0e, 0x01, 0x0c, 0x00, 0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    // INFO_TS submessage
    0x09, 0x01, 0x08, 0x00, 0x5e, 0xce, 0x37, 0x68, 0x6e, 0x59, 0x81, 0x0c,
    // INFO_DATA submessage
    0x15, 0x05, 0x24, 0x00, 0x00, 0x00, 0x10, 0x00,
    // Reader entity ID
    0x00, 0x00, 0x13, 0x04,
    // Writer entity ID
    0x00, 0x00, 0x13, 0x03,
    // Sequence number
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    // Payload header
    0x00, 0x01, 0x00, 0x00,
    // Payload
    0x05, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00};

static void call_app_reader(
    const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE],
    const uint8_t reader_entity_id_list[SUB_TOPICS_MAX][GUID_ENTITYID_SIZE],
    hls_uint<SUB_TOPICS_MAX>  sub_enable,
    hls_uint<SUB_TOPICS_MAX> *sub_app_data_recv,
    hls_uint<SUB_TOPICS_MAX> *sub_app_data_req,
    hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel,
    hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant,
    uint8_t                   sub_app_data_0[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_1[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_2[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_3[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_len[SUB_TOPICS_MAX],
    uint16_t sub_app_data_rep_id[SUB_TOPICS_MAX], const uint8_t test_data[],
    size_t test_data_len) {
    for (auto j = 0; j < test_data_len; j++) {
        hls_uint<9> x = test_data[j];
        if (j == (test_data_len - 1)) {
            x |= hls_uint<9>(0x100);
        }
        app_reader(x, reader_guid_prefix, reader_entity_id_list, sub_enable,
                   sub_app_data_recv, sub_app_data_req, sub_app_data_rel,
                   &sub_app_data_grant, sub_app_data_0, sub_app_data_1,
                   sub_app_data_2, sub_app_data_3, sub_app_data_len,
                   sub_app_data_rep_id);
    }
}

static int check_app_reader(hls_uint<SUB_TOPICS_MAX> sub_enable,
                            hls_uint<SUB_TOPICS_MAX> sub_app_data_recv,
                            hls_uint<SUB_TOPICS_MAX> sub_app_data_req,
                            hls_uint<SUB_TOPICS_MAX> sub_app_data_rel,
                            hls_uint<SUB_TOPICS_MAX> sub_app_data_grant,
                            const uint8_t sub_app_data[MAX_APP_DATA_LEN],
                            uint8_t       sub_app_data_len,
                            hls_uint<SUB_TOPICS_MAX> flag,
                            const uint8_t            test_data[],
                            uint8_t                  test_data_payload_len,
                            int16_t                  test_data_payload_offset) {
    assert(sub_app_data_recv == (sub_enable & sub_app_data_grant & flag));
    assert(sub_app_data_req == (sub_enable & flag));
    assert(sub_app_data_rel == (sub_enable & sub_app_data_grant & flag));
    if ((sub_enable & sub_app_data_grant & flag) != 0) {
        // The topic is enabled and granted.
        if (sub_app_data_len != test_data_payload_len) {
            std::cout << "ROS2rapper received wrong app data length "
                      << static_cast<int>(sub_app_data_len) << '.' << std::endl;
            return 3;
        }
        for (auto j = 0; j < test_data_payload_len; j++) {
            if (sub_app_data[j] != test_data[j + test_data_payload_offset]) {
                std::cout << "ROS2rapper received wrong data at " << j << '.'
                          << std::endl;
                return 4;
            }
        }
    }
    return 0;
}

#define CALL_APP_READER(test_data)                                             \
    do {                                                                       \
        sub_app_data_recv = 0;                                                 \
        sub_app_data_req = 0;                                                  \
        sub_app_data_rel = 0;                                                  \
        call_app_reader(reader_guid_prefix, reader_entity_id_list, sub_enable, \
                        &sub_app_data_recv, &sub_app_data_req,                 \
                        &sub_app_data_rel, sub_app_data_grant, sub_app_data_0, \
                        sub_app_data_1, sub_app_data_2, sub_app_data_3,        \
                        sub_app_data_len, sub_app_data_rep_id, test_data,      \
                        sizeof(test_data));                                    \
    } while (0)

#define CHECK_APP_READER(flag, sub_app_data, sub_app_data_len, test_data,      \
                         test_data_payload_len, test_data_payload_offset)      \
    assert(check_app_reader(sub_enable, sub_app_data_recv, sub_app_data_req,   \
                            sub_app_data_rel, sub_app_data_grant,              \
                            sub_app_data, sub_app_data_len, flag, test_data,   \
                            test_data_payload_len, test_data_payload_offset)   \
           == 0)

static int test_app_reader() {
    constexpr uint8_t reader_guid_prefix[GUID_PREFIX_SIZE] = {
        0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    constexpr uint8_t reader_entity_id_list[SUB_TOPICS_MAX][GUID_ENTITYID_SIZE]
        = {
            {0x00, 0x00, 0x10, 0x04},
            {0x00, 0x00, 0x11, 0x04},
            {0x00, 0x00, 0x12, 0x04},
            {0x00, 0x00, 0x13, 0x04}
    };

    hls_uint<SUB_TOPICS_MAX> sub_enable;
    hls_uint<SUB_TOPICS_MAX> sub_app_data_recv;
    hls_uint<SUB_TOPICS_MAX> sub_app_data_req;
    hls_uint<SUB_TOPICS_MAX> sub_app_data_rel;
    hls_uint<SUB_TOPICS_MAX> sub_app_data_grant;

    uint8_t  sub_app_data_0[MAX_APP_DATA_LEN];
    uint8_t  sub_app_data_1[MAX_APP_DATA_LEN];
    uint8_t  sub_app_data_2[MAX_APP_DATA_LEN];
    uint8_t  sub_app_data_3[MAX_APP_DATA_LEN];
    uint8_t  sub_app_data_len[SUB_TOPICS_MAX];
    uint16_t sub_app_data_rep_id[SUB_TOPICS_MAX];

    unsigned int n_patterns = (1 << SUB_TOPICS_MAX);
    for (unsigned int sub_enable_pattern = 0; sub_enable_pattern < n_patterns;
         sub_enable_pattern++) {
        for (unsigned int sub_app_data_grant_pattern = 0;
             sub_app_data_grant_pattern < n_patterns;
             sub_app_data_grant_pattern++) {
            sub_enable = sub_enable_pattern;
            sub_app_data_grant = sub_app_data_grant_pattern;
            int16_t test_data_payload_offset = 76;

            // Reader entity ID: {0x00, 0x00, 0x10, 0x04}
            CALL_APP_READER(app_reader_test_data_0);
            CHECK_APP_READER(1, sub_app_data_0, sub_app_data_len[0],
                             app_reader_test_data_0, 8,
                             test_data_payload_offset);

            // Reader entity ID: {0x00, 0x00, 0x11, 0x04}
            CALL_APP_READER(app_reader_test_data_1);
            CHECK_APP_READER(2, sub_app_data_1, sub_app_data_len[1],
                             app_reader_test_data_1, 8,
                             test_data_payload_offset);

            // Reader entity ID: {0x00, 0x00, 0x12, 0x04}
            CALL_APP_READER(app_reader_test_data_2);
            CHECK_APP_READER(4, sub_app_data_2, sub_app_data_len[2],
                             app_reader_test_data_2, 8,
                             test_data_payload_offset);

            // Reader entity ID: {0x00, 0x00, 0x13, 0x04}
            CALL_APP_READER(app_reader_test_data_3);
            CHECK_APP_READER(8, sub_app_data_3, sub_app_data_len[3],
                             app_reader_test_data_3, 12,
                             test_data_payload_offset);
        }
    }

    return 0;
}

int test_multi_topic_subscription() {
    assert(test_app_reader() == 0);
    return 0;
}
