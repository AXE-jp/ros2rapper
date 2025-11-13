#include "test_utils.hpp"
#include "endpoint.hpp"
#include "ros2.hpp"
#include "rtps.hpp"
#include <cstdint>

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

bool is_sedp_endpoint_alive(const sedp_reader_tbl_t *tbl, unsigned int idx) {
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, idx, 0);
    return ((data & SEDP_ENDPOINT_ALIVE) != 0);
}

void get_sedp_reader_tbl_flags(uint8_t *flags, const sedp_reader_tbl_t *tbl,
                               unsigned int idx) {
    uint64_t data;
    get_sedp_reader_tbl(&data, tbl, idx, 0);
    *flags = data & 0xff;
}

void set_sedp_reader_tbl_liveliness_and_guid_prefix(
    bool alive, const uint8_t guid_prefix[12], sedp_reader_tbl_t *tbl,
    unsigned int idx) {
    uint64_t wdata_0 = alive ? SEDP_ENDPOINT_ALIVE : 0;
    uint64_t wdata_1 = 0;
    for (auto j = 0; j < 4; j++) {
        wdata_0 |= static_cast<uint64_t>(guid_prefix[j]) << (8 * (j + 4));
    }
    for (auto j = 0; j < 8; j++) {
        wdata_1 |= static_cast<uint64_t>(guid_prefix[j + 4]) << (8 * j);
    }
    set_sedp_reader_tbl(wdata_0, tbl, idx, 0);
    set_sedp_reader_tbl(wdata_1, tbl, idx, 1);
}

void set_sedp_reader_tbl_liveliness_and_guid_prefix_unknown(
    bool alive, sedp_reader_tbl_t *tbl, unsigned int idx) {
    set_sedp_reader_tbl(alive ? SEDP_ENDPOINT_ALIVE : 0, tbl, idx, 0);
    set_sedp_reader_tbl(0, tbl, idx, 1);
}

void get_sedp_reader_tbl_children(hls_uint<APP_READER_MAX> *children,
                                  const sedp_reader_tbl_t  *tbl,
                                  unsigned int              idx) {
    uint64_t data_9, data_10;
    get_sedp_reader_tbl(&data_9, tbl, idx, 9);
    get_sedp_reader_tbl(&data_10, tbl, idx, 10);
    *children = data_9 | (hls_uint<APP_READER_MAX>(data_10) << 64);
}

void set_sedp_reader_tbl_children(hls_uint<APP_READER_MAX> children,
                                  sedp_reader_tbl_t *tbl, unsigned int idx) {
    set_sedp_reader_tbl(children & 0xffffffffffffffff, tbl, idx, 9);
    set_sedp_reader_tbl(children >> 64, tbl, idx, 10);
}
