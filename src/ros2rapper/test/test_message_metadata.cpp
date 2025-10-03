// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "duration.hpp"
#include "message_metadata.hpp"
#include <cassert>
#include <cstdio>

static int test_spdp_metadata() {
    const uint8_t  metatraffic_port[2] = {1, 2};
    const uint8_t  default_port[2] = {3, 4};
    const duration lease_duration
        = {.seconds = 0x01234567, .fraction = 0x89abcdef};

    message_metadata_t msg_metadata;
    uint8_t            metatraffic_port_out[2];
    uint8_t            default_port_out[2];
    duration           lease_duration_out;

    serialize_spdp_metadata(metatraffic_port, default_port, lease_duration,
                            &msg_metadata);
    deserialize_spdp_metadata(metatraffic_port_out, default_port_out,
                              &lease_duration_out, &msg_metadata);

    assert(metatraffic_port_out[0] == metatraffic_port[0]);
    assert(metatraffic_port_out[1] == metatraffic_port[1]);
    assert(default_port_out[0] == default_port[0]);
    assert(default_port_out[1] == default_port[1]);
    assert(lease_duration_out.seconds == lease_duration.seconds);
    assert(lease_duration_out.fraction == lease_duration.fraction);

    return 0;
}

static int test_sedp_metadata() {
    const uint8_t reader_guid_prefix[12]
        = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    const int64_t seqnum = 0x02468ace13579bdf;
    const uint8_t usertraffic_port[2] = {17, 18};
    const uint8_t app_entity_id[4] = {19, 20, 21, 22};

    message_metadata_t msg_metadata;
    uint8_t            reader_guid_prefix_out[12];
    int64_t            seqnum_out;
    uint8_t            usertraffic_port_out[2];
    uint8_t            app_entity_id_out[4];

    serialize_sedp_metadata(reader_guid_prefix, seqnum, usertraffic_port,
                            app_entity_id, &msg_metadata);
    deserialize_sedp_metadata(reader_guid_prefix_out, &seqnum_out,
                              usertraffic_port_out, app_entity_id_out,
                              &msg_metadata);

    for (auto i = 0; i < 12; i++) {
        assert(reader_guid_prefix_out[i] == reader_guid_prefix[i]);
    }
    assert(seqnum_out == seqnum);
    assert(usertraffic_port_out[0] == usertraffic_port[0]);
    assert(usertraffic_port_out[1] == usertraffic_port[1]);
    for (auto i = 0; i < 4; i++) {
        assert(app_entity_id_out[i] == app_entity_id[i]);
    }

    return 0;
}

static int test_sedp_heartbeat_metadata() {
    const uint8_t reader_guid_prefix[12]
        = {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
    const int64_t  first_seqnum = 0x76543210fedcba98;
    const int64_t  last_seqnum = 0x48c059d16ae27bf3;
    const uint32_t cnt = 0x01234567;

    message_metadata_t msg_metadata;
    uint8_t            reader_guid_prefix_out[12];
    int64_t            first_seqnum_out;
    int64_t            last_seqnum_out;
    uint32_t           cnt_out;

    serialize_sedp_heartbeat_metadata(reader_guid_prefix, first_seqnum,
                                      last_seqnum, cnt, &msg_metadata);
    deserialize_sedp_heartbeat_metadata(reader_guid_prefix_out,
                                        &first_seqnum_out, &last_seqnum_out,
                                        &cnt_out, &msg_metadata);

    for (auto i = 0; i < 12; i++) {
        assert(reader_guid_prefix_out[i] == reader_guid_prefix[i]);
    }
    assert(first_seqnum_out == first_seqnum);
    assert(last_seqnum_out == last_seqnum);
    assert(cnt_out == cnt);

    return 0;
}

static int test_sedp_acknack_metadata(bool snstate_empty) {
    const uint8_t reader_guid_prefix[12]
        = {35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46};
    const uint8_t  snstate_base = 47;
    const uint32_t cnt = 0x89abcdef;

    message_metadata_t msg_metadata;
    uint8_t            reader_guid_prefix_out[12];
    uint8_t            snstate_base_out;
    bool               snstate_empty_out;
    uint32_t           cnt_out;

    serialize_sedp_acknack_metadata(reader_guid_prefix, snstate_base,
                                    snstate_empty, cnt, &msg_metadata);
    deserialize_sedp_acknack_metadata(reader_guid_prefix_out, &snstate_base_out,
                                      &snstate_empty_out, &cnt_out,
                                      &msg_metadata);

    for (auto i = 0; i < 12; i++) {
        assert(reader_guid_prefix_out[i] == reader_guid_prefix[i]);
    }
    assert(snstate_base_out == snstate_base);
    assert(snstate_empty_out == snstate_empty);
    assert(cnt_out == cnt);

    return 0;
}

static int test_app_metadata() {
    const uint8_t reader_guid_prefix[12]
        = {48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};
    const uint8_t reader_entity_id[4] = {60, 61, 62, 63};
    const uint8_t writer_entity_id[4] = {255, 254, 253, 252};

    message_metadata_t msg_metadata;
    uint8_t            reader_guid_prefix_out[12];
    uint8_t            reader_entity_id_out[4];
    uint8_t            writer_entity_id_out[4];

    serialize_app_metadata(reader_guid_prefix, reader_entity_id,
                           writer_entity_id, &msg_metadata);
    deserialize_app_metadata(reader_guid_prefix_out, reader_entity_id_out,
                             writer_entity_id_out, &msg_metadata);

    for (auto i = 0; i < 12; i++) {
        assert(reader_guid_prefix_out[i] == reader_guid_prefix[i]);
    }
    for (auto i = 0; i < 4; i++) {
        assert(reader_entity_id_out[i] == reader_entity_id[i]);
        assert(writer_entity_id_out[i] == writer_entity_id[i]);
    }

    return 0;
}

int test_message_metadata() {
    assert(test_spdp_metadata() == 0);
    assert(test_sedp_metadata() == 0);
    assert(test_sedp_heartbeat_metadata() == 0);
    assert(test_sedp_acknack_metadata(true) == 0);
    assert(test_sedp_acknack_metadata(false) == 0);
    assert(test_app_metadata() == 0);
    return 0;
}
