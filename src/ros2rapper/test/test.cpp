// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cassert>

int test_udp_ip_in();
int test_sedp_reader_heartbeat();
int test_spdp_reader();
int test_sedp_reader_2();
int test_remove_endpoints();
int test_multi_topic_subscription();
int test_message_metadata();

int main() {
    assert(test_udp_ip_in() == 0);
    assert(test_sedp_reader_heartbeat() == 0);
    assert(test_spdp_reader() == 0);
    assert(test_sedp_reader_2() == 0);
    assert(test_remove_endpoints() == 0);
    assert(test_multi_topic_subscription() == 0);
    assert(test_message_metadata() == 0);
    return 0;
}
