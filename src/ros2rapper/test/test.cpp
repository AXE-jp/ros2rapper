// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cassert>

int test_udp();
int test_sedp_reader_heartbeat();
int test_spdp_reader();
int test_sedp_reader_2();
int test_remove_endpoints();

int main() {
    assert(test_udp() == 0);
    assert(test_sedp_reader_heartbeat() == 0);
    assert(test_spdp_reader() == 0);
    assert(test_sedp_reader_2() == 0);
    assert(test_remove_endpoints() == 0);
    return 0;
}
