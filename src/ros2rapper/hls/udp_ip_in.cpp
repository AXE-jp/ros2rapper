// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "udp_ip_in.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "udp.hpp"
#include <cstdint>

#ifdef __SYNTHESIS__
#define TRACE(...)
#else
#include <cstdio>
#define TRACE(...) printf(__VA_ARGS__)
#endif

typedef uint8_t pending_index_t;
#define INVALID_PENDING_INDEX 0xff

struct pending_info {
    bool     is_used;
    uint16_t id;
    uint8_t  n_arrived;
    uint8_t  n_total;
    uint16_t len;
    uint32_t expiration;
};

#define TOTAL_FRAGMENTS_UNKNOWN 0xff

/* Cyber func=inline */
static void init_pending_info(pending_info *pending, uint16_t id,
                              uint32_t fragment_expiration) {
#pragma HLS inline
    pending->is_used = true;
    pending->id = id;
    pending->n_arrived = 0;
    pending->n_total = TOTAL_FRAGMENTS_UNKNOWN;
    pending->len = 0;
    pending->expiration = fragment_expiration;
    TRACE("%s: pending entry activated (id=%d)\n", __func__, id);
}

static void purge_pending_info(pending_info *pending) {
#pragma HLS inline
    pending->is_used = false;
}

/* Cyber func=inline */
static pending_index_t find_pending_info(pending_info *pendings, uint16_t id,
                                         uint32_t fragment_expiration) {
#pragma HLS inline
    pending_index_t found = INVALID_PENDING_INDEX;
    pending_index_t unused = INVALID_PENDING_INDEX;

#ifndef __SYNTHESIS__
    TRACE("%s: finding id=%d\n", __func__, id);
    TRACE("%s: ===============\n", __func__);
    for (int i = 0; i < MAX_PENDINGS; i++) {
        if (pendings[i].is_used)
            TRACE("%s: [%d] id=%d arrived=%d total=%d len=%d exp=%d\n",
                  __func__, i, pendings[i].id, pendings[i].n_arrived,
                  pendings[i].n_total, pendings[i].len, pendings[i].expiration);
        else
            TRACE("%s: [%d] not used\n", __func__, i);
    }
    TRACE("%s: ===============\n", __func__);
#endif

    /* Cyber unroll_times=all */
    for (int i = 0; i < MAX_PENDINGS; i++) {
#pragma HLS unroll
        if (pendings[i].is_used && pendings[i].id == id) {
            found = i;
            TRACE("%s: pending %d matched\n", __func__, found);
            break;
        }
        if (!pendings[i].is_used) {
            unused = i;
        }
    }
    if (found != INVALID_PENDING_INDEX) {
        return found;
    } else {
        if (unused != INVALID_PENDING_INDEX)
            init_pending_info(&pendings[unused], id, fragment_expiration);
        return unused;
    }
}

/* Cyber func=inline */
static void tick_pendings(pending_info *pendings) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (int i = 0; i < MAX_PENDINGS; i++) {
#pragma HLS unroll
        if (pendings[i].is_used) {
            // do not purge if all fragments were assembled
            if (pendings[i].n_arrived != pendings[i].n_total) {
                if (pendings[i].expiration == 0) {
                    purge_pending_info(&pendings[i]);
                    TRACE("%s: pending %d expired\n", __func__, i);
                } else {
                    pendings[i].expiration -= 1;
                }
            }
        }
    }
}

/* Cyber func=inline */
static int8_t get_fragment_index(uint16_t fragment_offset) {
#pragma HLS inline
#if MAX_IP_FRAGMENTS == 4
    switch (fragment_offset) {
    case (IP_FRAGMEMT_OFFSET_BASE * 0):
        return 0;
    case (IP_FRAGMEMT_OFFSET_BASE * 1):
        return 1;
    case (IP_FRAGMEMT_OFFSET_BASE * 2):
        return 2;
    case (IP_FRAGMEMT_OFFSET_BASE * 3):
        return 3;
    default:
        return -1;
    }
#elif MAX_IP_FRAGMENTS == 2
    switch (fragment_offset) {
    case (IP_FRAGMEMT_OFFSET_BASE * 0):
        return 0;
    case (IP_FRAGMEMT_OFFSET_BASE * 1):
        return 1;
    default:
        return -1;
    }
#elif MAX_IP_FRAGMENTS == 1
    switch (fragment_offset) {
    case (IP_FRAGMEMT_OFFSET_BASE * 0):
        return 0;
    default:
        return -1;
    }
#else
#error "not implemented!"
#endif
}

/* Cyber func=inline */
static int8_t get_payload_offset(pending_index_t pindex) {
#pragma HLS inline
#if MAX_PENDINGS == 4
    switch (pindex) {
    case 0:
        return 0;
    case 1:
        return (IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS);
    case 2:
        return (IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS * 2);
    case 3:
        return (IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS * 3);
    default:
        return 0;
    }
#elif MAX_PENDINGS == 2
    switch (pindex) {
    case 0:
        return 0;
    case 1:
        return (IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS);
    default:
        return 0;
    }
#elif MAX_PENDINGS == 1
    return 0;
#else
#error "not implemented!"
#endif
}

enum udp_ip_in_state_t {
    UDP_IP_IN_STATE_HEADER,
    UDP_IP_IN_STATE_PAYLOAD,
    UDP_IP_IN_STATE_PAYLOAD_TO_MEMORY,
    UDP_IP_IN_STATE_PAYLOAD_FROM_MEMORY,
    UDP_IP_IN_STATE_SKIP
};

/* Cyber func=process, bdltran_option=-s, process_valid=NO,
   async_reset_port=rst_n- */
void udp_ip_in(
    hls_stream<hls_uint<9>> &in /* Cyber port_mode=axi_stream:reg_both */,
    hls_stream<hls_uint<9>> &out /* Cyber port_mode=axi_stream:reg_both */,
    uint8_t  ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    uint32_t fragment_expiration /* Cyber port_mode=in */,
    uint8_t *error /* Cyber port_mode=shared */) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = ap_memory port = ip_payloads storage_type = ram_1p
#pragma HLS interface mode = ap_none port = fragment_expiration
#pragma HLS interface mode = ap_vld port = error

    static pending_info pendings[MAX_PENDINGS] /* Cyber array=REG */;
#pragma HLS array_partition variable = pendings complete dim = 0
    static pending_index_t pending_index = INVALID_PENDING_INDEX;

    /* state / usage of variables...
     * HEADER / len: payload length, offset: from the beginning of header
     * PAYLOAD / len: payload length, offset: from the beginning of header
     * PAYLOAD_TO_MEMORY / len: payload length, offset: from a fragment offset
     * PAYLOAD_FROM_MEMORY / len: not used, offset: from the beginning of a
     * payload buffer
     */
    static udp_ip_in_state_t state = UDP_IP_IN_STATE_HEADER;
    static uint16_t          offset = 0;
    static uint16_t          len = 0;
    static uint16_t          id = 0;
    static uint16_t          flags_and_offset = 0;
    static uint8_t           protocol = 0;

#define reset_state()                                                          \
    do {                                                                       \
        pending_index = INVALID_PENDING_INDEX;                                 \
        state = UDP_IP_IN_STATE_HEADER;                                        \
        offset = 0;                                                            \
        len = 0;                                                               \
        id = 0;                                                                \
        flags_and_offset = 0;                                                  \
        protocol = 0;                                                          \
    } while (0)

    hls_uint<9> x;
    uint8_t     data;
    bool        end;

    tick_pendings(pendings);

    switch (state) {
    case UDP_IP_IN_STATE_HEADER:
        if (in.empty()) {
            return;
        }
        in.read_nb(x);
        data = x & 0xff;
        end = x & 0x100;

        switch (offset) {
        case IP_HDR_OFFSET_TOT_LEN:
            len = data << 8;
            break;
        case (IP_HDR_OFFSET_TOT_LEN + 1):
            len |= data;
            len -= IP_HDR_SIZE;
            break;
        case IP_HDR_OFFSET_ID:
            id = data << 8;
            break;
        case (IP_HDR_OFFSET_ID + 1):
            id |= data;
            break;
        case IP_HDR_OFFSET_FLAG_OFF:
            flags_and_offset = data << 8;
            break;
        case (IP_HDR_OFFSET_FLAG_OFF + 1):
            flags_and_offset |= data;
            break;
        case IP_HDR_OFFSET_PROTOCOL:
            protocol = data;
            break;
        }

        offset++;
        if (end) {
            reset_state();
            TRACE("%s: state changed to HEADER.\n", __func__);
        } else if ((offset == IP_HDR_SIZE)
                   && (protocol != PSEUDO_HDR_PROTOCOL)) {
            // Ignore the received packet if it is not a UDP packet.
            state = UDP_IP_IN_STATE_SKIP;
            TRACE("%s: The received packet is not UDP.\n", __func__);
            TRACE("%s: state changed to SKIP.\n", __func__);
        } else if (offset == IP_HDR_SIZE) {
            bool     has_more_fragments = HAS_MORE_FRAGMENTS(flags_and_offset);
            uint16_t fragment_offset = GET_FRAGMENT_OFFSET(flags_and_offset);
            int8_t   findex = get_fragment_index(fragment_offset);
            if (findex == -1) {
                // can't process
                *error = ERR_UDP_IP_IN_CANNOT_PROCESS;
                state = UDP_IP_IN_STATE_SKIP;
                TRACE("%s: Can't process this fragment!\n", __func__);
                TRACE("%s: state changed to SKIP.\n", __func__);
            } else if ((findex == 0) && !has_more_fragments) {
                // not fragmented
                state = UDP_IP_IN_STATE_PAYLOAD;
                TRACE(
                    "%s: The received packet is a not-fragmented UDP packet.\n",
                    __func__);
                TRACE("%s: state changed to PAYLOAD.\n", __func__);
            } else {
                // fragmented
                pending_index
                    = find_pending_info(pendings, id, fragment_expiration);
                if (pending_index == INVALID_PENDING_INDEX) {
                    // no room for new datagram
                    *error = ERR_UDP_IP_IN_NO_ROOM;
                    state = UDP_IP_IN_STATE_SKIP;
                    TRACE("%s: No room for new datagram!\n", __func__);
                    TRACE("%s: state changed to SKIP.\n", __func__);
                } else {
                    pendings[pending_index].n_arrived += 1;
                    if (!has_more_fragments) {
                        // last fragment
                        pendings[pending_index].n_total = findex + 1;
                    }
                    state = UDP_IP_IN_STATE_PAYLOAD_TO_MEMORY;
                    TRACE("%s: state changed to PAYLOAD_TO_MEMORY\n", __func__);
                    offset = fragment_offset << 3;
                    pendings[pending_index].len += len;
                    TRACE("%s: memory write begin buf#%d offset=%d\n", __func__,
                          pending_index, offset);
                }
            }
        }
        break;
    case UDP_IP_IN_STATE_PAYLOAD:
        if (!out.full() && !in.empty()) {
            in.read_nb(x);
            end = x & 0x100;
            if (offset >= IP_HDR_SIZE + UDP_HDR_SIZE) {
                out.write(x);
            }
            offset++;
            if (end) {
                uint16_t expected = IP_HDR_SIZE + len;
                if (offset != expected) {
                    // length check
                    *error = ERR_UDP_IP_IN_INVALID_LENGTH;
                    TRACE("%s: length error %d != %d\n", __func__, offset,
                          expected);
                }
                reset_state();
                TRACE("%s: state changed to HEADER.\n", __func__);
            }
        }
        break;
    case UDP_IP_IN_STATE_PAYLOAD_TO_MEMORY:
        if (in.empty()) {
            return;
        }
        in.read_nb(x);
        data = x & 0xff;
        end = x & 0x100;
        ip_payloads[get_payload_offset(pending_index) + offset] = data;
        offset++;
        if (end) {
            TRACE("%s: n_arrived=%d, n_total=%d\n", __func__,
                  pendings[pending_index].n_arrived,
                  pendings[pending_index].n_total);
            uint16_t expected = pendings[pending_index].len;
            if (offset != expected) {
                // length check
                *error = ERR_UDP_IP_IN_INVALID_LENGTH;
                TRACE("%s: length error %d != %d\n", __func__, offset,
                      expected);
                purge_pending_info(&pendings[pending_index]);
                reset_state();
                TRACE("%s: state changed to HEADER.\n", __func__);
            } else if (pendings[pending_index].n_arrived
                       == pendings[pending_index].n_total) {
                // all fragments arrived
                state = UDP_IP_IN_STATE_PAYLOAD_FROM_MEMORY;
                TRACE("%s: state changed to PAYLOAD_FROM_MEMORY\n", __func__);
                offset = UDP_HDR_SIZE;
            } else {
                reset_state();
                TRACE("%s: state changed to HEADER.\n", __func__);
            }
        }
        break;
    case UDP_IP_IN_STATE_PAYLOAD_FROM_MEMORY:
        if (!out.full()) {
            data = ip_payloads[get_payload_offset(pending_index) + offset];
            offset++;
            if (offset == pendings[pending_index].len) {
                out.write(0x100 | data);
                purge_pending_info(&pendings[pending_index]);
                reset_state();
                TRACE("%s: state changed to HEADER.\n", __func__);
            } else {
                out.write(data);
            }
        }
        break;
    case UDP_IP_IN_STATE_SKIP:
        if (!in.empty()) {
            in.read_nb(x);
            end = x & 0x100;
            if (end) {
                reset_state();
                TRACE("%s: state changed to HEADER.\n", __func__);
            }
        }
        break;
    }
}
