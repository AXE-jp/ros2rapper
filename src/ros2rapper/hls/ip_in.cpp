#include "ip_in.hpp"
#include "checksum.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include <cstdint>

#ifdef __SYNTHESIS__
#define TRACE(...)
#else
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
void init_pending_info(pending_info *pending, uint16_t id,
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

#define purge_pending_info(p) ((p)->is_used = false)

/* Cyber func=inline */
pending_index_t find_pending_info(pending_info *pendings, uint16_t id,
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
void tick_pendings(pending_info *pendings) {
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
int8_t get_fragment_index(uint16_t fragment_offset) {
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
int8_t get_payload_offset(pending_index_t pindex) {
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

void ip_in(
    hls_stream<hls_uint<9>> &in, hls_stream<hls_uint<9>> &out,
    uint8_t  ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    uint32_t fragment_expiration, bool ignore_checksum, bool *parity_error) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = ap_memory port = ip_payloads storage_type = ram_1p
#pragma HLS interface mode = ap_none port = fragment_expiration
#pragma HLS interface mode = ap_none port = ignore_checksum
#pragma HLS interface mode = ap_none port = parity_error
    static bool        is_parity_error = false;
    static hls_uint<5> state;
#define IPIN_STATE_HEADER                    0
#define IPIN_STATE_PAYLOAD_PRE_0             1
#define IPIN_STATE_PAYLOAD_PRE_1             2
#define IPIN_STATE_PAYLOAD_PRE_2             3
#define IPIN_STATE_PAYLOAD_PRE_3             4
#define IPIN_STATE_PAYLOAD_PRE_4             5
#define IPIN_STATE_PAYLOAD_PRE_5             6
#define IPIN_STATE_PAYLOAD                   7
#define IPIN_STATE_PAYLOAD_TO_MEMORY         8
#define IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_0 9
#define IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_1 10
#define IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_2 11
#define IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_3 12
#define IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_4 13
#define IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_5 14
#define IPIN_STATE_PAYLOAD_FROM_MEMORY       15
#define IPIN_STATE_PARITY_ERROR_0            16
#define IPIN_STATE_PARITY_ERROR_1            17
#define IPIN_STATE_FRAGMENT_ERROR_0          18
#define IPIN_STATE_FRAGMENT_ERROR_1          19
#define IPIN_STATE_WAIT_DATAGRAM_END         20

    static uint16_t len;
    static uint16_t offset;
    /* state / usage of variables...
     * HEADER / len: payload length, offset: from the beginning of header
     * PAYLOAD / len: payload length, offset: from the beginning of header
     * PAYLOAD_TO_MEMORY / len: payload length, offset: from a fragment offset
     * PAYLOAD_FROM_MEMORY / len: not used, offset: from the beginning of a
     * payload buffer
     */
    static uint16_t sum;
    static uint16_t id;
    static uint16_t flags_and_offset;
    static bool     has_more_fragments;
    static uint16_t fragment_offset;
    static uint8_t  src_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = src_addr complete dim = 0

#define reset_state()                                                          \
    do {                                                                       \
        state = IPIN_STATE_HEADER;                                             \
        len = 0;                                                               \
        offset = 0;                                                            \
        sum = 0;                                                               \
        id = 0;                                                                \
        flags_and_offset = 0;                                                  \
        pending_index = INVALID_PENDING_INDEX;                                 \
    } while (0)

    static pending_info pendings[MAX_PENDINGS] /* Cyber array=REG */;
#pragma HLS array_partition variable = pendings complete dim = 0
    static pending_index_t pending_index = INVALID_PENDING_INDEX;

    hls_uint<9> x;
    uint8_t     data;
    bool        end;

    tick_pendings(pendings);

    switch (state) {
    case IPIN_STATE_HEADER: {
        x = in.read();
        data = x & 0xff;
        end = x & 0x100;

        checksum(sum, offset, data);

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
            has_more_fragments = HAS_MORE_FRAGMENTS(flags_and_offset);
            fragment_offset = GET_FRAGMENT_OFFSET(flags_and_offset);
            break;
        case (IP_HDR_OFFSET_SADDR + 0):
            src_addr[0] = data;
            break;
        case (IP_HDR_OFFSET_SADDR + 1):
            src_addr[1] = data;
            break;
        case (IP_HDR_OFFSET_SADDR + 2):
            src_addr[2] = data;
            break;
        case (IP_HDR_OFFSET_SADDR + 3):
            src_addr[3] = data;
            break;
        }

        offset++;

        if (offset == IP_HDR_SIZE) {
            if (!ignore_checksum && sum != 0xffff) {
                state = IPIN_STATE_PARITY_ERROR_0;
                TRACE("%s: state changed to PARITY_ERROR\n", __func__);
            } else {
                int8_t findex = get_fragment_index(fragment_offset);

                if (findex == -1) {
                    // can't process
                    state = IPIN_STATE_FRAGMENT_ERROR_0;
                    TRACE("%s: Can't process this fragment!\n", __func__);
                    TRACE("%s: state changed to FRAGMENT_ERROR\n", __func__);
                } else {
                    if (has_more_fragments || findex != 0) {
                        // fragmented
                        pending_index = find_pending_info(pendings, id,
                                                          fragment_expiration);
                        if (pending_index == INVALID_PENDING_INDEX) {
                            // no room for new datagram
                            state = IPIN_STATE_FRAGMENT_ERROR_0;
                            TRACE("%s: No room for new datagram!\n", __func__);
                            TRACE("%s: state changed to FRAGMENT_ERROR\n",
                                  __func__);
                        } else {
                            pendings[pending_index].n_arrived += 1;
                            if (!has_more_fragments) {
                                // last fragment
                                pendings[pending_index].n_total = findex + 1;
                            }
                            state = IPIN_STATE_PAYLOAD_TO_MEMORY;
                            TRACE("%s: state changed to PAYLOAD_TO_MEMORY\n",
                                  __func__);
                        }
                    } else {
                        // not fragmented
                        state = IPIN_STATE_PAYLOAD_PRE_0;
                        TRACE("%s: state changed to PAYLOAD\n", __func__);
                    }

                    switch (state) {
                    case IPIN_STATE_PAYLOAD_TO_MEMORY:
                        offset = fragment_offset << 3;
                        pendings[pending_index].len += len;
                        TRACE("%s: memory write begin buf#%d offset=%d\n",
                              __func__, pending_index, offset);
                    }
                }
            }
        }

        if (end) {
            if (pending_index != INVALID_PENDING_INDEX)
                purge_pending_info(&pendings[pending_index]);
            reset_state();
            TRACE("%s: state changed to HEADER\n", __func__);
        }
    } break;
    case IPIN_STATE_PAYLOAD_PRE_0:
        out.write(src_addr[0]);
        state = IPIN_STATE_PAYLOAD_PRE_1;
        break;
    case IPIN_STATE_PAYLOAD_PRE_1:
        out.write(src_addr[1]);
        state = IPIN_STATE_PAYLOAD_PRE_2;
        break;
    case IPIN_STATE_PAYLOAD_PRE_2:
        out.write(src_addr[2]);
        state = IPIN_STATE_PAYLOAD_PRE_3;
        break;
    case IPIN_STATE_PAYLOAD_PRE_3:
        out.write(src_addr[3]);
        state = IPIN_STATE_PAYLOAD_PRE_4;
        break;
    case IPIN_STATE_PAYLOAD_PRE_4:
        out.write(len >> 8);
        state = IPIN_STATE_PAYLOAD_PRE_5;
        break;
    case IPIN_STATE_PAYLOAD_PRE_5:
        out.write((end ? 0x100 : 0) | (len & 0xff));
        state = IPIN_STATE_PAYLOAD;
        break;
    case IPIN_STATE_PAYLOAD: {
        x = in.read();
        data = x & 0xff;
        end = x & 0x100;

        out.write(x);
        offset++;

        if (end) {
            if (offset - IP_HDR_SIZE != len) {
                // length check
                is_parity_error = true; // TODO: error reporting
                TRACE("%s: length error %d != %d\n", __func__,
                      offset + IP_HDR_SIZE, len);
            }
            reset_state();
            TRACE("%s: state changed to HEADER\n", __func__);
        }
    } break;
    case IPIN_STATE_PAYLOAD_TO_MEMORY: {
        x = in.read();
        data = x & 0xff;
        end = x & 0x100;

        ip_payloads[get_payload_offset(pending_index) + offset] = data;
        offset++;

        if (end) {
            TRACE("%s: n_arrived=%d, n_total=%d\n", __func__,
                  pendings[pending_index].n_arrived,
                  pendings[pending_index].n_total);
            if (offset != (fragment_offset << 3) + len) {
                // length check
                TRACE("%s: length error %d != %d\n", __func__, offset,
                      pendings[pending_index].len);
                is_parity_error = true; // TODO: error reporting
                purge_pending_info(&pendings[pending_index]);
                reset_state();
                TRACE("%s: state changed to HEADER\n", __func__);
            } else if (pendings[pending_index].n_arrived
                       == pendings[pending_index].n_total) {
                // all fragments arrived
                state = IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_0;
                TRACE("%s: state changed to PAYLOAD_FROM_MEMORY\n", __func__);
                offset = 0;
            } else {
                reset_state();
                TRACE("%s: state changed to HEADER\n", __func__);
            }
        }
    } break;
    case IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_0:
        out.write(src_addr[0]);
        state = IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_1;
        break;
    case IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_1:
        out.write(src_addr[1]);
        state = IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_2;
        break;
    case IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_2:
        out.write(src_addr[2]);
        state = IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_3;
        break;
    case IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_3:
        out.write(src_addr[3]);
        state = IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_4;
        break;
    case IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_4:
        out.write(pendings[pending_index].len >> 8);
        state = IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_5;
        break;
    case IPIN_STATE_PAYLOAD_FROM_MEMORY_PRE_5:
        out.write(pendings[pending_index].len & 0xff);
        state = IPIN_STATE_PAYLOAD_FROM_MEMORY;
        break;

    case IPIN_STATE_PAYLOAD_FROM_MEMORY:
        if (offset == pendings[pending_index].len - 1) {
            out.write(
                0x100
                | ip_payloads[get_payload_offset(pending_index) + offset]);
            purge_pending_info(&pendings[pending_index]);
            reset_state();
            TRACE("%s: state changed to HEADER\n", __func__);
        } else {
            out.write(ip_payloads[get_payload_offset(pending_index) + offset]);
            offset++;
        }
        break;
    case IPIN_STATE_PARITY_ERROR_0:
        out.write(len >> 8);
        state = IPIN_STATE_PARITY_ERROR_1;
        break;
    case IPIN_STATE_PARITY_ERROR_1:
        out.write(0x100 | (len & 0xff));
        is_parity_error = true;
        state = IPIN_STATE_WAIT_DATAGRAM_END;
        TRACE("%s: state changed to WAIT_DATAGRAM_END\n", __func__);
        break;
    case IPIN_STATE_FRAGMENT_ERROR_0:
        out.write(len >> 8);
        state = IPIN_STATE_FRAGMENT_ERROR_1;
        break;
    case IPIN_STATE_FRAGMENT_ERROR_1:
        out.write(0x100 | (len & 0xff));
        is_parity_error = true; // TODO: error reporting
        state = IPIN_STATE_WAIT_DATAGRAM_END;
        TRACE("%s: state changed to WAIT_DATAGRAM_END\n", __func__);
        break;
    case IPIN_STATE_WAIT_DATAGRAM_END: {
        x = in.read();
        data = x & 0xff;
        end = x & 0x100;

        if (end) {
            reset_state();
            TRACE("%s: state changed to HEADER\n", __func__);
        }
    } break;
    }

    *parity_error = is_parity_error;
}
