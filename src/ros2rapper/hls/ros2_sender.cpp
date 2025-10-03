#include "message_metadata.hpp"

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2_sender(
    hls_stream<message_metadata_t> &in /* Cyber port_mode=cw_fifo */
) {
#pragma HLS interface mode = ap_fifo port = in
#pragma HLS interface mode = ap_ctrl_none port = return
    in.read();
}
