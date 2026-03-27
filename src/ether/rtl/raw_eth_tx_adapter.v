// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`include "ros2_ether_config.vh"
`default_nettype none

module raw_eth_tx_adapter (
    input  wire clk,
    input  wire rst_n,
    input  wire enable,

    output wire [$clog2(`ROS2_MAX_RAW_ETH_TX_DATA_LEN)-3:0] tx_raw_eth_data_addr,
    output wire tx_raw_eth_data_ce,
    input  wire [31:0] tx_raw_eth_data_rdata,
    input  wire tx_raw_eth_frame_ready,
    output wire tx_raw_eth_completed,

    output wire [7:0] tx_raw_eth_axis_tdata,
    output wire tx_raw_eth_axis_tvalid,
    input  wire tx_raw_eth_axis_tready,
    output wire tx_raw_eth_axis_tlast,

    output wire tx_raw_eth_ip_hdr_valid,
    input  wire tx_raw_eth_ip_hdr_ready,
    output wire [5:0]  tx_raw_eth_ip_dscp,
    output wire [1:0]  tx_raw_eth_ip_ecn,
    output wire [15:0] tx_raw_eth_ip_length,
    output wire [7:0]  tx_raw_eth_ip_ttl,
    output wire [7:0]  tx_raw_eth_ip_protocol,
    output wire [31:0] tx_raw_eth_ip_source_ip,
    output wire [31:0] tx_raw_eth_ip_dest_ip,
    output wire [7:0]  tx_raw_eth_ip_payload_axis_tdata,
    output wire tx_raw_eth_ip_payload_axis_tvalid,
    input  wire tx_raw_eth_ip_payload_axis_tready,
    output wire tx_raw_eth_ip_payload_axis_tlast,

    output wire select_tx_raw_eth_axis,
    output wire select_tx_raw_eth_ip
);
    localparam [5:0] STATE_IDLE               = 6'd0;
    localparam [5:0] STATE_READ_ETH_HDR_0     = 6'd1;
    localparam [5:0] STATE_READ_ETH_HDR_1     = 6'd2;
    localparam [5:0] STATE_READ_ETH_HDR_2     = 6'd3;
    localparam [5:0] STATE_READ_ETH_HDR_3     = 6'd4; // Identify the packet type here
    localparam [5:0] STATE_SEND_ETH_HDR_0     = 6'd5; // If the frame is not IPv4
    /* verilator lint_off UNUSEDPARAM */
    localparam [5:0] STATE_SEND_ETH_HDR_1     = 6'd6;
    localparam [5:0] STATE_SEND_ETH_HDR_2     = 6'd7;
    localparam [5:0] STATE_SEND_ETH_HDR_3     = 6'd8;
    localparam [5:0] STATE_SEND_ETH_HDR_4     = 6'd9;
    localparam [5:0] STATE_SEND_ETH_HDR_5     = 6'd10;
    localparam [5:0] STATE_SEND_ETH_HDR_6     = 6'd11;
    localparam [5:0] STATE_SEND_ETH_HDR_7     = 6'd12;
    localparam [5:0] STATE_SEND_ETH_HDR_8     = 6'd13;
    localparam [5:0] STATE_SEND_ETH_HDR_9     = 6'd14;
    localparam [5:0] STATE_SEND_ETH_HDR_10    = 6'd15;
    localparam [5:0] STATE_SEND_ETH_HDR_11    = 6'd16;
    localparam [5:0] STATE_SEND_ETH_HDR_12    = 6'd17;
    /* verilator lint_on UNUSEDPARAM */
    localparam [5:0] STATE_SEND_ETH_HDR_13    = 6'd18;
    localparam [5:0] STATE_SEND_ETH_PAYLOAD_0 = 6'd19;
    localparam [5:0] STATE_SEND_ETH_PAYLOAD_1 = 6'd20;
    localparam [5:0] STATE_SEND_ETH_PAYLOAD_2 = 6'd21;
    localparam [5:0] STATE_SEND_ETH_PAYLOAD_3 = 6'd22;
    localparam [5:0] STATE_READ_IP_HDR_0      = 6'd23; // If the frame is IPv4
    localparam [5:0] STATE_READ_IP_HDR_1      = 6'd24;
    localparam [5:0] STATE_READ_IP_HDR_2      = 6'd25;
    localparam [5:0] STATE_READ_IP_HDR_3      = 6'd26;
    localparam [5:0] STATE_READ_IP_HDR_4      = 6'd27;
    localparam [5:0] STATE_WAIT_IP_HDR_READY  = 6'd28;
    localparam [5:0] STATE_SEND_IP_PAYLOAD_0  = 6'd29;
    localparam [5:0] STATE_SEND_IP_PAYLOAD_1  = 6'd30;
    localparam [5:0] STATE_SEND_IP_PAYLOAD_2  = 6'd31;
    localparam [5:0] STATE_SEND_IP_PAYLOAD_3  = 6'd32;
    localparam [5:0] STATE_COMPLETE           = 6'd33;
    reg [5:0] state_reg, state_next;

    reg tx_raw_eth_frame_ready_before;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_raw_eth_frame_ready_before <= 1'b0;
        end else begin
            tx_raw_eth_frame_ready_before <= tx_raw_eth_frame_ready;
        end
    end
    // The rising edge of tx_raw_eth_frame_ready
    wire tx_raw_eth_kick = ~tx_raw_eth_frame_ready_before & tx_raw_eth_frame_ready;

    reg    tx_raw_eth_completed_reg;
    wire   tx_raw_eth_completed_next = (state_next == STATE_COMPLETE);
    assign tx_raw_eth_completed = tx_raw_eth_completed_reg;

    wire [31:0] ram_fifo_data;
    wire ram_fifo_valid;
    reg  ram_fifo_ready;
    wire ram_fifo_start = ((state_reg != STATE_IDLE) && (state_reg != STATE_COMPLETE));
    raw_eth_tx_adapter_fifo #(
        .MAX_DATA_LEN(`ROS2_MAX_RAW_ETH_TX_DATA_LEN/4),
        .DATA_WIDTH(32)
    )
    raw_eth_tx_adapter_fifo_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(ram_fifo_start),
        .ram_addr(tx_raw_eth_data_addr),
        .ram_ce(tx_raw_eth_data_ce),
        .ram_rdata(tx_raw_eth_data_rdata),
        .out_tdata(ram_fifo_data),
        .out_tvalid(ram_fifo_valid),
        .out_tready(ram_fifo_ready)
    );

    // Store the ether header data.
    reg [111:0] hdr_mem_reg, hdr_mem_next;

    localparam COUNT_WIDTH = $clog2(`ROS2_MAX_RAW_ETH_TX_DATA_LEN);
    // Indicate how much data is left in bytes.
    // If the current data is the last, count_reg becomes 0.
    reg [COUNT_WIDTH-1:0] count_reg, count_next;

    reg [23:0] payload_reg, payload_next;

    reg  [7:0] eth_axis_tdata;
    reg  eth_axis_tvalid;
    wire eth_axis_tlast  = (count_reg == 0);
    wire eth_axis_tready = tx_raw_eth_axis_tready;
    assign tx_raw_eth_axis_tdata  = eth_axis_tdata;
    assign tx_raw_eth_axis_tvalid = eth_axis_tvalid;
    assign tx_raw_eth_axis_tlast  = eth_axis_tlast;

    reg  ip_hdr_valid_reg;
    wire ip_hdr_valid_next = (state_next == STATE_WAIT_IP_HDR_READY);
    assign tx_raw_eth_ip_hdr_valid = ip_hdr_valid_reg;

    //reg [3:0]  ip_version_reg, ip_version_next;
    //reg [3:0]  ip_ihl_reg, ip_ihl_next;
    reg [5:0]  ip_dscp_reg, ip_dscp_next;
    reg [1:0]  ip_ecn_reg, ip_ecn_next;
    reg [15:0] ip_length_reg, ip_length_next;
    //reg [15:0] ip_identification_reg, ip_identification_next;
    //reg [2:0]  ip_flags_reg, ip_flags_next;
    //reg [12:0] ip_fragmentation_offset_reg, ip_fragmentation_offset_next;
    reg [7:0]  ip_ttl_reg, ip_ttl_next;
    reg [7:0]  ip_protocol_reg, ip_protocol_next;
    //reg [15:0] ip_header_checksum_reg, ip_header_checksum_next;
    reg [31:0] ip_source_ip_reg, ip_source_ip_next;
    reg [31:0] ip_dest_ip_reg, ip_dest_ip_next;
    assign tx_raw_eth_ip_dscp = ip_dscp_reg;
    assign tx_raw_eth_ip_ecn = ip_ecn_reg;
    assign tx_raw_eth_ip_length  = ip_length_reg;
    assign tx_raw_eth_ip_ttl = ip_ttl_reg;
    assign tx_raw_eth_ip_protocol = ip_protocol_reg;
    assign tx_raw_eth_ip_source_ip = ip_source_ip_reg;
    assign tx_raw_eth_ip_dest_ip = ip_dest_ip_reg;

    reg  [7:0] ip_axis_tdata;
    reg  ip_axis_tvalid;
    wire ip_axis_tlast  = (count_reg == 0);
    wire ip_axis_tready = tx_raw_eth_ip_payload_axis_tready;
    assign tx_raw_eth_ip_payload_axis_tdata  = ip_axis_tdata;
    assign tx_raw_eth_ip_payload_axis_tvalid = ip_axis_tvalid;
    assign tx_raw_eth_ip_payload_axis_tlast  = ip_axis_tlast;

    reg  select_tx_raw_eth_axis_reg;
    wire select_tx_raw_eth_axis_next = ((state_next >= STATE_SEND_ETH_HDR_0) && (state_next <= STATE_SEND_ETH_PAYLOAD_3));
    assign select_tx_raw_eth_axis = select_tx_raw_eth_axis_reg;

    reg  select_tx_raw_eth_ip_reg;
    wire select_tx_raw_eth_ip_next = ((state_next >= STATE_READ_IP_HDR_0) && (state_next <= STATE_SEND_IP_PAYLOAD_3));
    assign select_tx_raw_eth_ip = select_tx_raw_eth_ip_reg;

    always @* begin
        state_next = state_reg;
        ram_fifo_ready  = 1'b0;
        hdr_mem_next = hdr_mem_reg;
        count_next = count_reg;
        payload_next = payload_reg;
        eth_axis_tdata  = 8'd0;
        eth_axis_tvalid = 1'b0;
        ip_dscp_next = ip_dscp_reg;
        ip_ecn_next = ip_ecn_reg;
        ip_length_next = ip_length_reg;
        ip_ttl_next = ip_ttl_reg;
        ip_protocol_next = ip_protocol_reg;
        ip_source_ip_next = ip_source_ip_reg;
        ip_dest_ip_next = ip_dest_ip_reg;
        ip_axis_tdata  = 8'd0;
        ip_axis_tvalid = 1'b0;

        if (state_reg == STATE_IDLE) begin
            if (tx_raw_eth_kick) begin
                state_next = STATE_READ_ETH_HDR_0;
            end
        end else if (state_reg == STATE_READ_ETH_HDR_0) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                count_next = ram_fifo_data[COUNT_WIDTH-1:0]; // the frame length
                hdr_mem_next[15:0] = ram_fifo_data[31:16];
                // ram_fifo_data[15:0] is the frame length.
                if ((ram_fifo_data[15:0] == 0) || (ram_fifo_data[15:0] > (`ROS2_MAX_RAW_ETH_TX_DATA_LEN - 2))) begin
                    // The frame has no data, or the designated length is larger than memory size.
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_READ_ETH_HDR_1;
                end
            end
        end else if (state_reg == STATE_READ_ETH_HDR_1) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                hdr_mem_next[47:16] = ram_fifo_data;
                state_next = STATE_READ_ETH_HDR_2;
            end
        end else if (state_reg == STATE_READ_ETH_HDR_2) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                hdr_mem_next[79:48] = ram_fifo_data;
                state_next = STATE_READ_ETH_HDR_3;
            end
        end else if (state_reg == STATE_READ_ETH_HDR_3) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                hdr_mem_next[111:80] = ram_fifo_data;
                // count_reg - the frame length
                // ram_fifo_data[31:16] - ether frame type
                if ((count_reg >= 14) && (ram_fifo_data[31:16] == 16'h00_08)) begin
                    // The ether header is valid and the frame type is IPv4
                    if (count_reg > 34) begin
                        // The IP header is valid and the IP packet has non-empty payload.
                        state_next = STATE_READ_IP_HDR_0;
                        count_next = count_reg - 35;
                    end else begin
                        state_next = STATE_COMPLETE;
                    end
                end else begin
                    state_next = STATE_SEND_ETH_HDR_0;
                    count_next = count_reg - 1;
                end
            end
        end else if ((state_reg >= STATE_SEND_ETH_HDR_0) && (state_reg <= STATE_SEND_ETH_HDR_13)) begin
            eth_axis_tdata  = hdr_mem_reg[8*(state_reg - STATE_SEND_ETH_HDR_0) +: 8];
            eth_axis_tvalid = 1'b1;
            if (eth_axis_tready) begin
                if (eth_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else if (state_reg == STATE_SEND_ETH_HDR_13) begin
                    state_next = STATE_SEND_ETH_PAYLOAD_0;
                    count_next = count_reg - 1;
                end else begin
                    state_next = state_reg + 1;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_ETH_PAYLOAD_0) begin
            // Read RAM data from the FIFO
            ram_fifo_ready = eth_axis_tready;
            payload_next = ram_fifo_data[31:8];
            eth_axis_tdata  = ram_fifo_data[7:0];
            eth_axis_tvalid = ram_fifo_valid;
            if (ram_fifo_valid && eth_axis_tready) begin
                if (eth_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_ETH_PAYLOAD_1;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_ETH_PAYLOAD_1) begin
            eth_axis_tdata  = payload_reg[7:0];
            eth_axis_tvalid = 1'b1;
            if (eth_axis_tready) begin
                if (eth_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_ETH_PAYLOAD_2;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_ETH_PAYLOAD_2) begin
            eth_axis_tdata  = payload_reg[15:8];
            eth_axis_tvalid = 1'b1;
            if (eth_axis_tready) begin
                if (eth_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_ETH_PAYLOAD_3;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_ETH_PAYLOAD_3) begin
            eth_axis_tdata  = payload_reg[23:16];
            eth_axis_tvalid = 1'b1;
            if (eth_axis_tready) begin
                if (eth_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_ETH_PAYLOAD_0;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_READ_IP_HDR_0) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                //ip_version_next = ram_fifo_data[7:4];
                //ip_ihl_next = ram_fifo_data[3:0];
                ip_dscp_next = ram_fifo_data[15:10];
                ip_ecn_next = ram_fifo_data[9:8];
                ip_length_next[15:8] = ram_fifo_data[23:16];
                ip_length_next[7:0]  = ram_fifo_data[31:24];
                state_next = STATE_READ_IP_HDR_1;
            end
        end else if (state_reg == STATE_READ_IP_HDR_1) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                //ip_identification_next[15:8] = ram_fifo_data[7:0];
                //ip_identification_next[7:0]  = ram_fifo_data[15:8];
                //ip_flags_next = ram_fifo_data[23:21];
                //ip_fragmentation_offset_next = {ram_fifo_data[20:16], ram_fifo_data[31:24]};
                state_next = STATE_READ_IP_HDR_2;
            end
        end else if (state_reg == STATE_READ_IP_HDR_2) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                ip_ttl_next = ram_fifo_data[7:0];
                ip_protocol_next = ram_fifo_data[15:8];
                //ip_header_checksum_next[15:8] = ram_fifo_data[23:16];
                //ip_header_checksum_next[7:0]  = ram_fifo_data[31:24];
                state_next = STATE_READ_IP_HDR_3;
            end
        end else if (state_reg == STATE_READ_IP_HDR_3) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                ip_source_ip_next[31:24] = ram_fifo_data[7:0];
                ip_source_ip_next[23:16] = ram_fifo_data[15:8];
                ip_source_ip_next[15:8]  = ram_fifo_data[23:16];
                ip_source_ip_next[7:0]   = ram_fifo_data[31:24];
                state_next = STATE_READ_IP_HDR_4;
            end
        end else if (state_reg == STATE_READ_IP_HDR_4) begin
            ram_fifo_ready = 1'b1;
            if (ram_fifo_valid) begin
                ip_dest_ip_next[31:24] = ram_fifo_data[7:0];
                ip_dest_ip_next[23:16] = ram_fifo_data[15:8];
                ip_dest_ip_next[15:8]  = ram_fifo_data[23:16];
                ip_dest_ip_next[7:0]   = ram_fifo_data[31:24];
                state_next = STATE_WAIT_IP_HDR_READY;
            end
        end else if (state_reg == STATE_WAIT_IP_HDR_READY) begin
            if (tx_raw_eth_ip_hdr_ready) begin
                state_next = STATE_SEND_IP_PAYLOAD_0;
            end
        end else if (state_reg == STATE_SEND_IP_PAYLOAD_0) begin
            // Read RAM data from the FIFO
            ram_fifo_ready = ip_axis_tready;
            payload_next = ram_fifo_data[31:8];
            ip_axis_tdata  = ram_fifo_data[7:0];
            ip_axis_tvalid = ram_fifo_valid;
            if (ram_fifo_valid && ip_axis_tready) begin
                if (ip_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_IP_PAYLOAD_1;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_IP_PAYLOAD_1) begin
            ip_axis_tdata  = payload_reg[7:0];
            ip_axis_tvalid = 1'b1;
            if (ip_axis_tready) begin
                if (ip_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_IP_PAYLOAD_2;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_IP_PAYLOAD_2) begin
            ip_axis_tdata  = payload_reg[15:8];
            ip_axis_tvalid = 1'b1;
            if (ip_axis_tready) begin
                if (ip_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_IP_PAYLOAD_3;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_SEND_IP_PAYLOAD_3) begin
            ip_axis_tdata  = payload_reg[23:16];
            ip_axis_tvalid = 1'b1;
            if (ip_axis_tready) begin
                if (ip_axis_tlast) begin
                    state_next = STATE_COMPLETE;
                end else begin
                    state_next = STATE_SEND_IP_PAYLOAD_0;
                    count_next = count_reg - 1;
                end
            end
        end else if (state_reg == STATE_COMPLETE) begin
            if (!tx_raw_eth_frame_ready) begin
                state_next = STATE_IDLE;
            end
        end else begin
            state_next = STATE_COMPLETE;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= STATE_IDLE;
            tx_raw_eth_completed_reg <= 1'b0;
            hdr_mem_reg <= 112'd0;
            count_reg <= {COUNT_WIDTH{1'b0}};
            payload_reg <= 24'd0;
            ip_hdr_valid_reg <= 1'b0;
            ip_dscp_reg <= 6'd0;
            ip_ecn_reg <= 2'd0;
            ip_length_reg <= 16'd0;
            ip_ttl_reg <= 8'd0;
            ip_protocol_reg <= 8'd0;
            ip_source_ip_reg <= 32'd0;
            ip_dest_ip_reg <= 32'd0;
            select_tx_raw_eth_axis_reg <= 1'b0;
            select_tx_raw_eth_ip_reg <= 1'b0;
        end else begin
            if (!enable) begin
                state_reg <= STATE_IDLE;
                tx_raw_eth_completed_reg <= 1'b0;
                hdr_mem_reg <= 112'd0;
                count_reg <= {COUNT_WIDTH{1'b0}};
                payload_reg <= 24'd0;
                ip_hdr_valid_reg <= 1'b0;
                ip_dscp_reg <= 6'd0;
                ip_ecn_reg <= 2'd0;
                ip_length_reg <= 16'd0;
                ip_ttl_reg <= 8'd0;
                ip_protocol_reg <= 8'd0;
                ip_source_ip_reg <= 32'd0;
                ip_dest_ip_reg <= 32'd0;
                select_tx_raw_eth_axis_reg <= 1'b0;
                select_tx_raw_eth_ip_reg <= 1'b0;
            end else begin
                state_reg <= state_next;
                tx_raw_eth_completed_reg <= tx_raw_eth_completed_next;
                hdr_mem_reg <= hdr_mem_next;
                count_reg <= count_next;
                payload_reg <= payload_next;
                ip_hdr_valid_reg <= ip_hdr_valid_next;
                ip_dscp_reg <= ip_dscp_next;
                ip_ecn_reg <= ip_ecn_next;
                ip_length_reg <= ip_length_next;
                ip_ttl_reg <= ip_ttl_next;
                ip_protocol_reg <= ip_protocol_next;
                ip_source_ip_reg <= ip_source_ip_next;
                ip_dest_ip_reg <= ip_dest_ip_next;
                select_tx_raw_eth_axis_reg <= select_tx_raw_eth_axis_next;
                select_tx_raw_eth_ip_reg <= select_tx_raw_eth_ip_next;
            end
        end
    end
endmodule

// Read data from a RAM
module raw_eth_tx_adapter_read #(
    parameter ADDR_WIDTH = 9,
    parameter DATA_WIDTH = 32
)
(
    input  wire clk,
    input  wire rst_n,

    output wire [ADDR_WIDTH-1:0] ram_addr,
    output wire ram_ce,
    input  wire [DATA_WIDTH-1:0] ram_rdata,

    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire addr_valid,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire rdata_valid
);
    reg [ADDR_WIDTH-1:0] r_ram_addr;
    reg r_ram_ce;
    reg r_rdata_valid;

    assign ram_addr = r_ram_addr;
    assign ram_ce = r_ram_ce;
    assign rdata = ram_rdata;
    assign rdata_valid = r_rdata_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_ram_addr <= {ADDR_WIDTH{1'b0}};
            r_ram_ce <= 1'b0;
            r_rdata_valid <= 1'b0;
        end else begin
            r_ram_addr <= addr;
            r_ram_ce <= addr_valid;
            r_rdata_valid <= r_ram_ce;
        end
    end
endmodule

// Read data from a RAM, and output it.
module raw_eth_tx_adapter_fifo #(
    parameter MAX_DATA_LEN=400,
    parameter DATA_WIDTH=32
)
(
    input  wire clk,
    input  wire rst_n,
    input  wire start,

    // RAM ports
    output wire [$clog2(MAX_DATA_LEN)-1:0] ram_addr,
    output wire ram_ce,
    input  wire [DATA_WIDTH-1:0] ram_rdata,

    // AXIS ports
    output wire [DATA_WIDTH-1:0] out_tdata,
    output wire out_tvalid,
    input  wire out_tready
);
    localparam ADDR_WIDTH = $clog2(MAX_DATA_LEN);

    reg  [ADDR_WIDTH-1:0] r_addr;
    wire w_addr_valid;
    wire [DATA_WIDTH-1:0] w_rdata_new;
    wire w_rdata_new_valid;

    raw_eth_tx_adapter_read #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    )
    raw_eth_tx_adapter_read_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ram_addr(ram_addr),
        .ram_ce(ram_ce),
        .ram_rdata(ram_rdata),
        .addr(r_addr),
        .addr_valid(w_addr_valid),
        .rdata(w_rdata_new),
        .rdata_valid(w_rdata_new_valid)
    );

    // FIFO (w_rdata_new -> rdata_reg[0] -> rdata_reg[1] -> out)
    reg [DATA_WIDTH-1:0] rdata_reg [0:1];
    reg [DATA_WIDTH-1:0] rdata_next [0:1];
    reg [1:0] rdata_valid_reg;
    reg [1:0] rdata_valid_next;

    assign out_tdata  = rdata_valid_reg[1] ? rdata_reg[1] : w_rdata_new;
    assign out_tvalid = rdata_valid_reg[1] ? 1'b1         : w_rdata_new_valid;
    always @* begin
        rdata_next[1]       = rdata_reg[1];
        rdata_valid_next[1] = rdata_valid_reg[1];
        rdata_next[0]       = rdata_reg[0];
        rdata_valid_next[0] = rdata_valid_reg[0];
        if (!out_tready) begin
            case (rdata_valid_reg)
                2'b00: begin
                    rdata_next[1]       = w_rdata_new;
                    rdata_valid_next[1] = w_rdata_new_valid;
                end
                2'b10: begin
                    rdata_next[0]       = w_rdata_new;
                    rdata_valid_next[0] = w_rdata_new_valid;
                end
            endcase
        end else begin
            case (rdata_valid_reg)
                2'b10: begin
                    rdata_next[1]       = w_rdata_new;
                    rdata_valid_next[1] = w_rdata_new_valid;
                end
                2'b11: begin
                    rdata_next[1]       = rdata_reg[0];
                    rdata_valid_next[1] = 1'b1;
                    rdata_next[0]       = w_rdata_new;
                    rdata_valid_next[0] = w_rdata_new_valid;
                end
            endcase
        end
    end
    // w_addr_valid is asserted if and only if
    // rdata_valid_reg[0] is guaranteed to be 1'b0 after two clock cycles.
    assign w_addr_valid = start & ~rdata_valid_next[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_addr <= {ADDR_WIDTH{1'b0}};
            rdata_valid_reg <= 2'd0;
            rdata_reg[0] <= {DATA_WIDTH{1'b0}};
            rdata_reg[1] <= {DATA_WIDTH{1'b0}};
        end else begin
            if (!start) begin
                r_addr <= {ADDR_WIDTH{1'b0}};
                rdata_valid_reg <= 2'd0;
                rdata_reg[0] <= {DATA_WIDTH{1'b0}};
                rdata_reg[1] <= {DATA_WIDTH{1'b0}};
            end else begin
                if (w_addr_valid) begin
                    if (r_addr + 1'b1 == MAX_DATA_LEN) begin
                        r_addr <= {ADDR_WIDTH{1'b0}};
                    end else begin
                        r_addr <= r_addr + 1'b1;
                    end
                end
                rdata_valid_reg <= rdata_valid_next;
                rdata_reg[0] <= rdata_next[0];
                rdata_reg[1] <= rdata_next[1];
            end
        end
    end
endmodule

`default_nettype wire
