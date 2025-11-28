// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`include "ros2_ether_config.vh"
`default_nettype none

module raw_eth_rx_adapter (
    input  wire clk,
    input  wire rst_n,
    input  wire enable,

    output wire [7:0] rx_raw_eth_axis_tdata,
    output wire rx_raw_eth_axis_tvalid,
    input  wire rx_raw_eth_axis_tready,
    output wire rx_raw_eth_axis_tlast,

    output wire [$clog2(`ROS2_MAX_RAW_ETH_RX_DATA_LEN)-3:0] rx_raw_eth_data_addr,
    output wire rx_raw_eth_data_ce,
    output wire [3:0] rx_raw_eth_data_we,
    output wire [31:0] rx_raw_eth_data_wdata,
    output wire rx_raw_eth_data_frame_ready,
    input  wire rx_raw_eth_data_ack,

    input wire [31:0] ip_addr,
    input wire [31:0] subnet_mask
);
    reg [4:0] state_reg;
    reg [4:0] state_next;
    localparam [4:0] IDLE                    = 5'd0;
    localparam [4:0] READING                 = 5'd1; // Read the first 46 bytes to identify the type of the receiving packet
    localparam [4:0] IDENTIFY_PACKET_TYPE    = 5'd2;
    localparam [4:0] SENDING_STREAM          = 5'd3; // Send the remaining packet data
    localparam [4:0] SENDING_SHORT_PACKET_00 = 5'd4; // Send data_mem data (if the packet length <= 46)
    localparam [4:0] SENDING_SHORT_PACKET_01 = 5'd5;
    localparam [4:0] SENDING_SHORT_PACKET_02 = 5'd6;
    localparam [4:0] SENDING_SHORT_PACKET_03 = 5'd7;
    localparam [4:0] SENDING_SHORT_PACKET_04 = 5'd8;
    localparam [4:0] SENDING_SHORT_PACKET_05 = 5'd9;
    localparam [4:0] SENDING_SHORT_PACKET_06 = 5'd10;
    localparam [4:0] SENDING_SHORT_PACKET_07 = 5'd11;
    localparam [4:0] SENDING_SHORT_PACKET_08 = 5'd12;
    localparam [4:0] SENDING_SHORT_PACKET_09 = 5'd13;
    localparam [4:0] SENDING_SHORT_PACKET_10 = 5'd14;
    localparam [4:0] SENDING_SHORT_PACKET_11 = 5'd15;
    localparam [4:0] COUNT_ONLY              = 5'd16; // When the packet length exceeds `ROS2_MAX_RAW_ETH_MAX_DATA_LEN
    localparam [4:0] FINISH                  = 5'd17; // Set the packet length and assert frame_ready
    localparam [4:0] WAIT_LAST               = 5'd18; // Ignore the receiving packet
    // 19 is not used.
    localparam [4:0] SENDING_MEM_DATA_00     = 5'd20; // Send data_men data (if the packet length > 46)
    localparam [4:0] SENDING_MEM_DATA_01     = 5'd21;
    localparam [4:0] SENDING_MEM_DATA_02     = 5'd22;
    localparam [4:0] SENDING_MEM_DATA_03     = 5'd23;
    localparam [4:0] SENDING_MEM_DATA_04     = 5'd24;
    localparam [4:0] SENDING_MEM_DATA_05     = 5'd25;
    localparam [4:0] SENDING_MEM_DATA_06     = 5'd26;
    localparam [4:0] SENDING_MEM_DATA_07     = 5'd27;
    localparam [4:0] SENDING_MEM_DATA_08     = 5'd28;
    localparam [4:0] SENDING_MEM_DATA_09     = 5'd29;
    localparam [4:0] SENDING_MEM_DATA_10     = 5'd30;
    localparam [4:0] SENDING_MEM_DATA_11     = 5'd31;

    reg  frame_ready_reg;
    wire frame_ready_next = (frame_ready_reg & ~rx_raw_eth_data_ack) | (state_reg == FINISH);
    assign rx_eth_raw_data_frame_ready = frame_ready_reg;

    reg axis_tready_reg;
    wire axis_tready_next = (state_next == IDLE) | (state_next == READING) | (state_next == SENDING_STREAM) | (state_next == COUNT_ONLY) | (state_next == WAIT_LAST);
    assign rx_raw_eth_axis_tready = axis_tready_reg;

    // Packet length
    localparam COUNT_WIDTH = 16;
    reg [COUNT_WIDTH-1:0] count_reg;
    reg [COUNT_WIDTH-1:0] count_next;
    
    integer iter;
    localparam DATA_MEM_SIZE = 46; // 14 + 20 + 8 + 4
    reg [7:0] data_mem [0:DATA_MEM_SIZE*8-1];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (iter = 0; iter < DATA_MEM_SIZE; iter = iter + 1) begin
                data_mem[iter] <= 8'd0;
            end
        end else begin
            if ((state == READING) && (count_reg < DATA_MEM_SIZE)) begin
                data_mem[count_reg] <= rx_raw_eth_axis_tdata;
            end
        end
    end
    // data_mem
    // Ether header
    // 0 ~ 5: dest_mac
    // 6 ~ 11: src_mac
    // 12, 13: type
    // IP header
    // 14: version, header length
    // 15: differentiated service field
    // 16, 17: total length
    // 18, 19: identification
    // 20, 21: flags and fragment offset
    // 22: time to live
    // 23: protocol
    // 24, 25: header checksum
    // 26 ~ 29: source ip address
    // 30 ~ 33: destination ip address
    // UDP header
    // 34, 35: source port
    // 36, 37: destination port
    // 38, 39: length
    // 40, 41: checksum
    // RTPS
    // 42 ~ 45: Magic
    wire is_arp = (data_mem[12] == 8'h08) & (data_mem[13] == 8'h06);
    wire is_ipv4 = (data_mem[12] == 8'h08) & (data_mem[13] == 8'h00) & (data_mem[14][7:4] == 4'd4);
    wire no_header_option = (data_mem[4][3:0] == 4'd5);

    wire [31:0] unicast_mask;
    assign unicast_mask[7:0]   = data_mem[30] ^ ip_addr[7:0];
    assign unicast_mask[15:8]  = data_mem[31] ^ ip_addr[15:8];
    assign unicast_mask[23:16] = data_mem[32] ^ ip_addr[23:16];
    assign unicast_mask[31:24] = data_mem[33] ^ ip_addr[31:24];
    wire [31:0] broadcast_mask;
    assign broadcast_mask[7:0]   = subnet_mask[7:0]   | ~data_mem[30];
    assign broadcast_mask[15:8]  = subnet_mask[15:8]  | ~data_mem[31];
    assign broadcast_mask[23:16] = subnet_mask[23:16] | ~data_mem[32];
    assign broadcast_mask[31:24] = subnet_mask[31:24] | ~data_mem[33];
    wire is_unicast_to_me = (unicast_mask == 32'd0);
    wire is_multicast = (data_mem[30][7:4] == 4'b1110);
    wire is_broadcast = ((unicast_mask & broadcast_mask) == 32'd0);
    
    wire more_fragment = data_mem[20][5];
    wire fragment_offset = {data_mem[20][4:0], data_mem[21]};
    wire is_fragment = is_ipv4 & (more_fragment | (fragment_offset != 13'd0));
    wire is_udp = (data_mem[23] == 8'd17);
    wire has_rtps_magic = (data_mem[42] == "R") & (data_mem[43] == "T") & (data_mem[44] == "P") & (data_mem[45] == "S");
    wire is_rtps = is_ipv4 & no_header_option & (is_unicast_to_me | is_multicast | is_broadcast) & is_udp & has_rtps_magic;

    localparam ADDR_WIDTH = $clog2(`ROS2_MAX_RAW_ETH_RX_DATA_LEN) - 2;
    reg [ADDR_WIDTH-1:0] ram_addr_reg;
    reg [ADDR_WIDTH-1:0] ram_addr_next;
    reg ram_ce_reg;
    reg ram_ce_next;
    reg [3:0] ram_we_reg;
    reg [3:0] ram_we_next;
    reg [31:0] ram_wdata_reg;
    reg [31:0] ram_wdata_next;
    assign rx_eth_raw_data_addr  = ram_addr_reg;
    assign rx_eth_raw_data_ce    = ram_ce_reg;
    assign rx_eth_raw_data_we    = ram_we_reg;
    assign rx_eth_raw_data_wdata = ram_wdata_reg;
    always @* begin
        ram_addr_next = {ADDR_WIDTH{1'b0}};
        ram_ce_next = 1'b0;
        ram_we_next = 4'd0;
        ram_wdata_next = 32'd0;
        for (iter = 0; iter < 11; iter = iter + 1) begin
            if ((state_reg == (SENDING_SHORT_PACKET_00 + iter))
                    || (state_reg == (SENDING_MEM_DATA_00 + iter))) begin
                ram_addr_next = iter + 1;
                ram_ce_next = 1'b1;
                ram_we_next = 4'b1111;
                ram_wdata_next[7:0]   = data_mem[4*iter];
                ram_wdata_next[15:8]  = data_mem[4*iter+1];
                ram_wdata_next[23:16] = data_mem[4*iter+2];
                ram_wdata_next[31:24] = data_mem[4*iter+3];
            end
        end
        if ((state_reg == SENDING_SHORT_PACKET_11)
                || (state_reg == SENDING_MEM_DATA_11)) begin
            ram_addr_next = 12;
            ram_ce_next = 1'b1;
            ram_we_next = 4'b0011;
            ram_wdata_next[7:0]   = data_mem[44];
            ram_wdata_next[15:8]  = data_mem[45];
        end else if (state_reg == SENDING_STREAM) begin
            ram_addr_next = count_reg[COUNT_WIDTH-1:2] + 1'b1;
            ram_ce_next = 1'b1;
            case (count_reg[1:0]) begin
                2'd0: ram_we_next = 4'b0001;
                2'd1: ram_we_next = 4'b0010;
                2'd2: ram_we_next = 4'b0100;
                2'd3: ram_we_next = 4'b1000;
            endcase
            ram_wdata_next = {4{rx_raw_eth_axis_tdata}};
        end else if (state_reg == FINISH) begin
            ram_addr_next = {ADDR_WIDTH{1'b0}};
            ram_ce_next = 1'b1;
            ram_we_next = 4'b0011;
            ram_wdata_next[15:0] = count_reg;
        end
    end
    
    always @* begin
        state_next = state_reg;
        count_next = count_reg;

        if (state_reg == IDLE) begin
            count_next = {COUNT_WIDTH{1'b0}};
            if (rx_raw_eth_tx_tvalid) begin
                // If state_reg is IDLE, axis_tready_reg should be 1'b1.
                if (frame_ready_reg) begin
                    // rx_raw_eth_data is not available
                    if (!rx_raw_eth_tx_last) begin
                        state_next = WAIT_LAST;
                    end
                end else begin
                    count_next = 1;
                    if (rx_raw_eth_tx_tlast) begin
                        state_next = SENDING_SHORT_PACKET_00;
                    end else begin
                        state_next = READING;
                    end
                end
            end
        end else if (state_reg == READING) begin
            if (rx_raw_eth_tx_valid) begin
                // If state_reg is READING, axis_tready_reg should be 1'b1.
                count_next = count_reg + 1'b1;
                if (rx_raw_eth_tx_last) begin
                    state_next = SENDING_SHORT_PACKET_00;
                end else if (count_next == DATA_MEM_SIZE) begin
                    state_next = IDENTIFY_PACKET_TYPE;
                end
            end
        end else if (state_reg == IDENTIFY_PACKET_TYPE) begin
            if (!is_arp || !is_ipv4 || is_fragment || !is_rtps) begin
                state_next = SENDING_STREAM;
            end
        end else if (state_reg == SENDING_STREAM) begin
            if (rx_raw_eth_tx_valid) begin
                // If state_reg is SENDING_STREAM, axis_tready_reg should be 1'b1.
                count_next = count_reg + 1'b1;
                if (rx_raw_eth_tx_last) begin
                    state_next = SENDING_MEM_DATA_00;
                end else if (count_next == (`ROS2_MAX_RAW_ETH_RX_DATA_LEN - 4)) begin
                    // The packet is too long, so count length only.
                    state_next = COUNT_ONLY;
                end
            end
        end else if (state_reg == COUNT_ONLY) begin
            if (rx_raw_eth_tx_valid) begin
                // If state_reg is COUNT_ONLY, axis_tready_reg should be 1'b1.
                count_next = count_reg + 1'b1;
                if (rx_raw_eth_tx_last) begin
                    state_next = SENDING_MEM_DATA_00;
                end
            end
        end else if (state_reg == FINISH) begin
            count_next = {COUNT_WIDTH{1'b0}};
            state_next = IDLE;
        end else if (state_reg == WAIT_LAST) begin
            if (rx_raw_eth_tx_valid & rx_raw_eth_tx_last) begin
                // If state_reg is WAIT_LAST, axis_tready_reg should be 1'b1.
                count_next = {COUNT_WIDTH{1'b0}};
                state_next = IDLE;
            end
        end else if ((state_reg == SENDING_SHORT_PACKET_11) || (state_reg == SENDING_MEM_DATA_11)) begin
            state_next = FINISH;
        end
        for (iter = 0; iter < 11; iter = iter + 1) begin
            if ((state_reg == (SENDING_SHORT_PACKET_00 + iter))
                    || (state_reg == (SENDING_MEM_DATA_00 + iter))) begin
                state_next = state_reg + 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= IDLE;
            frame_ready_reg <= 1'b0;
            axis_tready_reg <= 1'b0;
            count_reg <= {COUNT_WIDTH{1'b0}};
            ram_addr_reg <= {ADDR_WIDTH{1'b0}};
            ram_ce_reg <= 1'b0;
            ram_we_reg <= 4'd0;
            ram_wdata_reg <= 32'd0;
        end else begin
            if (!enable) begin
                state_reg <= IDLE;
                frame_ready_reg <= 1'b0;
                axis_tready_reg <= 1'b0;
                count_reg <= {COUNT_WIDTH{1'b0}};
                ram_addr_reg <= {ADDR_WIDTH{1'b0}};
                ram_ce_reg <= 1'b0;
                ram_we_reg <= 4'd0;
                ram_wdata_reg <= 32'd0;
            end else begin
                state_reg <= state_next;
                frame_ready_reg <= frame_ready_next;
                axis_tready_reg <= axis_tready_next;
                count_reg <= count_next;
                ram_addr_reg <= ram_addr_next;
                ram_ce_reg <= ram_ce_next;
                ram_we_reg <= ram_we_next;
                ram_wdata_reg <= ram_wdata_next;
            end
        end
    end

endmodule

`default_nettype wire
