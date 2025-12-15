// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`include "ros2_ether_config.vh"
`default_nettype none

module raw_eth_rx_adapter (
    input  wire clk,
    input  wire rst_n,
    input  wire enable,

    input  wire [7:0] rx_raw_eth_axis_tdata,
    input  wire rx_raw_eth_axis_tvalid,
    output wire rx_raw_eth_axis_tready,
    input  wire rx_raw_eth_axis_tlast,

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
    localparam [4:0] IDLE                       = 5'd0;
    localparam [4:0] READING                    = 5'd1; // Read the first 46 bytes to identify the type of the receiving packet
    localparam [4:0] IDENTIFY_PACKET_TYPE_SHORT = 5'd2; // If packet length <= 46 bytes
    localparam [4:0] IDENTIFY_PACKET_TYPE_LONG  = 5'd3; // If packet length > 46 bytes
    localparam [4:0] SENDING_STREAM             = 5'd4; // Receive and write the remaining packet data
    localparam [4:0] COUNT_ONLY                 = 5'd5; // When the packet length exceeds `ROS2_MAX_RAW_ETH_MAX_DATA_LEN - 2
    localparam [4:0] SENDING_MEM_DATA_00        = 5'd6; // Write data_mem data to the RAM
    localparam [4:0] SENDING_MEM_DATA_01        = 5'd7;
    localparam [4:0] SENDING_MEM_DATA_02        = 5'd8;
    localparam [4:0] SENDING_MEM_DATA_03        = 5'd9;
    localparam [4:0] SENDING_MEM_DATA_04        = 5'd10;
    localparam [4:0] SENDING_MEM_DATA_05        = 5'd11;
    localparam [4:0] SENDING_MEM_DATA_06        = 5'd12;
    localparam [4:0] SENDING_MEM_DATA_07        = 5'd13;
    localparam [4:0] SENDING_MEM_DATA_08        = 5'd14;
    localparam [4:0] SENDING_MEM_DATA_09        = 5'd15;
    localparam [4:0] SENDING_MEM_DATA_10        = 5'd16;
    localparam [4:0] SENDING_MEM_DATA_11        = 5'd17; // Finish writing packet data and become IDLE next
    localparam [4:0] WAIT_LAST                  = 5'd18; // Ignore the receiving packet

    reg  frame_ready_reg;
    wire frame_ready_next = (frame_ready_reg & ~rx_raw_eth_data_ack) | (state_reg == SENDING_MEM_DATA_11);
    assign rx_raw_eth_data_frame_ready = frame_ready_reg;

    reg axis_tready_reg;
    wire axis_tready_next = (state_next == IDLE) | (state_next == READING) | (state_next == SENDING_STREAM) | (state_next == COUNT_ONLY) | (state_next == WAIT_LAST);
    assign rx_raw_eth_axis_tready = axis_tready_reg;

    // Packet length
    localparam COUNT_WIDTH = 16;
    reg [COUNT_WIDTH-1:0] count_reg;
    reg [COUNT_WIDTH-1:0] count_next;

    integer iter;
    localparam DATA_MEM_SIZE = 46; // 14 (ether frame header) + 20 (IPv4 header) + 8 (UDP header) + 4 (RTPS magic)
    reg [7:0] data_mem [0:DATA_MEM_SIZE-1];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (iter = 0; iter < DATA_MEM_SIZE; iter = iter + 1) begin
                data_mem[iter] <= 8'd0;
            end
        end else begin
            if (state_reg == IDLE) begin
                data_mem[0] <= rx_raw_eth_axis_tdata;
                for (iter = 1; iter < DATA_MEM_SIZE; iter = iter + 1) begin
                    data_mem[iter] <= 8'd0;
                end
            end else if ((state_reg == READING) && (count_reg < DATA_MEM_SIZE)) begin
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
    wire is_arp = (data_mem[12] == 8'h08) && (data_mem[13] == 8'h06);
    wire is_ipv4 = (data_mem[12] == 8'h08) && (data_mem[13] == 8'h00) && (data_mem[14][7:4] == 4'd4);
    wire no_header_option = (data_mem[14][3:0] == 4'd5);

    wire [31:0] unicast_mask;
    assign unicast_mask[7:0]   = data_mem[30] ^ ip_addr[7:0];
    assign unicast_mask[15:8]  = data_mem[31] ^ ip_addr[15:8];
    assign unicast_mask[23:16] = data_mem[32] ^ ip_addr[23:16];
    assign unicast_mask[31:24] = data_mem[33] ^ ip_addr[31:24];
    wire [31:0] broadcast_mask;
    assign broadcast_mask[7:0]   = ~data_mem[30];
    assign broadcast_mask[15:8]  = ~data_mem[31];
    assign broadcast_mask[23:16] = ~data_mem[32];
    assign broadcast_mask[31:24] = ~data_mem[33];
    wire is_unicast_to_me = (unicast_mask == 32'd0);
    wire is_multicast = (data_mem[30][7:4] == 4'b1110);
    wire is_direct_broadcast = (((unicast_mask & subnet_mask) | (broadcast_mask & ~subnet_mask)) == 32'd0);
    wire is_limited_broadcast = (data_mem[30] == 8'hff) && (data_mem[31] == 8'hff) && (data_mem[32] == 8'hff) && (data_mem[33] == 8'hff);
    wire is_ipv4_to_me = is_ipv4 && (is_unicast_to_me || is_multicast || is_direct_broadcast || is_limited_broadcast);

    wire more_fragment = data_mem[20][5];
    wire [12:0] fragment_offset = {data_mem[20][4:0], data_mem[21]};
    wire is_fragment = is_ipv4_to_me && (more_fragment || (fragment_offset != 13'd0));
    wire is_udp = (data_mem[23] == 8'd17);
    wire has_rtps_magic = (data_mem[42] == "R") && (data_mem[43] == "T") && (data_mem[44] == "P") && (data_mem[45] == "S");
    wire is_rtps = is_ipv4_to_me && no_header_option && is_udp && has_rtps_magic;

    wire receive_this_packet = (!is_arp && (!is_ipv4 || (is_ipv4_to_me && (is_fragment || !is_rtps))));

    localparam ADDR_WIDTH = $clog2(`ROS2_MAX_RAW_ETH_RX_DATA_LEN) - 2;
    wire [ADDR_WIDTH+1:0] ram_byte_addr = count_reg + 2'd2; // Shift by the packet length size
    reg  [ADDR_WIDTH-1:0] ram_addr_reg;
    reg  [ADDR_WIDTH-1:0] ram_addr_next;
    reg  ram_ce_reg;
    reg  ram_ce_next;
    reg  [3:0] ram_we_reg;
    reg  [3:0] ram_we_next;
    reg  [31:0] ram_wdata_reg;
    reg  [31:0] ram_wdata_next;
    assign rx_raw_eth_data_addr  = ram_addr_reg;
    assign rx_raw_eth_data_ce    = ram_ce_reg;
    assign rx_raw_eth_data_we    = ram_we_reg;
    assign rx_raw_eth_data_wdata = ram_wdata_reg;
    always @* begin
        ram_addr_next = {ADDR_WIDTH{1'b0}};
        ram_ce_next = 1'b0;
        ram_we_next = 4'd0;
        ram_wdata_next = 32'd0;
        if (state_reg == SENDING_STREAM) begin
            ram_addr_next = ram_byte_addr[ADDR_WIDTH+1:2];
            ram_ce_next = rx_raw_eth_axis_tvalid;
            case (ram_byte_addr[1:0])
                2'd0: ram_we_next = 4'b0001;
                2'd1: ram_we_next = 4'b0010;
                2'd2: ram_we_next = 4'b0100;
                2'd3: ram_we_next = 4'b1000;
            endcase
            ram_wdata_next = {4{rx_raw_eth_axis_tdata}};
        end else if (state_reg == SENDING_MEM_DATA_00) begin
            ram_addr_next = 0;
            ram_ce_next = 1'b1;
            ram_we_next = 4'b1111;
            // First two bytes are the packet length
            ram_wdata_next[15:0] = count_reg;
            ram_wdata_next[23:16] = data_mem[0];
            ram_wdata_next[31:24] = data_mem[1];
        end
        for (iter = 1; iter < 12; iter = iter + 1) begin
            if (state_reg == (SENDING_MEM_DATA_00 + iter)) begin
                ram_addr_next = iter;
                ram_ce_next = 1'b1;
                ram_we_next = 4'b1111;
                ram_wdata_next[7:0]   = data_mem[4*iter - 2];
                ram_wdata_next[15:8]  = data_mem[4*iter - 1];
                ram_wdata_next[23:16] = data_mem[4*iter];
                ram_wdata_next[31:24] = data_mem[4*iter + 1];
            end
        end
    end

    always @* begin
        state_next = state_reg;
        count_next = count_reg;

        if (state_reg == IDLE) begin
            count_next = {COUNT_WIDTH{1'b0}};
            if (rx_raw_eth_axis_tvalid) begin
                // If state_reg is IDLE, axis_tready_reg should be 1'b1.
                if (frame_ready_reg) begin
                    // The RAM is not available
                    if (!rx_raw_eth_axis_tlast) begin
                        state_next = WAIT_LAST;
                    end
                end else begin
                    count_next = 1;
                    if (rx_raw_eth_axis_tlast) begin
                        state_next = IDENTIFY_PACKET_TYPE_SHORT;
                    end else begin
                        state_next = READING;
                    end
                end
            end
        end else if (state_reg == READING) begin
            // Receive first 46 bytes to identify the packet type.
            if (rx_raw_eth_axis_tvalid) begin
                // If state_reg is READING, axis_tready_reg should be 1'b1.
                count_next = count_reg + 1'b1;
                if (rx_raw_eth_axis_tlast) begin
                    state_next = IDENTIFY_PACKET_TYPE_SHORT;
                end else if (count_next == DATA_MEM_SIZE) begin
                    state_next = IDENTIFY_PACKET_TYPE_LONG;
                end
            end
        end else if (state_reg == IDENTIFY_PACKET_TYPE_SHORT) begin
            // If the packet length <= 46 bytes
            if (receive_this_packet) begin
                state_next = SENDING_MEM_DATA_00;
            end else begin
                state_next = IDLE;
            end
        end else if (state_reg == IDENTIFY_PACKET_TYPE_LONG) begin
            // If the packet length > 46 bytes
            if (receive_this_packet) begin
                state_next = SENDING_STREAM;
            end else begin
                state_next = WAIT_LAST;
            end
        end else if (state_reg == SENDING_STREAM) begin
            // Receive 47th and following bytes, and write them to the RAM
            if (rx_raw_eth_axis_tvalid) begin
                // If state_reg is SENDING_STREAM, axis_tready_reg should be 1'b1.
                count_next = count_reg + 1'b1;
                if (rx_raw_eth_axis_tlast) begin
                    state_next = SENDING_MEM_DATA_00;
                end else if (count_next == (`ROS2_MAX_RAW_ETH_RX_DATA_LEN - 2)) begin
                    // The packet is too long, so count the packet length only.
                    state_next = COUNT_ONLY;
                end
            end
        end else if (state_reg == COUNT_ONLY) begin
            if (rx_raw_eth_axis_tvalid) begin
                // If state_reg is COUNT_ONLY, axis_tready_reg should be 1'b1.
                count_next = count_reg + 1'b1;
                if (rx_raw_eth_axis_tlast) begin
                    state_next = SENDING_MEM_DATA_00;
                end
            end
        end else if (state_reg == WAIT_LAST) begin
            // Ignore the receiving packet and wait its end.
            if (rx_raw_eth_axis_tvalid & rx_raw_eth_axis_tlast) begin
                // If state_reg is WAIT_LAST, axis_tready_reg should be 1'b1.
                count_next = {COUNT_WIDTH{1'b0}};
                state_next = IDLE;
            end
        end else if (state_reg == SENDING_MEM_DATA_11) begin
            count_next = {COUNT_WIDTH{1'b0}};
            state_next = IDLE;
        end
        for (iter = 0; iter < 11; iter = iter + 1) begin
            if (state_reg == (SENDING_MEM_DATA_00 + iter)) begin
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
