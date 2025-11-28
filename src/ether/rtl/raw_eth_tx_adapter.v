// Copyright (c) 2021-2024 AXE, Inc.
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
    input  wire [$clog2(`ROS2_MAX_RAW_ETH_TX_DATA_LEN+1)-1:0] tx_raw_eth_data_len,
    input  wire tx_raw_eth_frame_ready,
    output wire tx_raw_eth_completed,

    output reg  [7:0] tx_raw_eth_axis_tdata,
    output reg  tx_raw_eth_axis_tvalid,
    input  wire tx_raw_eth_axis_tready,
    output reg  tx_raw_eth_axis_tlast
);
    reg [2:0] r_state;
    reg [2:0] w_state_next;
    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] WRITE_0 = 3'd1;
    localparam [2:0] WRITE_1 = 3'd2;
    localparam [2:0] WRITE_2 = 3'd3;
    localparam [2:0] WRITE_3 = 3'd4;

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

    reg r_completed;
    reg w_completed_next;
    assign tx_raw_eth_completed = r_completed;

    wire [31:0] w_rdata_new;
    wire w_rdata_new_valid;
    reg  w_rdata_new_ready;

    raw_eth_tx_adapter_fifo #(
        .MAX_DATA_LEN(`ROS2_MAX_RAW_ETH_TX_DATA_LEN/4),
        .DATA_WIDTH(32)
    )
    raw_eth_tx_adapter_fifo_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(r_state != IDLE),
        .rom_addr(tx_raw_eth_data_addr),
        .rom_ce(tx_raw_eth_data_ce),
        .rom_rdata(tx_raw_eth_data_rdata),
        .out_tdata(w_rdata_new),
        .out_tvalid(w_rdata_new_valid),
        .out_tready(w_rdata_new_ready)
    );

    localparam COUNT_WIDTH = $clog2(`ROS2_MAX_RAW_ETH_TX_DATA_LEN);
    reg [COUNT_WIDTH-1:0] r_count;
    reg [COUNT_WIDTH-1:0] w_count_next;
    reg [31:0] r_rdata;
    reg [31:0] w_rdata_next;

    always @* begin
        tx_raw_eth_axis_tlast = (r_count == {COUNT_WIDTH{1'b0}});

        w_state_next = r_state;
        w_completed_next = r_completed;
        w_count_next = r_count;
        w_rdata_next = r_rdata;

        w_rdata_new_ready = 1'b0;
        tx_raw_eth_axis_tdata = 8'd0;
        tx_raw_eth_axis_tvalid = 1'b0;

        if (r_state == IDLE) begin
            if (tx_raw_eth_kick) begin
                if ((tx_raw_eth_data_len != 0) && (tx_raw_eth_data_len <= `ROS2_MAX_RAW_ETH_TX_DATA_LEN)) begin
                    w_count_next = tx_raw_eth_data_len - 1'b1;
                    w_state_next = WRITE_0;
                end else begin
                    // Do nothing whe tx_raw_eth_data_len is invalid
                    w_completed_next = 1'b1;
                end
            end
        end else if (r_state == WRITE_0) begin
            w_rdata_new_ready = tx_raw_eth_axis_tready;
            w_rdata_next = w_rdata_new;
            tx_raw_eth_axis_tdata = w_rdata_new[7:0];
            tx_raw_eth_axis_tvalid = w_rdata_new_valid;
            if (w_rdata_new_valid && tx_raw_eth_axis_tready) begin
                if (tx_raw_eth_axis_tlast) begin
                    w_completed_next = 1'b1;
                    w_state_next = IDLE;
                end else begin
                    w_count_next = r_count - 1'b1;
                    w_state_next = WRITE_1;
                end
            end
        end else if (r_state == WRITE_1) begin
            tx_raw_eth_axis_tdata = r_rdata[15:8];
            tx_raw_eth_axis_tvalid = 1'b1;
            if (tx_raw_eth_axis_tready) begin
                if (tx_raw_eth_axis_tlast) begin
                    w_completed_next = 1'b1;
                    w_state_next = IDLE;
                end else begin
                    w_count_next = r_count - 1'b1;
                    w_state_next = WRITE_2;
                end
            end
        end else if (r_state == WRITE_2) begin
            tx_raw_eth_axis_tdata = r_rdata[23:16];
            tx_raw_eth_axis_tvalid = 1'b1;
            if (tx_raw_eth_axis_tready) begin
                if (tx_raw_eth_axis_tlast) begin
                    w_completed_next = 1'b1;
                    w_state_next = IDLE;
                end else begin
                    w_count_next = r_count - 1'b1;
                    w_state_next = WRITE_3;
                end
            end
        end else if (r_state == WRITE_3) begin
            tx_raw_eth_axis_tdata = r_rdata[31:24];
            tx_raw_eth_axis_tvalid = 1'b1;
            if (tx_raw_eth_axis_tready) begin
                if (tx_raw_eth_axis_tlast) begin
                    w_completed_next = 1'b1;
                    w_state_next = IDLE;
                end else begin
                    w_count_next = r_count - 1'b1;
                    w_state_next = WRITE_0;
                end
            end
        end else begin
            w_completed_next = 1'b1;
            w_state_next = IDLE;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state <= IDLE;
            r_completed <= 1'b0;
            r_count <= {COUNT_WIDTH{1'b0}};
            r_rdata <= 32'd0;
        end else begin
            if (!enable) begin
                r_state <= IDLE;
                r_completed <= 1'b0;
                r_count <= {COUNT_WIDTH{1'b0}};
                r_rdata <= 32'd0;
            end else begin
                r_state <= w_state_next;
                // deassert completed when frame_ready is deasserted
                r_completed <= w_completed_next & tx_raw_eth_frame_ready;
                r_count <= w_count_next;
                r_rdata <= w_rdata_next;
            end
        end
    end
endmodule

// Read data from a ROM
module raw_eth_tx_adapter_read #(
    parameter ADDR_WIDTH = 9,
    parameter DATA_WIDTH = 32
)
(
    input  wire clk,
    input  wire rst_n,

    output wire [ADDR_WIDTH-1:0] rom_addr,
    output wire rom_ce,
    input  wire [DATA_WIDTH-1:0] rom_rdata,

    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire addr_valid,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire rdata_valid
);
    reg [ADDR_WIDTH-1:0] r_rom_addr;
    reg r_rom_ce;
    reg r_rdata_valid;

    assign rom_addr = r_rom_addr;
    assign rom_ce = r_rom_ce;
    assign rdata = rom_rdata;
    assign rdata_valid = r_rdata_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_rom_addr <= {ADDR_WIDTH{1'b0}};
            r_rom_ce <= 1'b0;
            r_rdata_valid <= 1'b0;
        end else begin
            r_rom_addr <= addr;
            r_rom_ce <= addr_valid;
            r_rdata_valid <= r_rom_ce;
        end
    end
endmodule

// Read data from a ROM, and output it.
module raw_eth_tx_adapter_fifo #(
    parameter MAX_DATA_LEN=400,
    parameter DATA_WIDTH=32
)
(
    input  wire clk,
    input  wire rst_n,
    input  wire start,

    // ROM ports
    output wire [$clog2(MAX_DATA_LEN)-1:0] rom_addr,
    output wire rom_ce,
    input  wire [DATA_WIDTH-1:0] rom_rdata,

    // AXIS ports
    output reg  [DATA_WIDTH-1:0] out_tdata,
    output reg  out_tvalid,
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
        .rom_addr(rom_addr),
        .rom_ce(rom_ce),
        .rom_rdata(rom_rdata),
        .addr(r_addr),
        .addr_valid(w_addr_valid),
        .rdata(w_rdata_new),
        .rdata_valid(w_rdata_new_valid)
    );

    // FIFO (w_rdata_new -> r_rdata[0] -> r_rdata[1] -> out)
    reg [DATA_WIDTH-1:0] r_rdata [0:1];
    reg [1:0] r_rdata_valid;

    reg [DATA_WIDTH-1:0] w_rdata_next [0:1];
    reg [1:0] w_rdata_next_valid;

    always @* begin
        if (r_rdata_valid[1]) begin
            out_tdata  = r_rdata[1];
            out_tvalid = r_rdata_valid[1];
        end else begin
            out_tdata  = w_rdata_new;
            out_tvalid = w_rdata_new_valid;
        end

        w_rdata_next[1]       = r_rdata[1];
        w_rdata_next_valid[1] = r_rdata_valid[1];
        w_rdata_next[0]       = r_rdata[0];
        w_rdata_next_valid[0] = r_rdata_valid[0];
        if (!out_tready) begin
            case (r_rdata_valid)
                2'b00: begin
                    w_rdata_next[1]       = w_rdata_new;
                    w_rdata_next_valid[1] = w_rdata_new_valid;
                end
                2'b10: begin
                    w_rdata_next[0]       = w_rdata_new;
                    w_rdata_next_valid[0] = w_rdata_new_valid;
                end
            endcase
        end else begin
            case (r_rdata_valid)
                2'b10: begin
                    w_rdata_next[1]       = w_rdata_new;
                    w_rdata_next_valid[1] = w_rdata_new_valid;
                end
                2'b11: begin
                    w_rdata_next[1]       = r_rdata[0];
                    w_rdata_next_valid[1] = 1'b1;
                    w_rdata_next[0]       = w_rdata_new;
                    w_rdata_next_valid[0] = w_rdata_new_valid;
                end
            endcase
        end
    end
    // w_addr_valid is asserted if and only if
    // r_rdata_valid[0] is guaranteed to be 1'b0 after two clock cycles.
    assign w_addr_valid = enable & ~w_rdata_next_valid[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_addr <= {ADDR_WIDTH{1'b0}};
            r_rdata_valid <= 2'd0;
            r_rdata[0] <= {DATA_WIDTH{1'b0}};
            r_rdata[1] <= {DATA_WIDTH{1'b0}};
        end else begin
            if (!start) begin
                r_addr <= {ADDR_WIDTH{1'b0}};
                r_rdata_valid <= 2'd0;
                r_rdata[0] <= {DATA_WIDTH{1'b0}};
                r_rdata[1] <= {DATA_WIDTH{1'b0}};
            end else begin
                if (w_addr_valid) begin
                    if (r_addr + 1'b1 == MAX_DATA_LEN) begin
                        r_addr <= {ADDR_WIDTH{1'b0}};
                    end else begin
                        r_addr <= r_addr + 1'b1;
                    end
                end
                r_rdata_valid <= w_rdata_next_valid;
                r_rdata[0] <= w_rdata_next[0];
                r_rdata[1] <= w_rdata_next[1];
            end
        end
    end
endmodule

`default_nettype wire
