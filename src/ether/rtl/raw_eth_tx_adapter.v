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
    input  wire tx_raw_eth_frame_ready,
    output wire tx_raw_eth_completed,

    output wire [7:0] tx_raw_eth_axis_tdata,
    output wire tx_raw_eth_axis_tvalid,
    input  wire tx_raw_eth_axis_tready,
    output wire tx_raw_eth_axis_tlast
);
    reg [2:0] state_reg;
    reg [2:0] state_next;
    localparam [2:0] IDLE         = 3'd0;
    localparam [2:0] CHECK_LENGTH = 3'd1;
    localparam [2:0] WRITE_0      = 3'd2;
    localparam [2:0] WRITE_1      = 3'd3;
    localparam [2:0] WRITE_2      = 3'd4;
    localparam [2:0] WRITE_3      = 3'd5;

    localparam COUNT_WIDTH = $clog2(`ROS2_MAX_RAW_ETH_TX_DATA_LEN);
    reg [COUNT_WIDTH-1:0] count_reg;
    reg [COUNT_WIDTH-1:0] count_next;

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

    reg completed_reg;
    reg completed_next;
    assign tx_raw_eth_completed = completed_reg;

    wire [31:0] rdata_new;
    wire rdata_new_valid;
    reg  rdata_new_ready;

    reg  [7:0] eth_axis_tdata;
    reg  eth_axis_tvalid;
    assign tx_raw_eth_axis_tdata = eth_axis_tdata;
    assign tx_raw_eth_axis_tvalid = eth_axis_tvalid;
    assign tx_raw_eth_axis_tlast = (count_reg == 0);

    raw_eth_tx_adapter_fifo #(
        .MAX_DATA_LEN(`ROS2_MAX_RAW_ETH_TX_DATA_LEN/4),
        .DATA_WIDTH(32)
    )
    raw_eth_tx_adapter_fifo_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(state_reg != IDLE),
        .rom_addr(tx_raw_eth_data_addr),
        .rom_ce(tx_raw_eth_data_ce),
        .rom_rdata(tx_raw_eth_data_rdata),
        .out_tdata(rdata_new),
        .out_tvalid(rdata_new_valid),
        .out_tready(rdata_new_ready)
    );

    reg [31:0] rdata_reg;
    reg [31:0] rdata_next;

    always @* begin
        state_next = state_reg;
        completed_next = completed_reg;
        count_next = count_reg;
        rdata_next = rdata_reg;

        rdata_new_ready = 1'b0;
        eth_axis_tdata = 8'd0;
        eth_axis_tvalid = 1'b0;

        if (state_reg == IDLE) begin
            if (tx_raw_eth_kick) begin
                state_next = CHECK_LENGTH;
            end
        end else if (state_reg == CHECK_LENGTH) begin
            // Read from raw_eth_tx_adapter_fifo
            rdata_new_ready = 1'b1;
            rdata_next = rdata_new;
            if (rdata_new_valid) begin
                // Check frame data length
                if ((rdata_new[15:0] != 0) && (rdata_new[15:0] <= (`ROS2_MAX_RAW_ETH_TX_DATA_LEN - 2))) begin
                    count_next = rdata_new[15:0] - 1'b1;
                    state_next = WRITE_2;
                end else begin
                    // Do nothing if tx_raw_eth_data_len is invalid
                    completed_next = 1'b1;
                    state_next = IDLE;
                end
            end
        end else if (state_reg == WRITE_0) begin
            // Read 1 word (32 bit) data from raw_eth_tx_adapter_fifo
            // and set it to rdata_reg
            rdata_new_ready = tx_raw_eth_axis_tready;
            rdata_next = rdata_new;
            // Write the first byte of rdata_new
            eth_axis_tdata = rdata_new[7:0];
            eth_axis_tvalid = rdata_new_valid;
            if (rdata_new_valid && tx_raw_eth_axis_tready) begin
                if (count_reg == 0) begin
                    completed_next = 1'b1;
                    state_next = IDLE;
                end else begin
                    count_next = count_reg - 1'b1;
                    state_next = WRITE_1;
                end
            end
        end else if (state_reg == WRITE_1) begin
            // Write the second byte of rdata_reg
            eth_axis_tdata = rdata_reg[15:8];
            eth_axis_tvalid = 1'b1;
            if (tx_raw_eth_axis_tready) begin
                if (count_reg == 0) begin
                    completed_next = 1'b1;
                    state_next = IDLE;
                end else begin
                    count_next = count_reg - 1'b1;
                    state_next = WRITE_2;
                end
            end
        end else if (state_reg == WRITE_2) begin
            // Write the third byte of rdata_reg
            eth_axis_tdata = rdata_reg[23:16];
            eth_axis_tvalid = 1'b1;
            if (tx_raw_eth_axis_tready) begin
                if (count_reg == 0) begin
                    completed_next = 1'b1;
                    state_next = IDLE;
                end else begin
                    count_next = count_reg - 1'b1;
                    state_next = WRITE_3;
                end
            end
        end else if (state_reg == WRITE_3) begin
            // Write the last byte of rdata_reg
            eth_axis_tdata = rdata_reg[31:24];
            eth_axis_tvalid = 1'b1;
            if (tx_raw_eth_axis_tready) begin
                if (count_reg == 0) begin
                    completed_next = 1'b1;
                    state_next = IDLE;
                end else begin
                    count_next = count_reg - 1'b1;
                    state_next = WRITE_0;
                end
            end
        end else begin
            completed_next = 1'b1;
            state_next = IDLE;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= IDLE;
            completed_reg <= 1'b0;
            count_reg <= {COUNT_WIDTH{1'b0}};
            rdata_reg <= 32'd0;
        end else begin
            if (!enable) begin
                state_reg <= IDLE;
                completed_reg <= 1'b0;
                count_reg <= {COUNT_WIDTH{1'b0}};
                rdata_reg <= 32'd0;
            end else begin
                state_reg <= state_next;
                // deassert completed_reg when frame_ready is deasserted
                completed_reg <= completed_next & tx_raw_eth_frame_ready;
                count_reg <= count_next;
                rdata_reg <= rdata_next;
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
        .rom_addr(rom_addr),
        .rom_ce(rom_ce),
        .rom_rdata(rom_rdata),
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
