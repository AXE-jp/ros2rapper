`resetall
`default_nettype none
`timescale 1ns / 1ps

module test_1();
    logic clk;
    logic rst_n;
    always begin
        clk = 1'b1;
        #5;
        clk = 1'b0;
        #5;
    end

    wire [210:0] test_input_data[0:58];
    assign test_input_data[0]  = 211'h0_112233445566778899aabb00_0000000000007fffffffffffffff;
    assign test_input_data[1]  = 211'h0_112233445566778899aabb01_0000000000007fffffffffffffff;
    assign test_input_data[2]  = 211'h2_112233445566778899aabb00_0100000000000000000000000000;
    assign test_input_data[3]  = 211'h0_112233445566778899aabb02_0000000000007fffffffffffffff;
    assign test_input_data[4]  = 211'h3_112233445566778899aabb00_0000000000000000000000010000;
    assign test_input_data[5]  = 211'h0_112233445566778899aabb03_0000000000007fffffffffffffff;
    assign test_input_data[6]  = 211'h4_112233445566778899aabb00_0000000000000000000000010000;
    assign test_input_data[7]  = 211'h0_112233445566778899aabb04_0000000000007fffffffffffffff;
    assign test_input_data[8]  = 211'h5_112233445566778899aabb00_0000000000000000010300020000;
    assign test_input_data[9]  = 211'h0_112233445566778899aabb05_0000000000007fffffffffffffff;
    assign test_input_data[10] = 211'h6_112233445566778899aabb00_0000000000000000010400020000;
    assign test_input_data[11] = 211'h0_112233445566778899aabb06_0000000000007fffffffffffffff;
    assign test_input_data[12] = 211'h7_112233445566778899aabb01_0000000000000000000000000000;
    assign test_input_data[13] = 211'h0_112233445566778899aabb07_0000000000007fffffffffffffff;
    assign test_input_data[14] = 211'h1_112233445566778899aabb00_0302000000000000000000000000;
    assign test_input_data[15] = 211'h1_112233445566778899aabb00_0302000000000000000000000000;
    assign test_input_data[16] = 211'h3_112233445566778899aabb00_0000000000000000000000030000;
    assign test_input_data[17] = 211'h1_112233445566778899aabb00_0403000000000000000000000000;
    assign test_input_data[18] = 211'h4_112233445566778899aabb00_0000000000000000000000030000;
    assign test_input_data[19] = 211'h1_112233445566778899aabb00_0403000000000000000000000000;
    assign test_input_data[20] = 211'h5_112233445566778899aabb00_0000000000000000020300040000;
    assign test_input_data[21] = 211'h1_112233445566778899aabb00_0504000000000000000000000000;
    assign test_input_data[22] = 211'h6_112233445566778899aabb00_0000000000000000020400040000;
    assign test_input_data[23] = 211'h1_112233445566778899aabb00_0504000000000000000000000000;
    assign test_input_data[24] = 211'h7_112233445566778899aabb02_0000000000000000000000000000;
    assign test_input_data[25] = 211'h1_112233445566778899aabb00_0504000000000000000000000000;
    assign test_input_data[26] = 211'h2_112233445566778899aabb00_0504000000000000000000000000;
    assign test_input_data[27] = 211'h2_112233445566778899aabb00_0504000000000000000000000000;
    assign test_input_data[28] = 211'h4_112233445566778899aabb00_0000000000000000000000050000;
    assign test_input_data[29] = 211'h2_112233445566778899aabb00_0605000000000000000000000000;
    assign test_input_data[30] = 211'h5_112233445566778899aabb00_0000000000000000030300050000;
    assign test_input_data[31] = 211'h2_112233445566778899aabb00_0605000000000000000000000000;
    assign test_input_data[32] = 211'h6_112233445566778899aabb00_0000000000000000030400060000;
    assign test_input_data[33] = 211'h2_112233445566778899aabb00_0706000000000000000000000000;
    assign test_input_data[34] = 211'h7_112233445566778899aabb03_0000000000000000000000000000;
    assign test_input_data[35] = 211'h2_112233445566778899aabb00_0706000000000000000000000000;
    assign test_input_data[36] = 211'h3_112233445566778899aabb00_0000000000000000000000060000;
    assign test_input_data[37] = 211'h3_112233445566778899aabb00_0000000000000000000000070000;
    assign test_input_data[38] = 211'h5_112233445566778899aabb00_0000000000000000040300080000;
    assign test_input_data[39] = 211'h3_112233445566778899aabb00_0000000000000000000000090000;
    assign test_input_data[40] = 211'h6_112233445566778899aabb00_0000000000000000040400070000;
    assign test_input_data[41] = 211'h3_112233445566778899aabb00_00000000000000000000000a0000;
    assign test_input_data[42] = 211'h7_112233445566778899aabb04_0000000000000000000000000000;
    assign test_input_data[43] = 211'h3_112233445566778899aabb00_00000000000000000000000b0000;
    assign test_input_data[44] = 211'h4_112233445566778899aabb00_0000000000000000000000080000;
    assign test_input_data[45] = 211'h4_112233445566778899aabb00_0000000000000000000000090000;
    assign test_input_data[46] = 211'h6_112233445566778899aabb00_00000000000000000504000a0000;
    assign test_input_data[47] = 211'h4_112233445566778899aabb00_00000000000000000000000b0000;
    assign test_input_data[48] = 211'h7_112233445566778899aabb05_0000000000000000000000000000;
    assign test_input_data[49] = 211'h4_112233445566778899aabb00_00000000000000000000000c0000;
    assign test_input_data[50] = 211'h5_112233445566778899aabb00_00000000000000000503000c0000;
    assign test_input_data[51] = 211'h5_112233445566778899aabb00_00000000000000000603000d0000;
    assign test_input_data[52] = 211'h6_112233445566778899aabb00_00000000000000000604000d0000;
    assign test_input_data[53] = 211'h5_112233445566778899aabb00_00000000000000000703000e0000;
    assign test_input_data[54] = 211'h7_112233445566778899aabb06_0000000000000000000000000000;
    assign test_input_data[55] = 211'h7_112233445566778899aabb07_0000000000000000000000000000;
    assign test_input_data[56] = 211'h6_112233445566778899aabb00_00000000000000000704000e0000;
    assign test_input_data[57] = 211'h6_112233445566778899aabb00_00000000000000000804000f0000;
    assign test_input_data[58] = 211'h7_112233445566778899aabb00_0000000000000000000000000000;

    logic input_data_valid;
    logic input_data_ready;
    logic [210:0] input_data;

    localparam [1:0] STATE_INIT    = 2'd0;
    localparam [1:0] STATE_SENDING = 2'd1;
    localparam [1:0] STATE_FIN     = 2'd2;
    logic [1:0] state;
    logic [5:0] input_data_count;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_data_valid <= 1'b0;
            input_data <= 211'd0;
            state <= STATE_INIT;
            input_data_count <= 6'd0;
        end else begin
            if (state == STATE_INIT) begin
                state <= STATE_SENDING;
                input_data_count <= 6'd1;
                input_data <= test_input_data[input_data_count];
                input_data_valid <= 1'b1;
            end else if (state == STATE_SENDING) begin
                if (input_data_ready) begin
                    if (input_data_count < 6'd59) begin
                        state <= STATE_SENDING;
                        input_data_count <= input_data_count + 1'b1;
                        input_data <= test_input_data[input_data_count];
                        input_data_valid <= 1'b1;
                    end else begin
                        state <= STATE_FIN;
                        input_data_count <= 6'd0;
                        input_data <= 211'd0;
                        input_data_valid <= 1'b0;
                    end
                end
            end
        end
    end

    wire [15:0] ros2_port_num_seed = 16'd7400;
    wire [31:0] ros2_participant_lease_duration_seconds = 32'd20;
    wire [31:0] ros2_participant_lease_duration_fraction = 32'd0;

    wire [$clog2(128*11)-1:0] sedp_reader_tbl_mem_addr;
    wire sedp_reader_tbl_mem_cs;
    wire sedp_reader_tbl_mem_we;
    wire [63:0] sedp_reader_tbl_mem_wdata;
    wire [63:0] sedp_reader_tbl_mem_rdata;
    ram_1rw #(
        .DEPTH(128*11),
        .DWIDTH(64)
    )
    sedp_reader_tbl_mem (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_cs_n(~sedp_reader_tbl_mem_cs),
        .i_we_n(~sedp_reader_tbl_mem_we),
        .i_wmask(8'b11111111),
        .i_addr(sedp_reader_tbl_mem_addr),
        .i_wdata(sedp_reader_tbl_mem_wdata),
        .o_rdata(sedp_reader_tbl_mem_rdata)
    );

    wire [$clog2(128)-1:0] app_reader_tbl_mem_addr;
    wire app_reader_tbl_mem_cs;
    wire app_reader_tbl_mem_we;
    wire [63:0] app_reader_tbl_mem_wdata;
    wire [63:0] app_reader_tbl_mem_rdata;
    ram_1rw #(
        .DEPTH(128),
        .DWIDTH(64)
    )
    app_reader_tbl_mem (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_cs_n(~app_reader_tbl_mem_cs),
        .i_we_n(~app_reader_tbl_mem_we),
        .i_wmask(8'b11111111),
        .i_addr(app_reader_tbl_mem_addr),
        .i_wdata(app_reader_tbl_mem_wdata),
        .o_rdata(app_reader_tbl_mem_rdata)
    );

    ros2_main
    ros2_main_inst (
        .clk(clk),
        .rst_n(rst_n),

        .sedp_reader_tbl_ram_AD1(sedp_reader_tbl_mem_addr),
        .sedp_reader_tbl_ram_CS1(sedp_reader_tbl_mem_cs),
        .sedp_reader_tbl_ram_WE1(sedp_reader_tbl_mem_we),
        .sedp_reader_tbl_ram_WD1(sedp_reader_tbl_mem_wdata),
        .sedp_reader_tbl_ram_RD1(sedp_reader_tbl_mem_rdata),

        .app_reader_tbl_ram_AD1(app_reader_tbl_mem_addr),
        .app_reader_tbl_ram_CS1(app_reader_tbl_mem_cs),
        .app_reader_tbl_ram_WE1(app_reader_tbl_mem_we),
        .app_reader_tbl_ram_WD1(app_reader_tbl_mem_wdata),
        .app_reader_tbl_ram_RD1(app_reader_tbl_mem_rdata),

        .pub_enable(1),
        .sub_enable(1),

        .in_TDATA(input_data),
        .in_TREADY(input_data_ready),
        .in_TVALID(input_data_valid),

        .out_TDATA(),
        .out_TREADY(1'b1),
        .out_TVALID(),

        .conf_port_num_seed(ros2_port_num_seed),
        .conf_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
        .conf_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),

        .cnt_interval_set_wd(),
        .cnt_interval_set_we(),
        .cnt_spdp_wr_set_wd(),
        .cnt_spdp_wr_set_we(),
        .cnt_sedp_pub_wr_set_wd(),
        .cnt_sedp_pub_wr_set_we(),
        .cnt_sedp_sub_wr_set_wd(),
        .cnt_sedp_sub_wr_set_we(),
        .cnt_sedp_pub_hb_set_wd(),
        .cnt_sedp_pub_hb_set_we(),
        .cnt_sedp_sub_hb_set_wd(),
        .cnt_sedp_sub_hb_set_we(),
        .cnt_sedp_pub_an_set_wd(),
        .cnt_sedp_pub_an_set_we(),
        .cnt_sedp_sub_an_set_wd(),
        .cnt_sedp_sub_an_set_we(),
        .cnt_app_wr_set_wd(),
        .cnt_app_wr_set_we(),

        .cnt_interval_elapsed(1'b0),
        .cnt_spdp_wr_elapsed(1'b0),
        .cnt_sedp_pub_wr_elapsed(1'b0),
        .cnt_sedp_sub_wr_elapsed(1'b0),
        .cnt_sedp_pub_hb_elapsed(1'b0),
        .cnt_sedp_sub_hb_elapsed(1'b0),
        .cnt_sedp_pub_an_elapsed(1'b0),
        .cnt_sedp_sub_an_elapsed(1'b0),
        .cnt_app_wr_elapsed(1'b0),

        .timestamp_i64(64'd0),
        .sedp_reader_cnt_wd(),
        .sedp_reader_cnt_we(),
        .app_reader_cnt_wd(),
        .app_reader_cnt_we()
    );

    initial begin
        rst_n = 1'b0;
        #1000;
        rst_n = 1'b1;
    end
endmodule

`resetall
