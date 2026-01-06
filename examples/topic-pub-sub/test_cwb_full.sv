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

    wire [210:0] test_input_data[0:64];
    // SPDP from node 1 (GUID prefix: 0x111111111111111111111111)
    // IP address: 0.0.0.0, UDP port: 0, participant lease duration: INFINITY
    assign test_input_data[0]  = 211'h0_111111111111111111111111_0000000000007fffffffffffffff;
    // SPDP from node 2 (GUID prefix: 0x222222222222222222222222)
    assign test_input_data[1]  = 211'h0_222222222222222222222222_0000000000007fffffffffffffff;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 1, last_sn: 0)
    assign test_input_data[2]  = 211'h1_111111111111111111111111_0100000000000000000000000000;
    // SPDP from node 3 (GUID prefix: 0x333333333333333333333333)
    assign test_input_data[3]  = 211'h0_333333333333333333333333_0000000000007fffffffffffffff;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 1, last_sn: 0)
    assign test_input_data[4]  = 211'h2_111111111111111111111111_0100000000000000000000000000;
    // SPDP from node 4 (GUID prefix: 0x444444444444444444444444)
    assign test_input_data[5]  = 211'h0_444444444444444444444444_0000000000007fffffffffffffff;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 1)
    assign test_input_data[6]  = 211'h3_111111111111111111111111_0000000000000000000000010000;
    // SPDP from node 5 (GUID prefix: 0x555555555555555555555555)
    assign test_input_data[7]  = 211'h0_555555555555555555555555_0000000000007fffffffffffffff;
    // SEDP from node 1 (sub, not published topic, sequence numer: 1)
    assign test_input_data[8]  = 211'h4_111111111111111111111111_0000000000000000000000010000;
    // SPDP from node 6 (GUID prefix: 0x666666666666666666666666)
    assign test_input_data[9]  = 211'h0_666666666666666666666666_0000000000007fffffffffffffff;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x11111103, sequence number: 2)
    // IP address: 0.0.0.0, UDP port: 0
    assign test_input_data[10] = 211'h5_111111111111111111111111_0000000000001111110300020000;
    // SPDP from node 7 (GUID prefix: 0x777777777777777777777777)
    assign test_input_data[11] = 211'h0_777777777777777777777777_0000000000007fffffffffffffff;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x11111104, sequence number: 2)
    // IP address: 0.0.0.0, UDP port: 0
    assign test_input_data[12] = 211'h6_111111111111111111111111_0000000000001111110400020000;
    // SPDP from node 8 (GUID prefix: 0x888888888888888888888888)
    assign test_input_data[13] = 211'h0_888888888888888888888888_0000000000007fffffffffffffff;
    // Node 2 leaves
    assign test_input_data[14] = 211'h7_222222222222222222222222_0000000000000000000000000000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 3, last_sn: 2)
    assign test_input_data[15] = 211'h1_111111111111111111111111_0302000000000000000000000000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 3, last_sn: 2)
    assign test_input_data[16] = 211'h1_111111111111111111111111_0302000000000000000000000000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 3, last_sn: 2)
    assign test_input_data[17] = 211'h2_111111111111111111111111_0302000000000000000000000000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 3, last_sn: 2)
    assign test_input_data[18] = 211'h1_111111111111111111111111_0302000000000000000000000000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 3)
    assign test_input_data[19] = 211'h3_111111111111111111111111_0000000000000000000000030000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 4, last_sn: 3)
    assign test_input_data[20] = 211'h1_111111111111111111111111_0403000000000000000000000000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 3)
    assign test_input_data[21] = 211'h4_111111111111111111111111_0000000000000000000000030000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 4, last_sn: 3)
    assign test_input_data[22] = 211'h1_111111111111111111111111_0403000000000000000000000000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x22222203, sequence number: 4)
    assign test_input_data[23] = 211'h5_111111111111111111111111_0000000000002222220300040000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 5, last_sn: 4)
    assign test_input_data[24] = 211'h1_111111111111111111111111_0504000000000000000000000000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x22222204, sequence number: 4)
    assign test_input_data[25] = 211'h6_111111111111111111111111_0000000000002222220400040000;
    // SEDP HEARTBEAT (pub) from node 1 (first_sn: 5, last_sn: 4)
    assign test_input_data[26] = 211'h1_111111111111111111111111_0504000000000000000000000000;
    // Node 3 leaves
    assign test_input_data[27] = 211'h7_333333333333333333333333_0000000000000000000000000000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 5, last_sn: 4)
    assign test_input_data[28] = 211'h2_111111111111111111111111_0504000000000000000000000000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 5, last_sn: 4)
    assign test_input_data[29] = 211'h2_111111111111111111111111_0504000000000000000000000000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 5)
    assign test_input_data[30] = 211'h3_111111111111111111111111_0000000000000000000000050000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 5, last_sn: 4)
    assign test_input_data[31] = 211'h2_111111111111111111111111_0504000000000000000000000000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 5)
    assign test_input_data[32] = 211'h4_111111111111111111111111_0000000000000000000000050000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 6, last_sn: 5)
    assign test_input_data[33] = 211'h2_111111111111111111111111_0605000000000000000000000000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x33333303, sequence number: 6)
    assign test_input_data[34] = 211'h5_111111111111111111111111_0000000000003333330300060000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 6, last_sn: 5)
    assign test_input_data[35] = 211'h2_111111111111111111111111_0605000000000000000000000000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x33333304, sequence number: 6)
    assign test_input_data[36] = 211'h6_111111111111111111111111_0000000000003333330400060000;
    // SEDP HEARTBEAT (sub) from node 1 (first_sn: 7, last_sn: 6)
    assign test_input_data[37] = 211'h2_111111111111111111111111_0706000000000000000000000000;
    // Node 4 leaves
    assign test_input_data[38] = 211'h7_444444444444444444444444_0000000000000000000000000000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 7)
    assign test_input_data[39] = 211'h3_111111111111111111111111_0000000000000000000000070000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 8)
    assign test_input_data[40] = 211'h3_111111111111111111111111_0000000000000000000000080000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 7)
    assign test_input_data[41] = 211'h4_111111111111111111111111_0000000000000000000000070000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 9)
    assign test_input_data[42] = 211'h3_111111111111111111111111_0000000000000000000000090000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x44444403, sequence number: 10)
    assign test_input_data[43] = 211'h5_111111111111111111111111_00000000000044444403000a0000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 11)
    assign test_input_data[44] = 211'h3_111111111111111111111111_00000000000000000000000b0000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x44444404, sequence number: 8)
    assign test_input_data[45] = 211'h6_111111111111111111111111_0000000000004444440400080000;
    // SEDP from node 1 (pub, not subscribed topic, sequence numer: 12)
    assign test_input_data[46] = 211'h3_111111111111111111111111_00000000000000000000000c0000;
    // Node 5 leaves
    assign test_input_data[47] = 211'h7_555555555555555555555555_0000000000000000000000000000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 9)
    assign test_input_data[48] = 211'h4_111111111111111111111111_0000000000000000000000090000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 10)
    assign test_input_data[49] = 211'h4_111111111111111111111111_00000000000000000000000a0000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x55555503, sequence number: 13)
    assign test_input_data[50] = 211'h5_111111111111111111111111_00000000000055555503000d0000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 11)
    assign test_input_data[51] = 211'h4_111111111111111111111111_00000000000000000000000b0000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x55555504, sequence number: 12)
    assign test_input_data[52] = 211'h6_111111111111111111111111_00000000000055555504000c0000;
    // SEDP from node 1 (sub, not published topic, sequence numer: 13)
    assign test_input_data[53] = 211'h4_111111111111111111111111_00000000000000000000000d0000;
    // Node 6 leaves
    assign test_input_data[54] = 211'h7_666666666666666666666666_0000000000000000000000000000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x66666603, sequence number: 14)
    assign test_input_data[55] = 211'h5_111111111111111111111111_00000000000066666603000e0000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x77777703, sequence number: 15)
    assign test_input_data[56] = 211'h5_111111111111111111111111_00000000000077777703000f0000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x66666604, sequence number: 14)
    assign test_input_data[57] = 211'h6_111111111111111111111111_00000000000066666604000e0000;
    // SEDP from node 1 (pub, topic_id: 0, entity_id: 0x88888803, sequence number: 16)
    assign test_input_data[58] = 211'h5_111111111111111111111111_0000000000008888880300100000;
    // Node 7 leaves
    assign test_input_data[59] = 211'h7_777777777777777777777777_0000000000000000000000000000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x77777704, sequence number: 15)
    assign test_input_data[60] = 211'h6_111111111111111111111111_00000000000077777704000f0000;
    // SEDP from node 1 (sub, topic_id: 0, entity_id: 0x88888804, sequence number: 16)
    assign test_input_data[61] = 211'h6_111111111111111111111111_0000000000008888880400100000;
    // Node 8 leaves
    assign test_input_data[62] = 211'h7_888888888888888888888888_0000000000000000000000000000;
    // Node 1 leaves
    assign test_input_data[63] = 211'h7_111111111111111111111111_0000000000000000000000000000;
    // SPDP from node 9 (GUID prefix: 0x999999999999999999999999)
    assign test_input_data[64] = 211'h0_999999999999999999999999_0000000000007fffffffffffffff;

    logic input_data_valid;
    logic input_data_ready;
    logic [210:0] input_data;

    localparam [1:0] STATE_INIT    = 2'd0;
    localparam [1:0] STATE_SENDING = 2'd1;
    localparam [1:0] STATE_FIN     = 2'd2;
    logic [1:0] state;
    logic [6:0] input_data_count;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_data_valid <= 1'b0;
            input_data <= 211'd0;
            state <= STATE_INIT;
            input_data_count <= 7'd0;
        end else begin
            if (state == STATE_INIT) begin
                state <= STATE_SENDING;
                input_data_count <= 7'd1;
                input_data <= test_input_data[input_data_count];
                input_data_valid <= 1'b1;
            end else if (state == STATE_SENDING) begin
                if (input_data_ready) begin
                    if (input_data_count <= 7'd64) begin
                        state <= STATE_SENDING;
                        input_data_count <= input_data_count + 1'b1;
                        input_data <= test_input_data[input_data_count];
                        input_data_valid <= 1'b1;
                    end else begin
                        state <= STATE_FIN;
                        input_data_count <= 7'd0;
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
        repeat(100) @(posedge clk);
        rst_n = 1'b1;
        while (state != STATE_FIN) @(posedge clk);
        repeat(1000) @(posedge clk);
        $finish;
    end
endmodule

`resetall
