`resetall
`default_nettype wire
`timescale 1ns / 1ps

module test();
    logic clk;
    logic rst_n;
    always begin
        clk = 1'b1;
        #5;
        clk = 1'b0;
        #5;
    end

    logic ap_start;
    logic ap_ready;
    
    initial begin
        rst_n = 1'b0;
        #1000;
        rst_n = 1'b1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ap_start <= 1'b1;
        end else begin
            if (ap_ready) begin
                ap_start <= 1'b0;
            end
        end
    end
    
    logic [7:0] input_data;
    logic input_valid;
    logic input_ready;
    
    test_input_0 test_input_inst (
        .ap_clk(clk),
        .ap_rst_n(rst_n),
        .ap_start,
        .ap_ready,
        .ap_idle(),
        .ap_done(),
        .out_r_TDATA(input_data),
        .out_r_TVALID(input_valid),
        .out_r_TREADY(input_ready)
    );

    wire [47:0] mac_addr         = 48'h00_00_00_00_00_02;
    wire [31:0] ip_addr          = {8'd100, 8'd1, 8'd168, 8'd192};
    wire [31:0] gateway_ip_addr  = {8'd1, 8'd1, 8'd168, 8'd192};
    wire [31:0] subnet_mask      = {8'd0, 8'd255, 8'd255, 8'd255};
    wire [15:0] ros2_vendor_id = 16'd0; // VENDOR_ID_UNKNOWN
    wire [32*8-1:0] ros2_node_name = "elpmaxe_reppar2sor";
    wire [7:0] ros2_node_name_len = 8'd19;
    wire [15:0] ros2_node_udp_port = 16'd52000;
    wire [15:0] ros2_port_num_seed = 16'd7400;
    wire [31:0] ros2_fragment_expiration = 32'd3333333333;
    wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;
    wire [31:0] ros2_participant_lease_duration_seconds = 32'd20;
    wire [31:0] ros2_participant_lease_duration_fraction = 32'd0;

    // --- ROS2 Pubisher Configuration
    wire [32*8-1:0] ros2_pub_topic_name = "bbb/tr";
    wire [7:0] ros2_pub_topic_name_len = 8'd7;
    wire [64*8-1:0] ros2_pub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len = 8'd29;
    reg [7:0] msg_number;

    localparam [7:0] ROS2_PUB_APP_DATA_STRLEN = 8'd22;
    localparam [10:0] ROS2_PUB_APP_DATA_LEN = ROS2_PUB_APP_DATA_STRLEN + 8'd4;
    wire [1024*8-1:0] ros2_pub_app_data = {msg_number, " - AGPF morF egasseM", 24'b0, ROS2_PUB_APP_DATA_STRLEN}; // Published message
    wire [7:0] ros2_pub_app_data_addr;
    wire ros2_pub_app_data_ce;
    reg  [31:0] ros2_pub_app_data_rdata;
    always @(posedge clk) begin
        if (ros2_pub_app_data_ce) begin
            if (ros2_pub_app_data_addr == 0) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[31:0];
            end else if (ros2_pub_app_data_addr == 1) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[63:32];
            end else if (ros2_pub_app_data_addr == 2) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[95:64];
            end else if (ros2_pub_app_data_addr == 3) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[127:96];
            end else if (ros2_pub_app_data_addr == 4) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[159:128];
            end else if (ros2_pub_app_data_addr == 5) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[191:160];
            end else if (ros2_pub_app_data_addr == 6) begin
                ros2_pub_app_data_rdata <= ros2_pub_app_data[223:192];
            end else begin
                ros2_pub_app_data_rdata <= 32'd0;
            end
        end
    end

    // --- ROS2 Publisher Message Control
    reg ros2_pub_app_data_req_0;
    reg ros2_pub_app_data_rel_0;
    wire ros2_pub_app_data_ack_0;
    wire ros2_pub_app_data_nack_0;
    reg [27:0] msg_change_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msg_number <= 8'd48; // '0'
            ros2_pub_app_data_req_0 <= 0;
            ros2_pub_app_data_rel_0 <= 0;
            msg_change_counter <= 0;
        end else begin
            msg_change_counter <= msg_change_counter + 1;
            ros2_pub_app_data_rel_0 <= 0;

            if (ros2_pub_app_data_req_0 & ros2_pub_app_data_ack_0) begin
                msg_number <= (msg_number == 8'd57) ? 8'd48 : msg_number + 1;
                ros2_pub_app_data_rel_0 <= 1;
                ros2_pub_app_data_req_0 <= 0;
                msg_change_counter <= 0;
            end else if (ros2_pub_app_data_req_0 & ros2_pub_app_data_nack_0) begin
                ros2_pub_app_data_req_0 <= 0;
            end else if (ros2_pub_app_data_rel_0 & ros2_pub_app_data_ack_0) begin
                ros2_pub_app_data_rel_0 <= 0;
            end else if (msg_change_counter[27]) begin
                ros2_pub_app_data_req_0 <= 1;
            end
        end
    end

    wire [3:0] ros2_pub_app_data_req;
    wire [3:0] ros2_pub_app_data_rel;
    wire [3:0] ros2_pub_app_data_ack;
    wire [3:0] ros2_pub_app_data_nack;
    assign ros2_pub_app_data_req[0] = ros2_pub_app_data_req_0;
    assign ros2_pub_app_data_rel[0] = ros2_pub_app_data_rel_0;
    assign ros2_pub_app_data_ack_0 = ros2_pub_app_data_ack[0];
    assign ros2_pub_app_data_nack_0 = ros2_pub_app_data_nack[0];

    // --- ROS2 Subscriber Configuration
    wire [32*8-1:0] ros2_sub_topic_name = "aaa/tr";
    wire [7:0] ros2_sub_topic_name_len = 8'd7;
    wire [64*8-1:0] ros2_sub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_sub_topic_type_name_len = 8'd29;

    // --- ROS2 Subscriber Received Message
    wire [63:0] recvinfo_din;
    wire recvinfo_write;
    reg  [10:0] ros2_sub_app_data_len;
    reg  [15:0] ros2_sub_app_data_rep_id;

    wire [9:0] ros2_sub_app_data_addr;
    wire ros2_sub_app_data_ce;
    wire ros2_sub_app_data_we;
    wire [7:0] ros2_sub_app_data_wdata;
    reg [7:0] rx_msg_reg[0:1024-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ros2_sub_app_data_len <= 0;
            ros2_sub_app_data_rep_id <= 0;
        end else begin
            if (recvinfo_write) begin
                ros2_sub_app_data_len <= recvinfo_din[15:0];
                ros2_sub_app_data_rep_id <= recvinfo_din[31:16];
            end
            if (ros2_sub_app_data_ce & ros2_sub_app_data_we)
                rx_msg_reg[ros2_sub_app_data_addr][7:0] <= ros2_sub_app_data_wdata;
        end
    end

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

    wire payloadsmem_cs;
    wire payloadsmem_we;
    wire [$clog2(2960)-1:0] payloadsmem_addr;
    wire [7:0] payloadsmem_wdata, payloadsmem_rdata;
    ram_1rw #(
        .DEPTH(2960),
        .DWIDTH(8)
    )
    payloadsmem (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_cs_n(~payloadsmem_cs),
        .i_we_n(~payloadsmem_we),
        .i_wmask(4'b1111),
        .i_addr(payloadsmem_addr),
        .i_wdata(payloadsmem_wdata),
        .o_rdata(payloadsmem_rdata)
    );

    localparam ROS2CLK_HZ = 100_000_000;
    localparam PRESCALER_DIV = 64;
    ros2rapper #(
        .SET_TX_PERIOD_BY_PARAMETER (1),
        .PRESCALER_DIV              (PRESCALER_DIV),
        .ROS2CLK_HZ                 (ROS2CLK_HZ),
        .TX_INTERVAL_COUNT          ((ROS2CLK_HZ / PRESCALER_DIV) / 100),
        .TX_PERIOD_SPDP_WR_COUNT    ((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_WR_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_WR_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_HB_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_HB_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_AN_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_AN_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_APP_WR_COUNT     ((ROS2CLK_HZ / PRESCALER_DIV) * 3)
    )
    ros2rapper_inst (
        .clk,
        .rst_n,
        .en(1'b1),
        .ros2pub_en(4'b0001),
        .ros2sub_en(4'b0001),
        .rx_fifo_dout(input_data),
        .rx_fifo_empty(~input_valid),
        .rx_fifo_rd_en(input_ready),
        .tx_fifo_din(),
        .tx_fifo_full(1'b0),
        .tx_fifo_wr_en(),
        .ip_addr,
        .subnet_mask,
        .ros2_vendor_id,
        .ros2_node_name,
        .ros2_node_name_len,
        .ros2_node_udp_port,
        .ros2_port_num_seed,
        .ros2_fragment_expiration(ros2_fragment_expiration),
        .ros2_guid_prefix(ros2_guid_prefix),
        .ros2_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
        .ros2_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),
        .ros2_ignore_ip_checksum(1'b0),

        .ros2_pub_topic_name_0(ros2_pub_topic_name),
        .ros2_pub_topic_name_len_0(ros2_pub_topic_name_len),
        .ros2_pub_topic_type_name_0(ros2_pub_topic_type_name),
        .ros2_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len),

        .ros2_pub_topic_name_1(0),
        .ros2_pub_topic_name_len_1(0),
        .ros2_pub_topic_type_name_1(0),
        .ros2_pub_topic_type_name_len_1(0),

        .ros2_pub_topic_name_2(0),
        .ros2_pub_topic_name_len_2(0),
        .ros2_pub_topic_type_name_2(0),
        .ros2_pub_topic_type_name_len_2(0),

        .ros2_pub_topic_name_3(0),
        .ros2_pub_topic_name_len_3(0),
        .ros2_pub_topic_type_name_3(0),
        .ros2_pub_topic_type_name_len_3(0),

        .ros2_sub_topic_name_0(ros2_sub_topic_name),
        .ros2_sub_topic_name_len_0(ros2_sub_topic_name_len),
        .ros2_sub_topic_type_name_0(ros2_sub_topic_type_name),
        .ros2_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len),

        .ros2_sub_topic_name_1(0),
        .ros2_sub_topic_name_len_1(0),
        .ros2_sub_topic_type_name_1(0),
        .ros2_sub_topic_type_name_len_1(0),

        .ros2_sub_topic_name_2(0),
        .ros2_sub_topic_name_len_2(0),
        .ros2_sub_topic_type_name_2(0),
        .ros2_sub_topic_type_name_len_2(0),

        .ros2_sub_topic_name_3(0),
        .ros2_sub_topic_name_len_3(0),
        .ros2_sub_topic_type_name_3(0),
        .ros2_sub_topic_type_name_len_3(0),

        .ros2_pub_app_data_0_addr(ros2_pub_app_data_addr),
        .ros2_pub_app_data_0_ce(ros2_pub_app_data_ce),
        .ros2_pub_app_data_0_rdata(ros2_pub_app_data_rdata),

        .ros2_pub_app_data_1_addr(),
        .ros2_pub_app_data_1_ce(),
        .ros2_pub_app_data_1_rdata(0),

        .ros2_pub_app_data_2_addr(),
        .ros2_pub_app_data_2_ce(),
        .ros2_pub_app_data_2_rdata(0),

        .ros2_pub_app_data_3_addr(),
        .ros2_pub_app_data_3_ce(),
        .ros2_pub_app_data_3_rdata(0),
        .ros2_pub_app_data_len_0(ROS2_PUB_APP_DATA_LEN),
        .ros2_pub_app_data_len_1(0),
        .ros2_pub_app_data_len_2(0),
        .ros2_pub_app_data_len_3(0),

        .ros2_pub_app_data_req(ros2_pub_app_data_req),
        .ros2_pub_app_data_rel(ros2_pub_app_data_rel),
        .ros2_pub_app_data_ack(ros2_pub_app_data_ack),
        .ros2_pub_app_data_nack(ros2_pub_app_data_nack),
        .ros2_pub_app_data_grant(),

        .ros2_sub_app_data_0_addr(ros2_sub_app_data_addr),
        .ros2_sub_app_data_0_ce(ros2_sub_app_data_ce),
        .ros2_sub_app_data_0_we(ros2_sub_app_data_we),
        .ros2_sub_app_data_0_wdata(ros2_sub_app_data_wdata),

        .ros2_sub_app_data_1_addr(),
        .ros2_sub_app_data_1_ce(),
        .ros2_sub_app_data_1_we(),
        .ros2_sub_app_data_1_wdata(),

        .ros2_sub_app_data_2_addr(),
        .ros2_sub_app_data_2_ce(),
        .ros2_sub_app_data_2_we(),
        .ros2_sub_app_data_2_wdata(),

        .ros2_sub_app_data_3_addr(),
        .ros2_sub_app_data_3_ce(),
        .ros2_sub_app_data_3_we(),
        .ros2_sub_app_data_3_wdata(),

        .ros2_sub_app_data_recvinfo_din(recvinfo_din),
        .ros2_sub_app_data_recvinfo_full_n(1'b1),
        .ros2_sub_app_data_recvinfo_write(recvinfo_write),

        .ros2_sub_app_data_req(0),
        .ros2_sub_app_data_rel(0),
        .ros2_sub_app_data_ack(),
        .ros2_sub_app_data_nack(),
        .ros2_sub_app_data_grant(),

        .ros2_sedp_reader_cnt(),
        .ros2_app_reader_cnt(),

        .sedp_reader_tbl_mem_addr(sedp_reader_tbl_mem_addr),
        .sedp_reader_tbl_mem_ce(sedp_reader_tbl_mem_cs),
        .sedp_reader_tbl_mem_we(sedp_reader_tbl_mem_we),
        .sedp_reader_tbl_mem_wdata(sedp_reader_tbl_mem_wdata),
        .sedp_reader_tbl_mem_rdata(sedp_reader_tbl_mem_rdata),

        .app_reader_tbl_mem_addr(app_reader_tbl_mem_addr),
        .app_reader_tbl_mem_ce(app_reader_tbl_mem_cs),
        .app_reader_tbl_mem_we(app_reader_tbl_mem_we),
        .app_reader_tbl_mem_wdata(app_reader_tbl_mem_wdata),
        .app_reader_tbl_mem_rdata(app_reader_tbl_mem_rdata),

        .ip_payloadsmem_addr(payloadsmem_addr),
        .ip_payloadsmem_ce(payloadsmem_cs),
        .ip_payloadsmem_we(payloadsmem_we),
        .ip_payloadsmem_wdata(payloadsmem_wdata),
        .ip_payloadsmem_rdata(payloadsmem_rdata)
    );
endmodule

`resetall
