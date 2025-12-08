// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"
`include "ros2_ether_config.vh"

module top (
    input  wire       clk,
    input  wire       rst_n,

    output wire       phy_ref_clk,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    output wire       phy_rst_n,

    output wire       led4,
    output wire       led5,
    output wire       led6,
    output wire       led7
);

    // --- Clock & Reset
    wire clk_int;
    wire clk_25mhz_int;
    wire rst_n_int;
    wire mmcm_locked;
    wire mmcm_clkfb;

    assign phy_ref_clk = clk_25mhz_int;

    localparam ROS2CLK_HZ = 80_000_000;
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKOUT0_DIVIDE_F(12.5),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0),
        .CLKOUT1_DIVIDE(40),
        .CLKOUT1_DUTY_CYCLE(0.5),
        .CLKOUT1_PHASE(0),
        .CLKOUT2_DIVIDE(1),
        .CLKOUT2_DUTY_CYCLE(0.5),
        .CLKOUT2_PHASE(0),
        .CLKOUT3_DIVIDE(1),
        .CLKOUT3_DUTY_CYCLE(0.5),
        .CLKOUT3_PHASE(0),
        .CLKOUT4_DIVIDE(1),
        .CLKOUT4_DUTY_CYCLE(0.5),
        .CLKOUT4_PHASE(0),
        .CLKOUT5_DIVIDE(1),
        .CLKOUT5_DUTY_CYCLE(0.5),
        .CLKOUT5_PHASE(0),
        .CLKOUT6_DIVIDE(1),
        .CLKOUT6_DUTY_CYCLE(0.5),
        .CLKOUT6_PHASE(0),
        .CLKFBOUT_MULT_F(10),
        .CLKFBOUT_PHASE(0),
        .DIVCLK_DIVIDE(1),
        .REF_JITTER1(0.010),
        .CLKIN1_PERIOD(10.0),
        .STARTUP_WAIT("FALSE"),
        .CLKOUT4_CASCADE("FALSE")
    )
    clk_mmcm_inst (
        .CLKIN1(clk),
        .CLKFBIN(mmcm_clkfb),
        .RST(~rst_n),
        .PWRDWN(1'b0),
        .CLKOUT0(clk_int),
        .CLKOUT0B(),
        .CLKOUT1(clk_25mhz_int),
        .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .CLKFBOUT(mmcm_clkfb),
        .CLKFBOUTB(),
        .LOCKED(mmcm_locked)
    );

    reg [3:0] sync_rst_reg;
    assign rst_n_int = sync_rst_reg[3];

    always @(posedge clk_int or negedge rst_n) begin
        if (!rst_n) begin
            sync_rst_reg <= 0;
        end else begin
            sync_rst_reg <= {sync_rst_reg[2:0], mmcm_locked};
        end
    end

    // --- Ethernet Configuration
    wire [47:0] mac_addr         = 48'h00_00_00_00_00_02;
    wire [31:0] ip_addr          = {8'd100, 8'd1, 8'd168, 8'd192};
    wire [31:0] gateway_ip_addr  = {8'd1, 8'd1, 8'd168, 8'd192};
    wire [31:0] subnet_mask      = {8'd0, 8'd255, 8'd255, 8'd255};

    // --- ARP Configuration
    localparam ARP_REQUEST_RETRY_COUNT = 4;
    localparam ARP_REQUEST_RETRY_INTERVAL = (125000000*2);
    localparam ARP_REQUEST_TIMEOUT = (125000000*30);

    // --- ROS2 Node Configuration
    wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name = "elpmaxe_reppar2sor";
    wire [7:0] ros2_node_name_len = 8'd19;
    wire [15:0] ros2_node_udp_port = 16'd52000;
    wire [15:0] ros2_port_num_seed = 16'd7400;
    wire [31:0] ros2_fragment_expiration = 32'd3333333333;
    wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;
    wire [31:0] ros2_participant_lease_duration_seconds = 32'd20;
    wire [31:0] ros2_participant_lease_duration_fraction = 32'd0;

    // --- ROS2 Pubisher Configuration
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name = "bbb/tr";
    wire [7:0] ros2_pub_topic_name_len = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len = 8'd29;
    reg [7:0] msg_number;

    localparam [7:0] ROS2_PUB_APP_DATA_STRLEN = 8'd22;
    localparam [`ROS2_APP_DATA_LEN_WIDTH-1:0] ROS2_PUB_APP_DATA_LEN = ROS2_PUB_APP_DATA_STRLEN + 8'd4;
    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data = {msg_number, " - AGPF morF egasseM", 24'b0, ROS2_PUB_APP_DATA_STRLEN}; // Published message
`ifdef ROS2_PUB_DATA_RAM
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_addr;
    wire ros2_pub_app_data_ce;
    reg  [31:0] ros2_pub_app_data_rdata;
    always @(posedge clk_int) begin
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
`endif

    // --- ROS2 Publisher Message Control
    reg ros2_pub_app_data_req_0;
    reg ros2_pub_app_data_rel_0;
    wire ros2_pub_app_data_ack_0;
    wire ros2_pub_app_data_nack_0;
    reg [27:0] msg_change_counter;
    always @(posedge clk_int or negedge rst_n_int) begin
        if (!rst_n_int) begin
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
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name = "aaa/tr";
    wire [7:0] ros2_sub_topic_name_len = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_sub_topic_type_name_len = 8'd29;

    // --- ROS2 Subscriber Received Message
    wire [63:0] recvinfo_din;
    wire recvinfo_write;
    reg  [`ROS2_APP_DATA_LEN_WIDTH-1:0] ros2_sub_app_data_len;
    reg  [15:0] ros2_sub_app_data_rep_id;

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_addr;
    wire ros2_sub_app_data_ce;
    wire ros2_sub_app_data_we;
    wire [7:0] ros2_sub_app_data_wdata;
    reg [7:0] rx_msg_reg[0:`ROS2_MAX_APP_DATA_LEN-1];

    always @(posedge clk_int or negedge rst_n_int) begin
        if (!rst_n_int) begin
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

    assign led4 = rx_msg_reg[0][0];
    assign led5 = rx_msg_reg[0][1];
    assign led6 = rx_msg_reg[0][2];
    assign led7 = rx_msg_reg[0][3];

    // --- SEDP Reader Table Memory
`ifdef ROS2_SEDP_READER_TBL_RAM
    wire [$clog2(`ROS2_SEDP_READER_MAX*11)-1:0] sedp_reader_tbl_mem_addr;
    wire sedp_reader_tbl_mem_cs;
    wire sedp_reader_tbl_mem_we;
    wire [63:0] sedp_reader_tbl_mem_wdata;
    wire [63:0] sedp_reader_tbl_mem_rdata;
    ram_1rw #(
        .DEPTH(`ROS2_SEDP_READER_MAX*11),
        .DWIDTH(64)
    )
    sedp_reader_tbl_mem (
        .i_clk(clk_int),
        .i_rst_n(rst_n_int),
        .i_cs_n(~sedp_reader_tbl_mem_cs),
        .i_we_n(~sedp_reader_tbl_mem_we),
        .i_wmask(8'b11111111),
        .i_addr(sedp_reader_tbl_mem_addr),
        .i_wdata(sedp_reader_tbl_mem_wdata),
        .o_rdata(sedp_reader_tbl_mem_rdata)
    );
`endif

    // --- IP Payload Memory
    wire payloadsmem_cs;
    wire payloadsmem_we;
    wire [`PAYLOADSMEM_AWIDTH-1:0] payloadsmem_addr;
    wire [7:0] payloadsmem_wdata, payloadsmem_rdata;
    ram_1rw #(
        .DEPTH(`PAYLOADSMEM_DEPTH),
        .DWIDTH(8)
    )
    payloadsmem (
        .i_clk(clk_int),
        .i_rst_n(rst_n_int),
        .i_cs_n(~payloadsmem_cs),
        .i_we_n(~payloadsmem_we),
        .i_wmask(4'b1111),
        .i_addr(payloadsmem_addr),
        .i_wdata(payloadsmem_wdata),
        .o_rdata(payloadsmem_rdata)
    );

    // --- ROS2rapper with Ethernet
    localparam PRESCALER_DIV = 64;
    ros2_ether #(
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
    ros2 (
        .clk(clk_int),
        .rst_n(rst_n_int),

        .ether_en(1'b1),
        .ros2pub_en(1),
        .ros2sub_en(1),

        .phy_rx_clk(phy_rx_clk),
        .phy_rxd(phy_rxd),
        .phy_rx_dv(phy_rx_dv),
        .phy_rx_er(phy_rx_er),
        .phy_tx_clk(phy_tx_clk),
        .phy_txd(phy_txd),
        .phy_tx_en(phy_tx_en),
        .phy_rst_n(phy_rst_n),

        .mac_addr(mac_addr),
        .ip_addr(ip_addr),
        .gateway_ip_addr(gateway_ip_addr),
        .subnet_mask(subnet_mask),

        .ros2_node_name(ros2_node_name),
        .ros2_node_name_len(ros2_node_name_len),
        .ros2_node_udp_port(ros2_node_udp_port),
        .ros2_port_num_seed(ros2_port_num_seed),
        .ros2_fragment_expiration(ros2_fragment_expiration),
        .ros2_guid_prefix(ros2_guid_prefix),
        .ros2_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
        .ros2_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),

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

`ifdef ROS2_PUB_DATA_FF
        .ros2_pub_app_data_0(ros2_pub_app_data),
        .ros2_pub_app_data_1(0),
        .ros2_pub_app_data_2(0),
        .ros2_pub_app_data_3(0),
`endif
`ifdef ROS2_PUB_DATA_RAM
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
`endif

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

`ifdef ROS2_SEDP_READER_TBL_RAM
        .sedp_reader_tbl_mem_addr(sedp_reader_tbl_mem_addr),
        .sedp_reader_tbl_mem_ce(sedp_reader_tbl_mem_cs),
        .sedp_reader_tbl_mem_we(sedp_reader_tbl_mem_we),
        .sedp_reader_tbl_mem_wdata(sedp_reader_tbl_mem_wdata),
        .sedp_reader_tbl_mem_rdata(sedp_reader_tbl_mem_rdata),
`endif

        .ip_payloadsmem_addr(payloadsmem_addr),
        .ip_payloadsmem_ce(payloadsmem_cs),
        .ip_payloadsmem_we(payloadsmem_we),
        .ip_payloadsmem_wdata(payloadsmem_wdata),
        .ip_payloadsmem_rdata(payloadsmem_rdata),

        .arp_req_retry_count(ARP_REQUEST_RETRY_COUNT),
        .arp_req_retry_interval(ARP_REQUEST_RETRY_INTERVAL),
        .arp_req_timeout(ARP_REQUEST_TIMEOUT)
    );

endmodule

`resetall
