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

    input  wire       sw0,
    input  wire       sw1,
    input  wire       sw2,
    input  wire       sw3,

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
    wire [3:0] ros2pub_en = {sw3, sw2, sw1, sw0};

    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_0 = "bbb/tr";
    wire [7:0] ros2_pub_topic_name_len_0 = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_0 = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len_0 = 8'd29;

    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_1 = "ccc/tr";
    wire [7:0] ros2_pub_topic_name_len_1 = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_1 = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len_1 = 8'd29;

    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_2 = "ddd/tr";
    wire [7:0] ros2_pub_topic_name_len_2 = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_2 = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len_2 = 8'd29;

    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_3 = "eee/tr";
    wire [7:0] ros2_pub_topic_name_len_3 = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_3 = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len_3 = 8'd29;

    localparam [7:0] ROS2_PUB_APP_DATA_STRLEN = 8'd23;
    localparam [7:0] ROS2_PUB_APP_DATA_LEN = ROS2_PUB_APP_DATA_STRLEN + 8'd4;

    // --- ROS2 Publisher Message Control
    reg [29:0] msg_change_counter;
    reg [3:0] prev_msg_change_counter;
    always @(posedge clk_int or negedge rst_n_int) begin
        if (!rst_n_int) begin
            msg_change_counter <= 30'd0;
            prev_msg_change_counter <= 4'd0;
        end else begin
            msg_change_counter <= msg_change_counter + 1'b1;
            prev_msg_change_counter[3:0] <= msg_change_counter[29:26];
        end
    end

    // Rising edge of msg_change_counter[29:26]
    wire [3:0] change_msg = (~prev_msg_change_counter) & msg_change_counter[29:26];

    wire [3:0] ros2_pub_app_data_req;
    wire [3:0] ros2_pub_app_data_rel;
    wire [3:0] ros2_pub_app_data_grant;

    // Published messages
    wire [7:0] msg_number[0:3];
    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_0 = {msg_number[0], "B - AGPF morF egasseM", 24'b0, ROS2_PUB_APP_DATA_STRLEN};
    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_1 = {msg_number[1], "C - AGPF morF egasseM", 24'b0, ROS2_PUB_APP_DATA_STRLEN};
    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_2 = {msg_number[2], "D - AGPF morF egasseM", 24'b0, ROS2_PUB_APP_DATA_STRLEN};
    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_3 = {msg_number[3], "E - AGPF morF egasseM", 24'b0, ROS2_PUB_APP_DATA_STRLEN};

    generate
        genvar iter;
        for (iter = 0; iter < 4; iter = iter + 1) begin : PUB_MSG_CTRL
            pub_msg_ctrl pub_msg_ctrl_inst(
                .clk(clk_int),
                .rst_n(rst_n_int),
                .msg_number(msg_number[iter]),
                .change_msg(change_msg[iter]),
                .ros2_pub_app_data_req(ros2_pub_app_data_req[iter]),
                .ros2_pub_app_data_rel(ros2_pub_app_data_rel[iter]),
                .ros2_pub_app_data_grant(ros2_pub_app_data_grant[iter])
            );
        end
    endgenerate

`ifdef ROS2_PUB_DATA_RAM
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_0_addr;
    wire ros2_pub_app_data_0_ce;
    reg  [31:0] ros2_pub_app_data_0_rdata;

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_1_addr;
    wire ros2_pub_app_data_1_ce;
    reg  [31:0] ros2_pub_app_data_1_rdata;

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_2_addr;
    wire ros2_pub_app_data_2_ce;
    reg  [31:0] ros2_pub_app_data_2_rdata;

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_3_addr;
    wire ros2_pub_app_data_3_ce;
    reg  [31:0] ros2_pub_app_data_3_rdata;

    integer i;
    always @(posedge clk_int) begin
        for (i = 0; i < 32; i = i + 1) begin
            if (ros2_pub_app_data_0_ce)
                ros2_pub_app_data_0_rdata[i] <= ros2_pub_app_data_0[32*ros2_pub_app_data_0_addr + i];
            if (ros2_pub_app_data_1_ce)
                ros2_pub_app_data_1_rdata[i] <= ros2_pub_app_data_1[32*ros2_pub_app_data_1_addr + i];
            if (ros2_pub_app_data_2_ce)
                ros2_pub_app_data_2_rdata[i] <= ros2_pub_app_data_2[32*ros2_pub_app_data_2_addr + i];
            if (ros2_pub_app_data_3_ce)
                ros2_pub_app_data_3_rdata[i] <= ros2_pub_app_data_3[32*ros2_pub_app_data_3_addr + i];
        end
    end
`endif

    // --- ROS2 Subscriber Configuration
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name = "aaa/tr";
    wire [7:0] ros2_sub_topic_name_len = 8'd7;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_sub_topic_type_name_len = 8'd29;

    // --- ROS2 Subscriber Received Message
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_addr;
    wire ros2_sub_app_data_ce;
    wire ros2_sub_app_data_we;
    wire [7:0] ros2_sub_app_data_wdata;
    reg [7:0] rx_msg_reg[0:`ROS2_MAX_APP_DATA_LEN-1];
    always @(posedge clk_int) begin
        if (ros2_sub_app_data_ce & ros2_sub_app_data_we)
            rx_msg_reg[ros2_sub_app_data_addr][7:0] <= ros2_sub_app_data_wdata;
    end
    wire [7:0] ros2_sub_app_data_len;
    wire [15:0] ros2_sub_app_data_rep_id;
    assign led4 = rx_msg_reg[0][0];
    assign led5 = rx_msg_reg[0][1];
    assign led6 = rx_msg_reg[0][2];
    assign led7 = rx_msg_reg[0][3];

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
        .PRESCALER_DIV              (PRESCALER_DIV),
        .TX_INTERVAL_COUNT          ((`ROS2CLK_HZ / PRESCALER_DIV) / 100),
        .TX_PERIOD_SPDP_WR_COUNT    ((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_WR_COUNT((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_WR_COUNT((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_HB_COUNT((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_HB_COUNT((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_AN_COUNT((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_AN_COUNT((`ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_APP_WR_COUNT     ((`ROS2CLK_HZ / PRESCALER_DIV) * 3)
    )
    ros2 (
        .clk(clk_int),
        .rst_n(rst_n_int),

        .ether_en(1'b1),
        .ros2pub_en(ros2pub_en),
        .ros2sub_en(4'b0001),

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
        .ros2_rx_udp_port(0),
        .ros2_port_num_seed(ros2_port_num_seed),
        .ros2_fragment_expiration(ros2_fragment_expiration),
        .ros2_guid_prefix(ros2_guid_prefix),
        .ros2_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
        .ros2_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),

        .ros2_pub_topic_name_0(ros2_pub_topic_name_0),
        .ros2_pub_topic_name_len_0(ros2_pub_topic_name_len_0),
        .ros2_pub_topic_type_name_0(ros2_pub_topic_type_name_0),
        .ros2_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len_0),

        .ros2_pub_topic_name_1(ros2_pub_topic_name_1),
        .ros2_pub_topic_name_len_1(ros2_pub_topic_name_len_1),
        .ros2_pub_topic_type_name_1(ros2_pub_topic_type_name_1),
        .ros2_pub_topic_type_name_len_1(ros2_pub_topic_type_name_len_1),

        .ros2_pub_topic_name_2(ros2_pub_topic_name_2),
        .ros2_pub_topic_name_len_2(ros2_pub_topic_name_len_2),
        .ros2_pub_topic_type_name_2(ros2_pub_topic_type_name_2),
        .ros2_pub_topic_type_name_len_2(ros2_pub_topic_type_name_len_2),

        .ros2_pub_topic_name_3(ros2_pub_topic_name_3),
        .ros2_pub_topic_name_len_3(ros2_pub_topic_name_len_3),
        .ros2_pub_topic_type_name_3(ros2_pub_topic_type_name_3),
        .ros2_pub_topic_type_name_len_3(ros2_pub_topic_type_name_len_3),

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
        .ros2_pub_app_data_0(ros2_pub_app_data_0),
        .ros2_pub_app_data_1(ros2_pub_app_data_1),
        .ros2_pub_app_data_2(ros2_pub_app_data_2),
        .ros2_pub_app_data_3(ros2_pub_app_data_3),
`endif
`ifdef ROS2_PUB_DATA_RAM
        .ros2_pub_app_data_0_addr(ros2_pub_app_data_0_addr),
        .ros2_pub_app_data_0_ce(ros2_pub_app_data_0_ce),
        .ros2_pub_app_data_0_rdata(ros2_pub_app_data_0_rdata),

        .ros2_pub_app_data_1_addr(ros2_pub_app_data_1_addr),
        .ros2_pub_app_data_1_ce(ros2_pub_app_data_1_ce),
        .ros2_pub_app_data_1_rdata(ros2_pub_app_data_1_rdata),

        .ros2_pub_app_data_2_addr(ros2_pub_app_data_2_addr),
        .ros2_pub_app_data_2_ce(ros2_pub_app_data_2_ce),
        .ros2_pub_app_data_2_rdata(ros2_pub_app_data_2_rdata),

        .ros2_pub_app_data_3_addr(ros2_pub_app_data_3_addr),
        .ros2_pub_app_data_3_ce(ros2_pub_app_data_3_ce),
        .ros2_pub_app_data_3_rdata(ros2_pub_app_data_3_rdata),
`endif

        .ros2_pub_app_data_len_0(ROS2_PUB_APP_DATA_LEN),
        .ros2_pub_app_data_len_1(ROS2_PUB_APP_DATA_LEN),
        .ros2_pub_app_data_len_2(ROS2_PUB_APP_DATA_LEN),
        .ros2_pub_app_data_len_3(ROS2_PUB_APP_DATA_LEN),

        .ros2_pub_app_data_req(ros2_pub_app_data_req),
        .ros2_pub_app_data_rel(ros2_pub_app_data_rel),
        .ros2_pub_app_data_grant(ros2_pub_app_data_grant),

        .ros2_sub_app_data_addr(ros2_sub_app_data_addr),
        .ros2_sub_app_data_ce(ros2_sub_app_data_ce),
        .ros2_sub_app_data_we(ros2_sub_app_data_we),
        .ros2_sub_app_data_wdata(ros2_sub_app_data_wdata),
        .ros2_sub_app_data_len(ros2_sub_app_data_len),
        .ros2_sub_app_data_rep_id(ros2_sub_app_data_rep_id),
        .ros2_sub_app_data_req(1'b0),
        .ros2_sub_app_data_rel(1'b0),
        .ros2_sub_app_data_grant(),
        .ros2_sub_app_data_recv(),

        .udp_rxbuf_rel(1'b1),
        .udp_rxbuf_grant(),
        .udp_rxbuf_addr(),
        .udp_rxbuf_ce(),
        .udp_rxbuf_we(),
        .udp_rxbuf_wdata(),

        .udp_txbuf_rel(1'b0),
        .udp_txbuf_grant(),
        .udp_txbuf_addr(),
        .udp_txbuf_ce(),
        .udp_txbuf_rdata(32'b0),

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


module pub_msg_ctrl(
    input wire clk,
    input wire rst_n,

    output reg [7:0] msg_number,
    input wire change_msg,

    output reg ros2_pub_app_data_req,
    output reg ros2_pub_app_data_rel,
    input wire ros2_pub_app_data_grant
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msg_number <= 8'd48;  // '0'
            ros2_pub_app_data_req <= 1'b0;
            ros2_pub_app_data_rel <= 1'b0;
        end else begin
            ros2_pub_app_data_rel <= 1'b0;
            if (change_msg) begin
                ros2_pub_app_data_req <= 1'b1;
            end else if (ros2_pub_app_data_req && ros2_pub_app_data_grant) begin
                msg_number <= (msg_number == 8'd57) ? 8'd48 : (msg_number + 1'b1);
                ros2_pub_app_data_req <= 1'b0;
                ros2_pub_app_data_rel <= 1'b1;
            end
        end
    end

endmodule

`resetall
