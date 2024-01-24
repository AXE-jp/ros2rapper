`resetall
`default_nettype none

`include "config.vh"

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

    output wire       led_r,
    output wire       led_g,
    output wire       led_b,
    output wire       led4,
    output wire       led5,
    output wire       led6,
    output wire       led7,
    input wire        sw0,
    input wire        sw1,
    input wire        sw2,
    input wire        sw3
);

localparam ARP_REQUEST_RETRY_COUNT = 4;
localparam ARP_REQUEST_RETRY_INTERVAL = (125000000*2);
localparam ARP_REQUEST_TIMEOUT = (125000000*30);

wire clk_int;
wire clk_25mhz_int;
wire rst_n_int;
wire mmcm_locked;

wire mmcm_clkfb;

MMCME2_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKOUT0_DIVIDE_F(13),
    .CLKOUT0_DUTY_CYCLE(0.5),
    .CLKOUT0_PHASE(0),
    .CLKOUT1_DIVIDE(39),
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
    .CLKFBOUT_MULT_F(9.75),
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

assign phy_ref_clk = clk_25mhz_int;

wire [47:0] mac_addr         = 48'h00_00_00_00_00_02;
wire [31:0] ip_addr          = {8'd100, 8'd1, 8'd168, 8'd192};
wire [31:0] gateway_ip_addr  = {8'd1, 8'd1, 8'd168, 8'd192};
wire [31:0] subnet_mask      = {8'd0, 8'd255, 8'd255, 8'd255};


//`define	USE_CTRL_DEMO

`ifdef USE_CTRL_DEMO

wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name;
wire [7:0] ros2_node_name_len;
wire [15:0] ros2_node_udp_port;
wire [15:0] ros2_cpu_udp_port;
wire [15:0] ros2_port_num_seed;
wire [31:0] ros2_tx_period;
wire [31:0] ros2_fragment_expiration;
wire [95:0] ros2_guid_prefix;

wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name;
wire [7:0] ros2_pub_topic_name_len;
wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name;
wire [7:0] ros2_pub_topic_type_name_len;

wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name;
wire [7:0] ros2_sub_topic_name_len;
wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name;
wire [7:0] ros2_sub_topic_type_name_len;

(*mark_debug="true"*) wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_data;
(*mark_debug="true"*) wire [7:0] ros2_app_data_len;

(*mark_debug="true"*) wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_rx_data;
(*mark_debug="true"*) wire [7:0] ros2_app_rx_data_len;
wire [7:0] ros2_app_rx_max_data_len;

(*mark_debug="true"*) wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_data_TDATA;
(*mark_debug="true"*) wire ros2_pub_data_TVALID;
(*mark_debug="true"*) wire ros2_pub_data_TREADY;
(*mark_debug="true"*) wire [7:0] ros2_pub_data_len_int;

ros2_ether_ctrl_demo
    ctrl_mode_inst (
	.ap_clk(clk_int),
	.ap_rst_n(rst_n_int),
	.ros2_conf_node_name(ros2_node_name),
	.ros2_conf_node_name_len(ros2_node_name_len),
	.ros2_conf_node_udp_port(ros2_node_udp_port),
	.ros2_conf_cpu_udp_port(ros2_cpu_udp_port),
	.ros2_conf_port_num_seed(ros2_port_num_seed),
	.ros2_conf_tx_period(ros2_tx_period),
	.ros2_conf_fragment_expiration(ros2_fragment_expiration),
	.ros2_conf_guid_prefix(ros2_guid_prefix),
	.ros2_conf_pub_topic_name(ros2_pub_topic_name),
	.ros2_conf_pub_topic_name_len(ros2_pub_topic_name_len),
	.ros2_conf_pub_topic_type_name(ros2_pub_topic_type_name),
	.ros2_conf_pub_topic_type_name_len(ros2_pub_topic_type_name_len),
	.ros2_conf_sub_topic_name(ros2_sub_topic_name),
	.ros2_conf_sub_topic_name_len(ros2_sub_topic_name_len),
	.ros2_conf_sub_topic_type_name(ros2_sub_topic_type_name),
	.ros2_conf_sub_topic_type_name_len(ros2_sub_topic_type_name_len),
	.ros2_pub_data_V_TDATA(ros2_pub_data_TDATA),
	.ros2_pub_data_V_TVALID(ros2_pub_data_TVALID),
	.ros2_pub_data_V_TREADY(ros2_pub_data_TREADY),
	.ros2_pub_data_len(ros2_pub_data_len_int),
	.ros2_sub_data(ros2_app_rx_data),
	.ros2_sub_data_len(ros2_app_rx_data_len),
	.ros2_sub_data_max_len(ros2_app_rx_max_data_len),
	.sw({sw3,sw2,sw1,sw0}),
	.led({led7,led6,led5,led4})
    );

stream_to_reg # (
	.DATA_BYTES(`ROS2_MAX_APP_DATA_LEN)
	)
    stream2reg_pub_data_inst(
	.clk(clk_int),
	.rst_n(rst_n_int),
	.i_data(ros2_pub_data_TDATA),
	.i_tvalid(ros2_pub_data_TVALID),
	.i_tready(ros2_pub_data_TREADY),
	.mem_len(ros2_pub_data_len_int),
	.o_data(ros2_app_data),
	.o_data_len(ros2_app_data_len)
    );

`else	// USE_CTRL_DEMO

wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name = "reklat";
wire [7:0] ros2_node_name_len = 8'd7;
wire [15:0] ros2_node_udp_port = 16'd52000;
wire [15:0] ros2_cpu_udp_port = 16'd1234;
wire [15:0] ros2_port_num_seed = 16'd7400;
wire [31:0] ros2_tx_period = 32'd12500000;
wire [31:0] ros2_fragment_expiration = 32'd3333333333;
wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;

wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name = "rettahc/tr";
wire [7:0] ros2_pub_topic_name_len = 8'd11;
wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
wire [7:0] ros2_pub_topic_type_name_len = 8'd29;

wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name = "rettahc/tr";
wire [7:0] ros2_sub_topic_name_len = 8'd11;
wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
wire [7:0] ros2_sub_topic_type_name_len = 8'd29;

wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_data = "!retsiger gifnoc morf dlrow ,olleh";
wire [7:0] ros2_app_data_len = 8'd35;

(*mark_debug="true"*) wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_rx_data;
(*mark_debug="true"*) wire [7:0] ros2_app_rx_data_len;
wire [7:0] ros2_app_rx_max_data_len = 8'd`ROS2_MAX_APP_DATA_LEN;

integer led_pos = 0;

always @(posedge clk_int or negedge rst_n_int) begin
    if (!rst_n_int) begin
        led_pos <= 0;
    end else begin
        if (ros2_app_rx_data_len >= 2) begin
            led_pos <= (ros2_app_rx_data_len - 2) * 8;
        end else begin
            led_pos <= 0;
        end
    end
end
assign	led7 = ros2_app_rx_data[led_pos + 3];
assign	led6 = ros2_app_rx_data[led_pos + 2];
assign	led5 = ros2_app_rx_data[led_pos + 1];
assign	led4 = ros2_app_rx_data[led_pos + 0];

`endif	// USE_CTRL_DEMO


wire [`UDP_RXBUF_AWIDTH-1:0] rxbuf_addr;
wire rxbuf_ce;
wire rxbuf_we;
wire [31:0] rxbuf_wdata;

reg rxbuf_cpu_rel;
wire rxbuf_cpu_grant;

reg [15:0] last_rx_size;
assign led_r = (last_rx_size >= 1 && last_rx_size <= 10);
assign led_g = (last_rx_size >= 11 && last_rx_size <= 20);
assign led_b = (last_rx_size > 20);

reg [31:0] txbuf_rdata;
reg txbuf_cpu_rel;
wire [`UDP_TXBUF_AWIDTH-1:0] txbuf_addr;
wire txbuf_cpu_grant;

reg [27:0] tx_cnt;

always @(posedge clk_int or negedge rst_n_int) begin
    if (!rst_n_int) begin
        txbuf_rdata <= 0;
        txbuf_cpu_rel <= 0;
        tx_cnt <= 0;
    end else begin
        if (tx_cnt[27]) begin
            txbuf_cpu_rel <= 1;
            tx_cnt <= 0;
        end else begin
            txbuf_cpu_rel <= 0;
            tx_cnt <= tx_cnt + 1;
        end
        case (txbuf_addr)
        6'h00: txbuf_rdata <= 32'h0a01a8c0;
        6'h01: txbuf_rdata <= 32'h045704d2;
        6'h02: txbuf_rdata <= 32'h00000007;
        6'h03: txbuf_rdata <= 32'h626f6f66;
        6'h04: txbuf_rdata <= 32'h000a7261;
        default: txbuf_rdata <= 32'h0;
        endcase
    end
end

always @(posedge clk_int or negedge rst_n_int) begin
    if (!rst_n_int) begin
        rxbuf_cpu_rel <= 0;
        last_rx_size <= 0;
    end else begin
        if (rxbuf_cpu_grant)
            rxbuf_cpu_rel <= 1;
        else
            rxbuf_cpu_rel <= 0;

        if (rxbuf_addr == 1 && rxbuf_we)
            last_rx_size <= rxbuf_wdata[31:16];
    end
end


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

ros2_ether ros2 (
    .clk(clk_int),
    .rst_n(rst_n_int),
    .ether_en(1'b1),
    .ros2pub_en(1'b1),
    .ros2sub_en(1'b1),
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
    .ros2_cpu_udp_port(ros2_cpu_udp_port),
    .ros2_port_num_seed(ros2_port_num_seed),
    .ros2_tx_period(ros2_tx_period),
    .ros2_fragment_expiration(ros2_fragment_expiration),
    .ros2_guid_prefix(ros2_guid_prefix),
    .ros2_pub_topic_name(ros2_pub_topic_name),
    .ros2_pub_topic_name_len(ros2_pub_topic_name_len),
    .ros2_pub_topic_type_name(ros2_pub_topic_type_name),
    .ros2_pub_topic_type_name_len(ros2_pub_topic_type_name_len),
    .ros2_sub_topic_name(ros2_sub_topic_name),
    .ros2_sub_topic_name_len(ros2_sub_topic_name_len),
    .ros2_sub_topic_type_name(ros2_sub_topic_type_name),
    .ros2_sub_topic_type_name_len(ros2_sub_topic_type_name_len),
    .ros2_app_data(ros2_app_data),
    .ros2_app_data_len(ros2_app_data_len),
    .ros2_app_rx_data(ros2_app_rx_data),
    .ros2_app_rx_data_len(ros2_app_rx_data_len),
    .ros2_app_rx_max_data_len(ros2_app_rx_max_data_len),
    .ros2_app_data_cpu_req(1'b0),
    .ros2_app_data_cpu_rel(1'b0),
    .ros2_app_data_cpu_grant(),
    .udp_rxbuf_cpu_rel(rxbuf_cpu_rel),
    .udp_rxbuf_cpu_grant(rxbuf_cpu_grant),
    .udp_rxbuf_addr(rxbuf_addr),
    .udp_rxbuf_ce(rxbuf_ce),
    .udp_rxbuf_we(rxbuf_we),
    .udp_rxbuf_wdata(rxbuf_wdata),
    .udp_txbuf_cpu_grant(txbuf_cpu_grant),
    .udp_txbuf_cpu_rel(txbuf_cpu_rel),
    .udp_txbuf_addr(txbuf_addr),
    .udp_txbuf_ce(),
    .udp_txbuf_rdata(txbuf_rdata),
    .payloadsmem_addr(payloadsmem_addr),
    .payloadsmem_ce(payloadsmem_cs),
    .payloadsmem_we(payloadsmem_we),
    .payloadsmem_wdata(payloadsmem_wdata),
    .payloadsmem_rdata(payloadsmem_rdata),
    .arp_req_retry_count(ARP_REQUEST_RETRY_COUNT),
    .arp_req_retry_interval(ARP_REQUEST_RETRY_INTERVAL),
    .arp_req_timeout(ARP_REQUEST_TIMEOUT)
);

endmodule

`resetall
