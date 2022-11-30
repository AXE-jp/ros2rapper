`resetall
`timescale 1ns / 1ps
`default_nettype none

`include "ether_config.vh"

module top (
    input  wire       clk,
    input  wire       reset_n,

    output wire       phy_ref_clk,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    input  wire       phy_col,
    input  wire       phy_crs,
    output wire       phy_reset_n
);

wire [47:0] mac_addr         = 48'h02_00_00_00_00_00;
wire [31:0] ip_addr          = {8'd192, 8'd168, 8'd1,   8'd100};
wire [31:0] gateway_ip_addr  = {8'd192, 8'd168, 8'd1,   8'd1};
wire [31:0] subnet_mask      = {8'd255, 8'd255, 8'd255, 8'd0};
wire [255:0] ros2_node_name = "reklat";
wire [7:0] ros2_node_name_len = 8'd7;
wire [15:0] ros2_node_udp_port = 16'd52000;
wire [15:0] ros2_port_num_seed = 16'd7400;
wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;
wire [255:0] ros2_topic_name = "rettahc/tr";
wire [7:0] ros2_topic_name_len = 8'd11;
wire [511:0] ros2_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
wire [7:0] ros2_topic_type_name_len = 8'd29;
wire [511:0] ros2_app_data = "!retsiger gifnoc morf dlrow ,olleh";
wire [7:0] ros2_app_data_len = 8'd35;

ros2_ether ros2 (
    .clk(clk),
    .reset_n(reset_n),
    .phy_ref_clk(phy_ref_clk),
    .phy_rx_clk(phy_rx_clk),
    .phy_rxd(phy_rxd),
    .phy_rx_dv(phy_rx_dv),
    .phy_rx_er(phy_rx_er),
    .phy_tx_clk(phy_tx_clk),
    .phy_txd(phy_txd),
    .phy_tx_en(phy_tx_en),
    .phy_col(phy_col),
    .phy_crs(phy_crs),
    .phy_reset_n(phy_reset_n),
    .mac_addr(mac_addr),
    .ip_addr(ip_addr),
    .gateway_ip_addr(gateway_ip_addr),
    .subnet_mask(subnet_mask),
    .ros2_node_name(ros2_node_name),
    .ros2_node_name_len(ros2_node_name_len),
    .ros2_node_udp_port(ros2_node_udp_port),
    .ros2_port_num_seed(ros2_port_num_seed),
    .ros2_guid_prefix(ros2_guid_prefix),
    .ros2_topic_name(ros2_topic_name),
    .ros2_topic_name_len(ros2_topic_name_len),
    .ros2_topic_type_name(ros2_topic_type_name),
    .ros2_topic_type_name_len(ros2_topic_type_name_len),
    .ros2_app_data(ros2_app_data),
    .ros2_app_data_len(ros2_app_data_len)
);

endmodule

module ros2_ether (
    input  wire       clk,
    input  wire       reset_n,

    output wire       phy_ref_clk,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    input  wire       phy_col,
    input  wire       phy_crs,
    output wire       phy_reset_n,

    input wire [47:0] mac_addr,
    input wire [31:0] ip_addr,
    input wire [31:0] gateway_ip_addr,
    input wire [31:0] subnet_mask,

    (* mark_debug = "true" *) input wire [255:0] ros2_node_name,
    input wire [7:0] ros2_node_name_len,
    input wire [15:0] ros2_node_udp_port,
    input wire [15:0] ros2_port_num_seed,
    input wire [95:0] ros2_guid_prefix,
    input wire [255:0] ros2_topic_name,
    input wire [7:0] ros2_topic_name_len,
    input wire [511:0] ros2_topic_type_name,
    input wire [7:0] ros2_topic_type_name_len,
    input wire [511:0] ros2_app_data,
    input wire [7:0] ros2_app_data_len
);

wire clk_ibufg;

wire clk_mmcm_out;
wire clk_int;
wire rst_int;

wire mmcm_rst = ~reset_n;
wire mmcm_locked;
wire mmcm_clkfb;

`ifdef TARGET_ASIC
assign clk_ibufg = clk;
`elsif TARGET_XILINX
IBUFG
clk_ibufg_inst(
    .I(clk),
    .O(clk_ibufg)
);
`endif

wire clk_25mhz_mmcm_out;
wire clk_25mhz_int;

`ifdef TARGET_ASIC
assign clk_mmcm_out = clk_ibufg;
assign clk_25mhz_mmcm_out = clk_ibufg;
assign mmcm_locked = ~mmcm_rst;
`elsif TARGET_XILINX
MMCME2_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKOUT0_DIVIDE_F(10),
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
    .CLKIN1(clk_ibufg),
    .CLKFBIN(mmcm_clkfb),
    .RST(mmcm_rst),
    .PWRDWN(1'b0),
    .CLKOUT0(clk_mmcm_out),
    .CLKOUT0B(),
    .CLKOUT1(clk_25mhz_mmcm_out),
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
`endif

`ifdef TARGET_ASIC
assign clk_int = clk_mmcm_out;
`elsif TARGET_XILINX
BUFG
clk_bufg_inst (
    .I(clk_mmcm_out),
    .O(clk_int)
);
`endif

`ifdef TARGET_ASIC
assign clk_25mhz_int = clk_25mhz_mmcm_out;
`elsif TARGET_XILINX
BUFG
clk_25mhz_bufg_inst (
    .I(clk_25mhz_mmcm_out),
    .O(clk_25mhz_int)
);
`endif

sync_reset #(
    .N(4)
)
sync_reset_inst (
    .clk(clk_int),
    .rst(~mmcm_locked),
    .out(rst_int)
);

assign phy_ref_clk = clk_25mhz_int;

wire tx_ip_hdr_valid;
wire tx_ip_hdr_ready;
wire [5:0] tx_ip_dscp;
wire [1:0] tx_ip_ecn;
wire [15:0] tx_ip_length;
wire [7:0] tx_ip_ttl;
wire [7:0] tx_ip_protocol;
wire [31:0] tx_ip_source_ip;
wire [31:0] tx_ip_dest_ip;
wire [7:0] tx_ip_payload_axis_tdata;
wire tx_ip_payload_axis_tvalid;
wire tx_ip_payload_axis_tready;
wire tx_ip_payload_axis_tlast;

wire rx_ip_hdr_valid;
wire rx_ip_hdr_ready;
wire [3:0] rx_ip_version;
wire [3:0] rx_ip_ihl;
wire [5:0] rx_ip_dscp;
wire [1:0] rx_ip_ecn;
wire [15:0] rx_ip_length;
wire [15:0] rx_ip_identification;
wire [2:0] rx_ip_flags;
wire [12:0] rx_ip_fragment_offset;
wire [7:0] rx_ip_ttl;
wire [7:0] rx_ip_protocol;
wire [15:0] rx_ip_header_checksum;
wire [31:0] rx_ip_source_ip;
wire [31:0] rx_ip_dest_ip;
wire [7:0] rx_ip_payload_axis_tdata;
wire rx_ip_payload_axis_tvalid;
wire rx_ip_payload_axis_tready;
wire rx_ip_payload_axis_tlast;

verilog_ethernet #(
`ifdef TARGET_ASIC
    .TARGET("GENERIC")
`elsif TARGET_XILINX
    .TARGET("XILINX")
`endif
)
verilog_ethernet_inst (
    .clk(clk_int),
    .rst(rst_int),

    .phy_rx_clk(phy_rx_clk),
    .phy_rxd(phy_rxd),
    .phy_rx_dv(phy_rx_dv),
    .phy_rx_er(phy_rx_er),
    .phy_tx_clk(phy_tx_clk),
    .phy_txd(phy_txd),
    .phy_tx_en(phy_tx_en),
    .phy_col(phy_col),
    .phy_crs(phy_crs),
    .phy_reset_n(phy_reset_n),

    .tx_ip_hdr_valid(tx_ip_hdr_valid),
    .tx_ip_hdr_ready(tx_ip_hdr_ready),
    .tx_ip_dscp(tx_ip_dscp),
    .tx_ip_ecn(tx_ip_ecn),
    .tx_ip_length(tx_ip_length),
    .tx_ip_ttl(tx_ip_ttl),
    .tx_ip_protocol(tx_ip_protocol),
    .tx_ip_source_ip(tx_ip_source_ip),
    .tx_ip_dest_ip(tx_ip_dest_ip),
    .tx_ip_payload_axis_tdata(tx_ip_payload_axis_tdata),
    .tx_ip_payload_axis_tvalid(tx_ip_payload_axis_tvalid),
    .tx_ip_payload_axis_tready(tx_ip_payload_axis_tready),
    .tx_ip_payload_axis_tlast(tx_ip_payload_axis_tlast),
    .tx_ip_payload_axis_tuser(1'b0),

    .rx_ip_hdr_valid(rx_ip_hdr_valid),
    .rx_ip_hdr_ready(rx_ip_hdr_ready),
    .rx_ip_version(rx_ip_version),
    .rx_ip_ihl(rx_ip_ihl),
    .rx_ip_dscp(rx_ip_dscp),
    .rx_ip_ecn(rx_ip_ecn),
    .rx_ip_length(rx_ip_length),
    .rx_ip_identification(rx_ip_identification),
    .rx_ip_flags(rx_ip_flags),
    .rx_ip_fragment_offset(rx_ip_fragment_offset),
    .rx_ip_ttl(rx_ip_ttl),
    .rx_ip_protocol(rx_ip_protocol),
    .rx_ip_header_checksum(rx_ip_header_checksum),
    .rx_ip_source_ip(rx_ip_source_ip),
    .rx_ip_dest_ip(rx_ip_dest_ip),
    .rx_ip_payload_axis_tdata(rx_ip_payload_axis_tdata),
    .rx_ip_payload_axis_tvalid(rx_ip_payload_axis_tvalid),
    .rx_ip_payload_axis_tready(rx_ip_payload_axis_tready),
    .rx_ip_payload_axis_tlast(rx_ip_payload_axis_tlast),
    .rx_ip_payload_axis_tuser(),

    .local_mac(mac_addr),
    .local_ip(ip_addr),
    .gateway_ip(gateway_ip_addr),
    .subnet_mask(subnet_mask)
);

wire tx_fifo_wr_en;
wire [7:0] tx_fifo_din;
wire tx_fifo_full;
wire tx_fifo_rd_en;
wire [7:0] tx_fifo_dout;
wire tx_fifo_empty;

siso #(
    .DATA_WIDTH(8),
    .DEPTH(`EXT_TX_FIFO_DEPTH)
)
tx_fifo (
    .clk(clk_int),
    .rst(rst_int),
    .wr_en(tx_fifo_wr_en),
    .din(tx_fifo_din),
    .full(tx_fifo_full),
    .rd_en(tx_fifo_rd_en),
    .dout(tx_fifo_dout),
    .empty(tx_fifo_empty)
);

wire rx_fifo_wr_en;
wire [7:0] rx_fifo_din;
wire rx_fifo_full;
wire rx_fifo_rd_en;
wire [7:0] rx_fifo_dout;
wire rx_fifo_empty;

siso #(
    .DATA_WIDTH(8),
    .DEPTH(`EXT_RX_FIFO_DEPTH)
)
rx_fifo (
    .clk(clk_int),
    .rst(rst_int),
    .wr_en(rx_fifo_wr_en),
    .din(rx_fifo_din),
    .full(rx_fifo_full),
    .rd_en(rx_fifo_rd_en),
    .dout(rx_fifo_dout),
    .empty(rx_fifo_empty)
);

ros2
ros2_i (
    .ap_clk(clk_int),
    .ap_rst(rst_int),
    .in_V_dout(rx_fifo_dout),
    .in_V_empty_n(~rx_fifo_empty),
    .in_V_read(rx_fifo_rd_en),
    .out_V_din(tx_fifo_din),
    .out_V_full_n(~tx_fifo_full),
    .out_V_write(tx_fifo_wr_en),
    .conf_ip_addr({ip_addr[7:0],ip_addr[15:8],ip_addr[23:16],ip_addr[31:24]}),
    .conf_node_name(ros2_node_name),
    .conf_node_name_len(ros2_node_name_len),
    .conf_node_udp_port(ros2_node_udp_port),
    .conf_port_num_seed(ros2_port_num_seed),
    .conf_guid_prefix(ros2_guid_prefix),
    .conf_topic_name(ros2_topic_name),
    .conf_topic_name_len(ros2_topic_name_len),
    .conf_topic_type_name(ros2_topic_type_name),
    .conf_topic_type_name_len(ros2_topic_type_name_len),
    .conf_app_data(ros2_app_data),
    .conf_app_data_len(ros2_app_data_len)
);

ip_tx
ip_tx_i (
    .ap_clk(clk_int),
    .ap_rst_n(~rst_int),
    .din_V_dout(tx_fifo_dout),
    .din_V_empty_n(~tx_fifo_empty),
    .din_V_read(tx_fifo_rd_en),
    .tx_hdr_valid(tx_ip_hdr_valid),
    .tx_hdr_ready(tx_ip_hdr_ready),
    .tx_hdr({
        tx_ip_dest_ip,
        tx_ip_source_ip,
        tx_ip_protocol,
        tx_ip_ttl,
        tx_ip_length,
        tx_ip_ecn,
        tx_ip_dscp
    }),
    .tx_payload_TVALID(tx_ip_payload_axis_tvalid),
    .tx_payload_TREADY(tx_ip_payload_axis_tready),
    .tx_payload_TDATA(tx_ip_payload_axis_tdata),
    .tx_payload_TLAST(tx_ip_payload_axis_tlast),
    .tx_payload_TKEEP(),
    .tx_payload_TSTRB()
);

ip_rx
ip_rx_i (
    .ap_clk(clk_int),
    .ap_rst_n(~rst_int),
    .dout_V_din(rx_fifo_din),
    .dout_V_full_n(~rx_fifo_full),
    .dout_V_write(rx_fifo_wr_en),
    .rx_hdr_valid(rx_ip_hdr_valid),
    .rx_hdr_ready(rx_ip_hdr_ready),
    .rx_hdr({
        rx_ip_dest_ip,
        rx_ip_source_ip,
        rx_ip_header_checksum,
        rx_ip_protocol,
        rx_ip_ttl,
        rx_ip_fragment_offset,
        rx_ip_flags,
        rx_ip_identification,
        rx_ip_length,
        rx_ip_ecn,
        rx_ip_dscp,
        rx_ip_ihl,
        rx_ip_version
    }),
    .rx_payload_TVALID(rx_ip_payload_axis_tvalid),
    .rx_payload_TREADY(rx_ip_payload_axis_tready),
    .rx_payload_TDATA(rx_ip_payload_axis_tdata),
    .rx_payload_TLAST(rx_ip_payload_axis_tlast),
    .rx_payload_TKEEP(),
    .rx_payload_TSTRB()
);

endmodule

`resetall
