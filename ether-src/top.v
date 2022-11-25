`resetall
`timescale 1ns / 1ps
`default_nettype none

`include "ether_config.vh"

// `define NO_IP

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

wire clk_ibufg;

wire clk_mmcm_out;
wire clk_int;
wire rst_int;

wire mmcm_rst = ~reset_n;
wire mmcm_locked;
wire mmcm_clkfb;

`ifdef NO_IP
assign clk_ibufg = clk;
`else
IBUFG
clk_ibufg_inst(
    .I(clk),
    .O(clk_ibufg)
);
`endif

wire clk_25mhz_mmcm_out;
wire clk_25mhz_int;

`ifdef NO_IP
assign clk_mmcm_out = clk_ibufg;
assign clk_25mhz_mmcm_out = clk_ibufg;
assign mmcm_locked = ~mmcm_rst;
`else
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

`ifdef NO_IP
assign clk_int = clk_mmcm_out;
`else
BUFG
clk_bufg_inst (
    .I(clk_mmcm_out),
    .O(clk_int)
);
`endif

`ifdef NO_IP
assign clk_25mhz_int = clk_25mhz_mmcm_out;
`else
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

wire [47:0] local_mac   = 48'h02_00_00_00_00_00;
wire [31:0] local_ip    = {8'd192, 8'd168, 8'd1,   8'd128};
wire [31:0] gateway_ip  = {8'd192, 8'd168, 8'd1,   8'd1};
wire [31:0] subnet_mask = {8'd255, 8'd255, 8'd255, 8'd0};

verilog_ethernet #(
`ifdef NO_IP
    .TARGET("GENERIC")
`else
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

    .local_mac(local_mac),
    .local_ip(local_ip),
    .gateway_ip(gateway_ip),
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

// echo_0
// echo_i (
//     .ap_clk(clk_int),
//     .ap_rst(rst_int),
//     .din_V_dout(rx_fifo_dout),
//     .din_V_empty_n(~rx_fifo_empty),
//     .din_V_read(rx_fifo_rd_en),
//     .dout_V_din(tx_fifo_din),
//     .dout_V_full_n(~tx_fifo_full),
//     .dout_V_write(tx_fifo_wr_en)
// );

ros2_0
ros2_i (
    .ap_clk(clk_int),
    .ap_rst(rst_int),
    .in_V_dout(rx_fifo_dout),
    .in_V_empty_n(~rx_fifo_empty),
    .in_V_read(rx_fifo_rd_en),
    .out_V_din(tx_fifo_din),
    .out_V_full_n(~tx_fifo_full),
    .out_V_write(tx_fifo_wr_en)
);

ip_tx_0
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
    .tx_payload_TKEEP(1'b1),
    .tx_payload_TSTRB(1'b1)
);

ip_rx_0
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
