`resetall
`default_nettype none

`include "config.vh"
`include "ether_config.vh"

module ether_top (
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
    input  wire       phy_col,
    input  wire       phy_crs,
    output wire       phy_rst_n,

    output wire       led_r,
    output wire       led_g,
    output wire       led_b
);

wire clk_int;
wire clk_25mhz_int;
wire rst_n_int;
wire mmcm_locked;

`ifdef TARGET_ASIC
// FIXME: use PLL
assign clk_int = clk;
assign clk_25mhz_int = clk;
assign rst_n_int = rst_n;

`elsif TARGET_XILINX
wire clk_ibufg;
wire clk_mmcm_out;
wire mmcm_clkfb;
wire clk_25mhz_mmcm_out;

wire rst_int;

IBUFG clk_ibufg_inst(
    .I(clk),
    .O(clk_ibufg)
);

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
    .CLKIN1(clk_ibufg),
    .CLKFBIN(mmcm_clkfb),
    .RST(~rst_n),
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

BUFG
clk_bufg_inst (
    .I(clk_mmcm_out),
    .O(clk_int)
);

BUFG
clk_25mhz_bufg_inst (
    .I(clk_25mhz_mmcm_out),
    .O(clk_25mhz_int)
);

assign rst_n_int = ~rst_int;

`endif


assign phy_ref_clk = clk_25mhz_int;

wire [47:0] mac_addr         = 48'h00_00_00_00_00_02;
wire [31:0] ip_addr          = {8'd100, 8'd1, 8'd168, 8'd192};
wire [31:0] gateway_ip_addr  = {8'd1, 8'd1, 8'd168, 8'd192};
wire [31:0] subnet_mask      = {8'd0, 8'd255, 8'd255, 8'd255};
wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name = "reklat";
wire [7:0] ros2_node_name_len = 8'd7;
wire [15:0] ros2_node_udp_port = 16'd52000;
wire [15:0] ros2_cpu_udp_port = 16'd1234;
wire [15:0] ros2_port_num_seed = 16'd7400;
wire [31:0] ros2_tx_period = 32'd12500000;
wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;
wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_topic_name = "rettahc/tr";
wire [7:0] ros2_topic_name_len = 8'd11;
wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
wire [7:0] ros2_topic_type_name_len = 8'd29;

wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_data = "!retsiger gifnoc morf dlrow ,olleh";
wire [7:0] ros2_app_data_len = 8'd35;

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
RAM_1RW_WRAP#(`PAYLOADSMEM_AWIDTH, 8) payloadsmem(
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
    .phy_rx_clk(phy_rx_clk),
    .phy_rxd(phy_rxd),
    .phy_rx_dv(phy_rx_dv),
    .phy_rx_er(phy_rx_er),
    .phy_tx_clk(phy_tx_clk),
    .phy_txd(phy_txd),
    .phy_tx_en(phy_tx_en),
    .phy_col(phy_col),
    .phy_crs(phy_crs),
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
    .ros2_guid_prefix(ros2_guid_prefix),
    .ros2_topic_name(ros2_topic_name),
    .ros2_topic_name_len(ros2_topic_name_len),
    .ros2_topic_type_name(ros2_topic_type_name),
    .ros2_topic_type_name_len(ros2_topic_type_name_len),
    .ros2_app_data(ros2_app_data),
    .ros2_app_data_len(ros2_app_data_len),
    .ros2_app_data_cpu_req(1'b0),
    .ros2_app_data_cpu_rel(1'b0),
    .udp_rxbuf_cpu_rel(rxbuf_cpu_rel),
    .udp_rxbuf_cpu_grant(rxbuf_cpu_grant),
    .udp_rxbuf_addr(rxbuf_addr),
    .udp_rxbuf_ce(rxbuf_ce),
    .udp_rxbuf_we(rxbuf_we),
    .udp_rxbuf_wdata(rxbuf_wdata),
    .udp_txbuf_cpu_grant(txbuf_cpu_grant),
    .udp_txbuf_cpu_rel(txbuf_cpu_rel),
    .udp_txbuf_addr(txbuf_addr),
    .udp_txbuf_rdata(txbuf_rdata),
    .payloadsmem_addr(payloadsmem_addr),
    .payloadsmem_ce(payloadsmem_cs),
    .payloadsmem_we(payloadsmem_we),
    .payloadsmem_wdata(payloadsmem_wdata),
    .payloadsmem_rdata(payloadsmem_rdata)
);

endmodule

module ros2_ether (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    input  wire       phy_col,
    input  wire       phy_crs,
    output wire       phy_rst_n,

    input wire [47:0] mac_addr,
    input wire [31:0] ip_addr,
    input wire [31:0] gateway_ip_addr,
    input wire [31:0] subnet_mask,

    input wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name,
    input wire [7:0] ros2_node_name_len,
    input wire [15:0] ros2_node_udp_port,
    input wire [15:0] ros2_cpu_udp_port,
    input wire [15:0] ros2_port_num_seed,
    input wire [31:0] ros2_tx_period,
    input wire [95:0] ros2_guid_prefix,
    input wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_topic_name,
    input wire [7:0] ros2_topic_name_len,
    input wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_topic_type_name,
    input wire [7:0] ros2_topic_type_name_len,
    input wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_data,
    input wire [7:0] ros2_app_data_len,
    input wire ros2_app_data_cpu_req,
    input wire ros2_app_data_cpu_rel,
    output wire ros2_app_data_cpu_grant,
    input wire udp_rxbuf_cpu_rel,
    output wire udp_rxbuf_cpu_grant,
    output wire [`UDP_RXBUF_AWIDTH-1:0] udp_rxbuf_addr,
    output wire udp_rxbuf_ce,
    output wire udp_rxbuf_we,
    output wire [31:0] udp_rxbuf_wdata,
    input wire udp_txbuf_cpu_rel,
    output wire udp_txbuf_cpu_grant,
    output wire [`UDP_TXBUF_AWIDTH-1:0] udp_txbuf_addr,
    input wire [31:0] udp_txbuf_rdata,
    output wire [`PAYLOADSMEM_AWIDTH-1:0] payloadsmem_addr,
    output wire payloadsmem_ce,
    output wire payloadsmem_we,
    output wire [7:0] payloadsmem_wdata,
    input wire [7:0] payloadsmem_rdata
);

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
    .clk(clk),
    .rst_n(rst_n),

    .phy_rx_clk(phy_rx_clk),
    .phy_rxd(phy_rxd),
    .phy_rx_dv(phy_rx_dv),
    .phy_rx_er(phy_rx_er),
    .phy_tx_clk(phy_tx_clk),
    .phy_txd(phy_txd),
    .phy_tx_en(phy_tx_en),
    .phy_col(phy_col),
    .phy_crs(phy_crs),
    .phy_reset_n(phy_rst_n),

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

    .local_mac({mac_addr[7:0], mac_addr[15:8], mac_addr[23:16], mac_addr[31:24], mac_addr[39:32], mac_addr[47:40]}),
    .local_ip({ip_addr[7:0], ip_addr[15:8], ip_addr[23:16], ip_addr[31:24]}),
    .gateway_ip({gateway_ip_addr[7:0], gateway_ip_addr[15:8], gateway_ip_addr[23:16], gateway_ip_addr[31:24]}),
    .subnet_mask({subnet_mask[7:0], subnet_mask[15:8], subnet_mask[23:16], subnet_mask[31:24]})
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
    .clk(clk),
    .rst_n(rst_n),
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
    .clk(clk),
    .rst_n(rst_n),
    .wr_en(rx_fifo_wr_en),
    .din(rx_fifo_din),
    .full(rx_fifo_full),
    .rd_en(rx_fifo_rd_en),
    .dout(rx_fifo_dout),
    .empty(rx_fifo_empty)
);

// arbiter for sharing app_data between CPU and ROS2rapper IP
localparam APP_DATA_GRANT_NONE = 2'b00;
localparam APP_DATA_GRANT_IP   = 2'b01;
localparam APP_DATA_GRANT_CPU  = 2'b10;

reg [1:0] r_ros2_app_data_grant;
wire ros2_app_data_ip_req, ros2_app_data_ip_rel, ros2_app_data_ip_grant;
assign ros2_app_data_ip_grant = r_ros2_app_data_grant[0];
assign ros2_app_data_cpu_grant = r_ros2_app_data_grant[1];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
    end else begin
        case (r_ros2_app_data_grant)
            APP_DATA_GRANT_NONE: begin
                case ({ros2_app_data_ip_req, ros2_app_data_cpu_req})
                    4'b00: r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
                    4'b01: r_ros2_app_data_grant <= APP_DATA_GRANT_CPU;
                    4'b10: r_ros2_app_data_grant <= APP_DATA_GRANT_IP;
                    4'b11: r_ros2_app_data_grant <= APP_DATA_GRANT_IP;
                endcase
            end
            APP_DATA_GRANT_IP:
                if (ros2_app_data_ip_rel) r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
            APP_DATA_GRANT_CPU:
                if (ros2_app_data_cpu_rel) r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
            default:
                r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
        endcase
    end
end

// arbiter for sharing UDP RX buffer between CPU and ROS2rapper IP
localparam UDP_RXBUF_GRANT_IP   = 1'b0;
localparam UDP_RXBUF_GRANT_CPU  = 1'b1;

reg r_udp_rxbuf_grant;
wire udp_rxbuf_ip_rel, udp_rxbuf_ip_grant;
assign udp_rxbuf_ip_grant = ~r_udp_rxbuf_grant;
assign udp_rxbuf_cpu_grant = r_udp_rxbuf_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_IP;
    end else begin
        case (r_udp_rxbuf_grant)
            UDP_RXBUF_GRANT_IP:
                if (udp_rxbuf_ip_rel) r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_CPU;
            UDP_RXBUF_GRANT_CPU:
                if (udp_rxbuf_cpu_rel) r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_IP;
        endcase
    end
end

// arbiter for sharing UDP TX buffer between CPU and ROS2rapper IP
localparam UDP_TXBUF_GRANT_IP   = 1'b0;
localparam UDP_TXBUF_GRANT_CPU  = 1'b1;

reg r_udp_txbuf_grant;
wire udp_txbuf_ip_rel, udp_txbuf_ip_grant;
assign udp_txbuf_ip_grant = ~r_udp_txbuf_grant;
assign udp_txbuf_cpu_grant = r_udp_txbuf_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_udp_txbuf_grant <= UDP_TXBUF_GRANT_CPU;
    end else begin
        case (r_udp_txbuf_grant)
            UDP_TXBUF_GRANT_IP:
                if (udp_txbuf_ip_rel) r_udp_txbuf_grant <= UDP_TXBUF_GRANT_CPU;
            UDP_TXBUF_GRANT_CPU:
                if (udp_txbuf_cpu_rel) r_udp_txbuf_grant <= UDP_TXBUF_GRANT_IP;
        endcase
    end
end

ros2
ros2_i (
    .ap_clk(clk),
    .ap_rst_n(rst_n),
    .in_V_dout(rx_fifo_dout),
    .in_V_empty_n(~rx_fifo_empty),
    .in_V_read(rx_fifo_rd_en),
    .out_V_din(tx_fifo_din),
    .out_V_full_n(~tx_fifo_full),
    .out_V_write(tx_fifo_wr_en),
    .udp_rxbuf_address0(udp_rxbuf_addr),
    .udp_rxbuf_ce0(udp_rxbuf_ce),
    .udp_rxbuf_we0(udp_rxbuf_we),
    .udp_rxbuf_d0(udp_rxbuf_wdata),
    .udp_txbuf_address0(udp_txbuf_addr),
    .udp_txbuf_q0(udp_txbuf_rdata),
    .payloads_address0(payloadsmem_addr),
    .payloads_ce0(payloadsmem_ce),
    .payloads_we0(payloadsmem_we),
    .payloads_d0(payloadsmem_wdata),
    .payloads_q0(payloadsmem_rdata),
    .conf_ip_addr(ip_addr),
    .conf_node_name(ros2_node_name),
    .conf_node_name_len(ros2_node_name_len),
    .conf_node_udp_port({ros2_node_udp_port[7:0], ros2_node_udp_port[15:8]}),
    .conf_cpu_udp_port({ros2_cpu_udp_port[7:0], ros2_cpu_udp_port[15:8]}),
    .conf_port_num_seed(ros2_port_num_seed),
    .conf_tx_period(ros2_tx_period),
    .conf_guid_prefix(ros2_guid_prefix),
    .conf_topic_name(ros2_topic_name),
    .conf_topic_name_len(ros2_topic_name_len),
    .conf_topic_type_name(ros2_topic_type_name),
    .conf_topic_type_name_len(ros2_topic_type_name_len),
    .conf_app_data(ros2_app_data),
    .conf_app_data_len(ros2_app_data_len),
    .app_data_req_ap_vld(ros2_app_data_ip_req),
    .app_data_rel_ap_vld(ros2_app_data_ip_rel),
    .app_data_grant({7'b0, ros2_app_data_ip_grant}),
    .udp_rxbuf_rel_ap_vld(udp_rxbuf_ip_rel),
    .udp_rxbuf_grant({7'b0, udp_rxbuf_ip_grant}),
    .udp_txbuf_rel_ap_vld(udp_txbuf_ip_rel),
    .udp_txbuf_grant({7'b0, udp_txbuf_ip_grant})
);

ip_tx
ip_tx_i (
    .ap_clk(clk),
    .ap_rst_n(rst_n),
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
    .ap_clk(clk),
    .ap_rst_n(rst_n),
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
