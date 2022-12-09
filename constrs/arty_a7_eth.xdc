set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]

set_property -dict {LOC E3 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name clk [get_ports clk]

set_property -dict {LOC C2 IOSTANDARD LVCMOS33} [get_ports reset_n]

set_false_path -from [get_ports reset_n]
set_input_delay 0.000 [get_ports reset_n]

set_property -dict {LOC F15 IOSTANDARD LVCMOS33} [get_ports phy_rx_clk]
set_property -dict {LOC D18 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[0]}]
set_property -dict {LOC E17 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[1]}]
set_property -dict {LOC E18 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[2]}]
set_property -dict {LOC G17 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[3]}]
set_property -dict {LOC G16 IOSTANDARD LVCMOS33} [get_ports phy_rx_dv]
set_property -dict {LOC C17 IOSTANDARD LVCMOS33} [get_ports phy_rx_er]
set_property -dict {LOC H16 IOSTANDARD LVCMOS33} [get_ports phy_tx_clk]
set_property -dict {LOC H14 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[0]}]
set_property -dict {LOC J14 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[1]}]
set_property -dict {LOC J13 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[2]}]
set_property -dict {LOC H17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[3]}]
set_property -dict {LOC H15 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports phy_tx_en]
set_property -dict {LOC D17 IOSTANDARD LVCMOS33} [get_ports phy_col]
set_property -dict {LOC G14 IOSTANDARD LVCMOS33} [get_ports phy_crs]
set_property -dict {LOC G18 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports phy_ref_clk]
set_property -dict {LOC C16 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports phy_reset_n]
#set_property -dict {LOC K13  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports phy_mdio]
#set_property -dict {LOC F16  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports phy_mdc]

create_clock -period 40.000 -name phy_rx_clk [get_ports phy_rx_clk]
create_clock -period 40.000 -name phy_tx_clk [get_ports phy_tx_clk]

set_false_path -to [get_ports {phy_ref_clk phy_reset_n}]
set_output_delay 0.000 [get_ports {phy_ref_clk phy_reset_n}]

#set_false_path -to [get_ports {phy_mdio phy_mdc}]
#set_output_delay 0 [get_ports {phy_mdio phy_mdc}]
#set_false_path -from [get_ports {phy_mdio}]
#set_input_delay 0 [get_ports {phy_mdio}]





create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 4096 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list clk_int]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 32 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {ros2_udp_rxbuf_d[0]} {ros2_udp_rxbuf_d[1]} {ros2_udp_rxbuf_d[2]} {ros2_udp_rxbuf_d[3]} {ros2_udp_rxbuf_d[4]} {ros2_udp_rxbuf_d[5]} {ros2_udp_rxbuf_d[6]} {ros2_udp_rxbuf_d[7]} {ros2_udp_rxbuf_d[8]} {ros2_udp_rxbuf_d[9]} {ros2_udp_rxbuf_d[10]} {ros2_udp_rxbuf_d[11]} {ros2_udp_rxbuf_d[12]} {ros2_udp_rxbuf_d[13]} {ros2_udp_rxbuf_d[14]} {ros2_udp_rxbuf_d[15]} {ros2_udp_rxbuf_d[16]} {ros2_udp_rxbuf_d[17]} {ros2_udp_rxbuf_d[18]} {ros2_udp_rxbuf_d[19]} {ros2_udp_rxbuf_d[20]} {ros2_udp_rxbuf_d[21]} {ros2_udp_rxbuf_d[22]} {ros2_udp_rxbuf_d[23]} {ros2_udp_rxbuf_d[24]} {ros2_udp_rxbuf_d[25]} {ros2_udp_rxbuf_d[26]} {ros2_udp_rxbuf_d[27]} {ros2_udp_rxbuf_d[28]} {ros2_udp_rxbuf_d[29]} {ros2_udp_rxbuf_d[30]} {ros2_udp_rxbuf_d[31]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 6 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {ros2_udp_rxbuf_addr[0]} {ros2_udp_rxbuf_addr[1]} {ros2_udp_rxbuf_addr[2]} {ros2_udp_rxbuf_addr[3]} {ros2_udp_rxbuf_addr[4]} {ros2_udp_rxbuf_addr[5]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 1 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list ros2_udp_rxbuf_ce]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 1 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list ros2/ros2_udp_rxbuf_ip_grant]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 1 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list ros2/ros2_udp_rxbuf_ip_rel]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 1 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list ros2_udp_rxbuf_we]]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_int]
