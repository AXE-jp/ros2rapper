# Clock
set_property -dict { PACKAGE_PIN K22 IOSTANDARD LVDS } [get_ports { CLK_200M_P }];
set_property -dict { PACKAGE_PIN K23 IOSTANDARD LVDS } [get_ports { CLK_200M_N }];
create_clock -period 5.000 -name core_clk [get_ports { CLK_200M_P }];
create_generated_clock -name sys_clk -source [get_pins clk_mmcm_inst/CLKIN1] -divide_by 2 [get_pins clk_mmcm_inst/CLKOUT0];

# Reset
set_property -dict { PACKAGE_PIN Y17 IOSTANDARD LVCMOS18 } [get_ports { RSTB }]; # IOB0
set_false_path -from [get_ports { RSTB }];
set_input_delay 0 [get_ports { RSTB }];

set_property -dict { PACKAGE_PIN B9   IOSTANDARD LVCMOS33 } [get_ports { led4 }]; # IOA 0
set_property -dict { PACKAGE_PIN A9   IOSTANDARD LVCMOS33 } [get_ports { led5 }]; # IOA 1
set_property -dict { PACKAGE_PIN B10  IOSTANDARD LVCMOS33 } [get_ports { led6 }]; # IOA 2
set_property -dict { PACKAGE_PIN A10  IOSTANDARD LVCMOS33 } [get_ports { led7 }]; # IOA 3

#set_property -dict { PACKAGE_PIN AA14 IOSTANDARD LVCMOS33 } [get_ports { phy_rx_clk }]; # IOA 48
#set_property -dict { PACKAGE_PIN AA19 IOSTANDARD LVCMOS18 } [get_ports { phy_rxd[0] }]; # IOB 4
#set_property -dict { PACKAGE_PIN AB19 IOSTANDARD LVCMOS18 } [get_ports { phy_rxd[1] }]; # IOB 5
#set_property -dict { PACKAGE_PIN AA20 IOSTANDARD LVCMOS18 } [get_ports { phy_rxd[2] }]; # IOB 6
#set_property -dict { PACKAGE_PIN AB20 IOSTANDARD LVCMOS18 } [get_ports { phy_rxd[3] }]; # IOB 7
#set_property -dict { PACKAGE_PIN AA22 IOSTANDARD LVCMOS18 } [get_ports { phy_rx_dv }]; # IOB 8
#set_property -dict { PACKAGE_PIN AB22 IOSTANDARD LVCMOS18 } [get_ports { phy_rx_er }]; # IOB 9
#set_property -dict { PACKAGE_PIN AB15 IOSTANDARD LVCMOS33 } [get_ports { phy_tx_clk }]; # IOA 50
#set_property -dict { PACKAGE_PIN AC22 IOSTANDARD LVCMOS18 } [get_ports { phy_txd[0] }]; # IOB 10
#set_property -dict { PACKAGE_PIN AC23 IOSTANDARD LVCMOS18 } [get_ports { phy_txd[1] }]; # IOB 11
#set_property -dict { PACKAGE_PIN Y22  IOSTANDARD LVCMOS18 } [get_ports { phy_txd[2] }]; # IOB 12
#set_property -dict { PACKAGE_PIN Y23  IOSTANDARD LVCMOS18 } [get_ports { phy_txd[3] }]; # IOB 13
#set_property -dict { PACKAGE_PIN V23  IOSTANDARD LVCMOS18 } [get_ports { phy_tx_en }]; # IOB 14
#set_property -dict { PACKAGE_PIN W23  IOSTANDARD LVCMOS18 } [get_ports { phy_rst_n }]; # IOB 15

set_property -dict { PACKAGE_PIN AA14 IOSTANDARD LVCMOS33 } [get_ports { phy_rx_clk }]; # IOA 48
set_property -dict { PACKAGE_PIN A13  IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[0] }]; # IOA 4
set_property -dict { PACKAGE_PIN A12  IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[1] }]; # IOA 5
set_property -dict { PACKAGE_PIN B14  IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[2] }]; # IOA 6
set_property -dict { PACKAGE_PIN A14  IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[3] }]; # IOA 7
set_property -dict { PACKAGE_PIN D9   IOSTANDARD LVCMOS33 } [get_ports { phy_rx_dv }]; # IOA 8
set_property -dict { PACKAGE_PIN C9   IOSTANDARD LVCMOS33 } [get_ports { phy_rx_er }]; # IOA 9
set_property -dict { PACKAGE_PIN AB15 IOSTANDARD LVCMOS33 } [get_ports { phy_tx_clk }]; # IOA 50
set_property -dict { PACKAGE_PIN F10  IOSTANDARD LVCMOS33 } [get_ports { phy_txd[0] }]; # IOA 10
set_property -dict { PACKAGE_PIN F9   IOSTANDARD LVCMOS33 } [get_ports { phy_txd[1] }]; # IOA 11
set_property -dict { PACKAGE_PIN G10  IOSTANDARD LVCMOS33 } [get_ports { phy_txd[2] }]; # IOA 12
set_property -dict { PACKAGE_PIN G9   IOSTANDARD LVCMOS33 } [get_ports { phy_txd[3] }]; # IOA 13
set_property -dict { PACKAGE_PIN C11  IOSTANDARD LVCMOS33 } [get_ports { phy_tx_en }]; # IOA 14
set_property -dict { PACKAGE_PIN B11  IOSTANDARD LVCMOS33 } [get_ports { phy_rst_n }]; # IOA 15

create_clock -period 40.000 -name phy_rx_clk [get_ports { phy_rx_clk }];
create_clock -period 40.000 -name phy_tx_clk [get_ports { phy_tx_clk }];

set_clock_groups -asynchronous -group {sys_clk} -group {phy_rx_clk}
set_clock_groups -asynchronous -group {sys_clk} -group {phy_tx_clk}
