SRCDIR = src

.PHONY: clean clone copy-src create-proj

clean:
	-rm -rf ${SRCDIR}
	-rm -rf arty_a7_eth
	-rm -rf ip_tx_rx
	-rm -rf ros2
	-rm -rf verilog-ethernet

clone:
	git clone eva:/opt/arty_a7_eth.git
	git clone eva:/opt/ip_tx_rx.git
	git clone eva:/opt/ros2.git
	git clone eva:/opt/verilog-ethernet.git

copy-src:
	mkdir -p ${SRCDIR}
	-cp arty_a7_eth/arty_a7_eth.srcs/sources_1/imports/arty_a7_eth/*.v ${SRCDIR}
	-cp ip_tx_rx/ip_tx/proj_ip_tx/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ip_tx_rx/ip_rx/proj_ip_rx/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}

create-proj:

