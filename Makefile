SRCDIR = src

.PHONY: all clean clone copy-src synth create-proj

all:

clean:
	-rm -rf ${SRCDIR}
	-rm -rf arty_a7_eth
	-rm -rf ip_tx_rx
	-rm -rf ros2
	-rm -rf verilog-ethernet
	-rm *.jou *.log *.xpr
	-rn -rf project.hw project.cache project.runs project.sim project.ip_user_files

clone:
	git clone -b master eva:/opt/git/arty_a7_eth.git
	git clone -b master eva:/opt/git/ip_tx_rx.git
	git clone -b use-fifoif-ethernet eva:/opt/git/ros2.git
	#git clone -b master eva:/opt/git/verilog-ethernet.git

copy-src:
	mkdir -p ${SRCDIR}
	-cp arty_a7_eth/arty_a7_eth.srcs/sources_1/imports/arty_a7_eth/*.v ${SRCDIR}
	-cp arty_a7_eth/arty_a7_eth.srcs/sources_1/imports/arty_a7_eth/*.vh ${SRCDIR}
	-cp arty_a7_eth/arty_a7_eth.srcs/constrs_1/imports/arty_a7_eth/arty_a7_eth.xdc .
	-cp ip_tx_rx/ip_tx/proj_ip_tx/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ip_tx_rx/ip_rx/proj_ip_rx/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}

synth:
	(cd ip_tx_rx/ip_tx; vitis_hls run_hls.tcl)
	(cd ip_tx_rx/ip_rx; vitis_hls run_hls.tcl)
	(cd ros2; vitis_hls run_hls.tcl)

create-proj:

