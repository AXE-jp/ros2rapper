SRCDIR = gensrc

.PHONY: all clean cleanall clone copy-src synth create-proj

all:

clean:
	-rm -rf ${SRCDIR}
	-rm *.jou *.log *.xpr
	-rm -rf project.hw project.cache project.runs project.sim project.ip_user_files

cleanall: clean
	-rm -rf ip_tx_rx
	-rm -rf ros2

clone:
	git clone -b master laxer-git@www4.axe.bz:/opt/git/ip_tx_rx.git
	git clone -b use-fifoif-ethernet laxer-git@www4.axe.bz:/opt/git/ros2.git

copy-src: clone
	mkdir -p ${SRCDIR}
	cp ether-src/*.v ${SRCDIR}
	cp ether-src/*.vh ${SRCDIR}
	cp ip_tx_rx/ip_tx/proj_ip_tx/solution1/syn/verilog/*.v ${SRCDIR}
	cp ip_tx_rx/ip_rx/proj_ip_rx/solution1/syn/verilog/*.v ${SRCDIR}
	cp ros2/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	cp ros2/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}

synth: copy-src
	(cd ip_tx_rx/ip_tx; vitis_hls run_hls.tcl)
	(cd ip_tx_rx/ip_rx; vitis_hls run_hls.tcl)
	(cd ros2; vitis_hls run_hls.tcl)

create-proj: synth
	vivado -mode batch -source create_project.tcl
