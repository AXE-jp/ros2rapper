SRCDIR = gensrc

.PHONY: all clean cleanall synth-ip_tx synth-ip_rx synth-ros2rapper synth copy-src synth create-proj

all:

clean:
	-rm -rf ${SRCDIR}
	-rm *.jou *.log *.xpr
	-rm -rf project.hw project.cache project.runs project.sim project.ip_user_files

synth-ip_tx:
	(cd ip_tx_rx/ip_tx; vitis_hls run_hls.tcl)

synth-ip_rx:
	(cd ip_tx_rx/ip_rx; vitis_hls run_hls.tcl)

synth-ros2rapper:
	(cd ros2rapper; vitis_hls run_hls.tcl)

synth: synth-ip_tx synth-ip_rx synth-ros2rapper

copy-src:
	mkdir -p ${SRCDIR}
	cp ether-src/*.v ${SRCDIR}
	cp ether-src/*.vh ${SRCDIR}
	cp ip_tx_rx/ip_tx/proj_ip_tx/solution1/syn/verilog/*.v ${SRCDIR}
	cp ip_tx_rx/ip_rx/proj_ip_rx/solution1/syn/verilog/*.v ${SRCDIR}
	cp ros2rapper/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	cp ros2rapper/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}

create-proj: synth copy-src
	vivado -mode batch -source create_project.tcl
