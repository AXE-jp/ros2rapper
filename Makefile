SRCDIR = gensrc

.PHONY: all clean cleanall synth-ip_tx synth-ip_rx synth-ros2rapper synth copy-src vivado-create-proj

all: copy-src

synth: synth-ip_tx synth-ip_rx synth-ros2rapper

synth-ip_tx:
	$(MAKE) -C ip_tx_rx/ip_tx synth

synth-ip_rx:
	$(MAKE) -C ip_tx_rx/ip_rx synth

synth-ros2rapper:
	$(MAKE) -C ros2rapper synth

copy-src: synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	cp ether-src/*.v ${SRCDIR}
	cp ether-src/*.vh ${SRCDIR}
	cp ip_tx_rx/ip_tx/proj_ip_tx/solution1/syn/verilog/*.v ${SRCDIR}
	cp ip_tx_rx/ip_rx/proj_ip_rx/solution1/syn/verilog/*.v ${SRCDIR}
	cp ros2rapper/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	cp ros2rapper/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}
	cp ../config.vh ${SRCDIR}
	cp ../priority_encoder.v ${SRCDIR}
	cp ../ram.v ${SRCDIR}

vivado-create-proj: copy-src
	vivado -mode batch -source create_project.tcl

clean:
	rm -rf ${SRCDIR}
	rm -f *.jou *.log *.xpr
	rm -rf project.hw project.cache project.runs project.sim project.ip_user_files

cleanall: clean
	$(MAKE) -C ip_tx_rx/ip_tx clean
	$(MAKE) -C ip_tx_rx/ip_rx clean
	$(MAKE) -C ros2rapper clean
