SRCDIR = gensrc

.PHONY: all clean cleanall cwb vitis vivado-create-proj

all: vitis

cwb: synth
	$(MAKE) -C ros2rapper -f Makefile.cwb synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/ros2.v ${SRCDIR}

vitis: synth
	$(MAKE) -C ros2rapper -f Makefile.vitis synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2rapper/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}
	./fix-hls-code.rb
	find ${SRCDIR} -name "*.v" | xargs sed -i "/\`timescale .*/d"

vivado-create-proj: copy-src
	vivado -mode batch -source create_project.tcl

clean:
	rm -rf ${SRCDIR}
	rm -f *.jou *.log *.xpr
	rm -rf project.hw project.cache project.runs project.sim project.ip_user_files

cleanall: clean
	$(MAKE) -C ros2rapper -f Makefile.vitis clean
	$(MAKE) -C ros2rapper -f Makefile.cwb clean
