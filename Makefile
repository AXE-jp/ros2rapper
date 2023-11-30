SRCDIR = gensrc

.PHONY: all clean cleanall fake-cwb cwb vitis vivado-create-proj

all:
	echo "Target is not specified."
	exit 1

fake-cwb:
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/ros2.v ${SRCDIR}
	sed -i '1s/^/`define ROS2RAPPER_HLS_CWB\n/' ${SRCDIR}/ros2_ether.v

cwb:
	$(MAKE) -C ros2rapper -f Makefile.cwb synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/ros2.v ${SRCDIR}
	sed -i '1s/^/`define ROS2RAPPER_HLS_CWB\n/' ${SRCDIR}/ros2_ether.v

vitis:
	$(MAKE) -C ros2rapper -f Makefile.vitis synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2rapper/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}
	-cp ctrl_demo/proj_ctrl_demo/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ctrl_demo/proj_ctrl_demo/solution1/syn/verilog/*.dat ${SRCDIR}
	./fix-hls-code.rb
	find ${SRCDIR} -name "*.v" | xargs sed -i "/\`timescale .*/d"
	sed -i '1s/^/`define ROS2RAPPER_HLS_VITIS\n/' ${SRCDIR}/ros2_ether.v

vivado-create-proj: vitis
	vivado -mode batch -source create_project.tcl

clean:
	rm -rf ${SRCDIR}
	rm -f *.jou *.log *.xpr
	rm -rf project.hw project.cache project.runs project.sim project.ip_user_files

cleanall: clean
	$(MAKE) -C ros2rapper -f Makefile.vitis clean
	$(MAKE) -C ros2rapper -f Makefile.cwb clean
