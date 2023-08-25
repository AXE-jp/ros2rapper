SRCDIR = gensrc

.PHONY: clean cleanall cwb vitis

cwb:
	$(MAKE) -C ros2rapper -f Makefile.cwb synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/ros2.v ${SRCDIR}

vitis:
	$(MAKE) -C ros2rapper -f Makefile.vitis synth
	-rm -rf ${SRCDIR}
	mkdir -p ${SRCDIR}
	-cp ether-src/*.v ${SRCDIR}
	-cp ros2rapper/proj_ros2/solution1/syn/verilog/*.v ${SRCDIR}
	-cp ros2rapper/proj_ros2/solution1/syn/verilog/*.dat ${SRCDIR}
	./fix-hls-code.rb
	find ${SRCDIR} -name "*.v" | xargs sed -i "/\`timescale .*/d"

clean:
	rm -rf ${SRCDIR}
	rm -f *.jou *.log

cleanall: clean
	$(MAKE) -C ros2rapper -f Makefile.vitis clean
	$(MAKE) -C ros2rapper -f Makefile.cwb clean
