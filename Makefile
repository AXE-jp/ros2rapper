SRCDIR = gensrc

.PHONY: all clean cleanall synth-ros2rapper synth copy-src vivado-create-proj

all: copy-src

synth: synth-ros2rapper

synth-ros2rapper:
	$(MAKE) -C ros2rapper synth

copy-src: synth
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
	$(MAKE) -C ros2rapper clean
