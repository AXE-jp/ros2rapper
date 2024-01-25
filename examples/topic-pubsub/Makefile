.PHONY: create-proj synth clean

create-proj:
	$(MAKE) -C ../src/ros2rapper vitis
	vivado -mode batch -source create_project.tcl

synth:
	vivado -mode batch -source synth_impl.tcl

clean:
	rm -f *.jou *.log *.xpr
	rm -rf ros2rapper-pubsub
