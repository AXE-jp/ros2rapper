.PHONY: create-proj clean

create-proj: vitis
	vivado -mode batch -source create_project.tcl

clean:
	rm -f *.jou *.log *.xpr
	rm -rf ros2rapper-project
