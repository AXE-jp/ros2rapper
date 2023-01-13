PROJNAME = proj_ros2
SRCS = app.cpp checksum.cpp ip.cpp ros2.cpp rtps.cpp sedp.cpp slip.cpp spdp.cpp udp.cpp
HDRS = app.hpp checksum.hpp common.hpp duration.hpp endpoint.hpp hls.hpp ip.hpp ros2.hpp rtps.hpp sedp.hpp slip.hpp spdp.hpp timestamp.hpp udp.hpp

.PHONY: synth clean
synth: $(PROJNAME)

$(PROJNAME): $(SRCS) $(HDRS) run_hls.tcl
	vitis_hls run_hls.tcl
	./fix-synthesized.rb

clean:
	rm -rf $(PROJNAME)
	rm -rf *.log
