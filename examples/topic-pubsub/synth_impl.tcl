cd "ros2rapper-pubsub"
open_project "ros2rapper-pubsub.xpr"

set_property part xc7a100tcsg324-1 [current_project]

reset_run synth_1
launch_runs synth_1 -jobs 16
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
	error "ERROR: synth_1 failed"
}

launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
	error "ERROR: impl_1 failed"
}

close_project
