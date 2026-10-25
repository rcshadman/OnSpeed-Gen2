# Runs the QEMU flight profile through the simulated ISM330DHCX and the Gen2 AHRS port on the host.
FW ?= ../../firmware
run_flight: run_flight.cpp $(FW)/main/flight_model.cpp $(FW)/main/imu_sim.cpp $(FW)/components/onspeed_core/ahrs.cpp
	g++ -O2 -Wall -I$(FW)/main -I$(FW)/components/onspeed_core/include $^ -o $@
run: run_flight
	./run_flight | tail -1 && ./run_flight fixes | tail -1
