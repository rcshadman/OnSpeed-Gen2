# Host test for the OnSpeed Gen2 AHRS port. No ESP-IDF needed.
CORE    := ../../firmware/components/onspeed_core
CXX     ?= g++
XTENSA  ?= /home/claude/sim/tools/xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++
OBJDUMP ?= /home/claude/sim/tools/xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump
NM      ?= /home/claude/sim/tools/xtensa-esp-elf/bin/xtensa-esp32s3-elf-nm
# The port itself must be clean under the strict single-precision warning set.
PORTFLAGS := -O2 -std=c++17 -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Wconversion \
             -fsingle-precision-constant -Werror -I$(CORE)/include
# Test/simulator code is double precision on purpose (truth model).
TESTFLAGS := -O2 -std=c++17 -Wall -Wextra -I$(CORE)/include

all: test_ahrs bench_ahrs opcount_host

ahrs.o: $(CORE)/ahrs.cpp $(CORE)/include/ahrs.h
	$(CXX) $(PORTFLAGS) -c $< -o $@

test_ahrs: test_ahrs.cpp sim.h ahrs.o
	$(CXX) $(TESTFLAGS) test_ahrs.cpp ahrs.o -o $@

bench_ahrs: bench.cpp sim.h ahrs.o
	$(CXX) $(TESTFLAGS) bench.cpp ahrs.o -o $@

# dynamic float-op count: the unmodified ahrs.cpp compiled with a counting float type
opcount_host: opcount_host.cpp $(CORE)/ahrs.cpp $(CORE)/include/ahrs.h
	$(CXX) -O1 -std=c++17 -I$(CORE) -I$(CORE)/include opcount_host.cpp -o $@

# ESP32-S3 cross-compile: proves no double-precision soft-float calls (__adddf3, __muldf3, ...)
ahrs_s3.o: $(CORE)/ahrs.cpp $(CORE)/include/ahrs.h
	$(XTENSA) -O2 -mlongcalls -std=gnu++17 -Wall -Wextra -Wdouble-promotion -Wfloat-conversion \
	  -fsingle-precision-constant -Werror -I$(CORE)/include -c $< -o $@

s3check: ahrs_s3.o
	@echo "undefined symbols in ahrs_s3.o:"; $(NM) -u ahrs_s3.o
	@if $(NM) -u ahrs_s3.o | grep -E 'df[0-9]|sfdf|dfsf|dfdi|didf|dfsi|sidf' ; then \
	   echo "FAIL: double-precision helpers referenced"; exit 1; else echo "OK: no double-precision helpers"; fi
	$(OBJDUMP) -d -C ahrs_s3.o > ahrs_s3.dis
	python3 opcount.py ahrs_s3.dis

run: all s3check
	./test_ahrs
	./bench_ahrs
	./opcount_host

clean:
	rm -f *.o test_ahrs bench_ahrs opcount_host ahrs_s3.dis results_table.md out.txt

.PHONY: all run clean s3check
