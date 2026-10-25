CORE := ../../firmware/components/onspeed_core
CXXFLAGS := -std=c++17 -O1 -g -Wall -Wextra -I$(CORE)/include -fsanitize=address,undefined -fno-sanitize-recover=all
SRC := test_core.cpp $(CORE)/onspeed_core.cpp $(CORE)/tone_synth.cpp $(CORE)/g5.cpp
test_core: $(SRC) $(wildcard $(CORE)/include/*.h)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@ -lm
run: test_core
	./test_core
.PHONY: run
