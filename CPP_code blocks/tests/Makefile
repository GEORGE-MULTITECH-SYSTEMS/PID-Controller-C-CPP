CXX      := g++
CC       := gcc
CXXFLAGS ?= -std=c++20 -O0 -g -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti
CFLAGS   ?= -std=c11 -O0 -g -Wall -Wextra
BUILD    := build
ROOT     := ..
CROOT    := "../../c_code blocks"

all: test demo

test: $(BUILD)/test_pid
	./$(BUILD)/test_pid

demo: $(BUILD)/pid_demo
	./$(BUILD)/pid_demo

$(BUILD)/test_pid: test_pid.cpp $(ROOT)/pid.hpp $(BUILD)/pid_c.o $(BUILD)/pid_tick_stub.o | $(BUILD)
	$(CXX) $(CXXFLAGS) -I $(ROOT) -I $(CROOT) -I $(CROOT)/tests -o $@ test_pid.cpp $(BUILD)/pid_c.o $(BUILD)/pid_tick_stub.o

$(BUILD)/pid_c.o: | $(BUILD)
	$(CC) $(CFLAGS) -I $(CROOT) -c -o $@ $(CROOT)/pid.c

$(BUILD)/pid_tick_stub.o: | $(BUILD)
	$(CC) $(CFLAGS) -I $(CROOT) -I $(CROOT)/tests -c -o $@ $(CROOT)/tests/pid_tick_stub.c

$(BUILD)/pid_demo: $(ROOT)/example/main.cpp $(ROOT)/pid.hpp $(BUILD)/pid_tick_c11.o | $(BUILD)
	$(CXX) $(CXXFLAGS) -I $(ROOT) -I $(CROOT) -o $@ $(ROOT)/example/main.cpp $(BUILD)/pid_tick_c11.o

$(BUILD)/pid_tick_c11.o: | $(BUILD)
	$(CC) $(CFLAGS) -I $(CROOT) -c -o $@ $(CROOT)/platform/pid_tick_c11.c

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

.PHONY: all test demo clean
