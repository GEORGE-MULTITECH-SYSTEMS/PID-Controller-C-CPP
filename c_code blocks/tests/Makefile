CC      := gcc
CFLAGS  ?= -std=c11 -O0 -g -Wall -Wextra -Wpedantic
BUILD   := build
ROOT    := ..
INC     := -I $(ROOT) -I .

all: test demo

test: $(BUILD)/test_pid
	./$(BUILD)/test_pid

demo: $(BUILD)/pid_demo
	./$(BUILD)/pid_demo

$(BUILD)/test_pid: test_pid.c pid_tick_stub.c pid_tick_stub.h $(ROOT)/pid.c $(ROOT)/pid.h $(ROOT)/pid_tick.h | $(BUILD)
	$(CC) $(CFLAGS) $(INC) -o $@ test_pid.c pid_tick_stub.c $(ROOT)/pid.c -lm

$(BUILD)/pid_demo: $(ROOT)/example/main.c $(ROOT)/pid.c $(ROOT)/pid.h $(ROOT)/platform/pid_tick_c11.c | $(BUILD)
	$(CC) $(CFLAGS) $(INC) -o $@ $(ROOT)/example/main.c $(ROOT)/pid.c $(ROOT)/platform/pid_tick_c11.c -lm

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

.PHONY: all test demo clean
