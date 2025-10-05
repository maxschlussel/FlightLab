.PHONY: all build clean run

CC = gcc
CFLAGS = -Wall -O0 -g -I. -I./src
SRC = $(wildcard src/*/*/*/*.c) $(wildcard src/*/*/*.c) $(wildcard src/*/*.c) $(wildcard *.c)

LOG_PATH = output/data_log.csv
EXE = build/flightlab.exe
SIM_CONFIG = examples/basic_aricraft.json
PYTHON = .venv/Scripts/python.exe

all: build run
	
build:
	$(CC) $(CFLAGS) $(SRC) -o $(EXE)

run:
	$(EXE) --data_log $(LOG_PATH) --config $(SIM_CONFIG)
	$(PYTHON) scripts/run_post_proc.py  --data_log $(LOG_PATH) --plot_csv

clean:
	del $(EXE)
	del *.o
	del $(LOG_PATH)
