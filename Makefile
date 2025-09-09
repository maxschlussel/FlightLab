CC = gcc
CFLAGS = -Wall -O0 -g \
-I. -I./src
SRC = $(wildcard src/*/*/*/*.c) $(wildcard src/*/*/*.c) $(wildcard src/*/*.c) $(wildcard *.c)
LOG_PATH = output/data_log.csv

EXE = build\flightlab.exe
PYTHON = .\.venv\Scripts\python.exe

all:
	$(CC) $(CFLAGS) $(SRC) -o $(EXE)
	.\$(EXE) $(LOG_PATH)
	$(PYTHON) scripts/plot.py  --data_log -$(LOG_PATH)

build:
	$(CC) $(CFLAGS) $(SRC) -o $(EXE)

clean:
	del $(EXE)
	del *.o
	del -$(LOG_PATH)
