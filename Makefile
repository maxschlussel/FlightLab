CC = gcc
CFLAGS = -Wall -O0 -g \
-I. -I./src
SRC = $(wildcard src/*/*/*/*.c) $(wildcard src/*/*/*.c) $(wildcard src/*/*.c) $(wildcard *.c)

EXE = build\flightlab.exe
PYTHON = .\.venv\Scripts\python.exe

all:
	$(CC) $(CFLAGS) $(SRC) -o $(EXE)
	.\$(EXE)
	$(PYTHON) scripts/plot.py  --data_log output/data_log.csv

build:
	$(CC) $(CFLAGS) $(SRC) -o $(EXE)

clean:
	del $(EXE)
	del *.o
	del output/data_log.csv
