CC = gcc
CFLAGS = -Wall
LIBS = -lmosquitto -lcjson

all: jbd_exporter

jbd_exporter: main.c
	$(CC) $(CFLAGS) -o jbd_exporter main.c $(LIBS)

clean:
	rm -f jbd_exporter
