CC ?= gcc
CFLAGS ?= -g -O0 -Wall -Wextra -Wpedantic
SANFLAGS ?= -fsanitize=address,undefined -fno-omit-frame-pointer

.PHONY: all test clean

all: qla_hole_test qla

qla_hole_test: qla_hole_test.c qla.h qli.h
	$(CC) $(CFLAGS) $(SANFLAGS) -o $@ qla_hole_test.c

test: qla_hole_test
	./qla_hole_test

qla: qla.c qli.h qla.h
	gcc -O0 -Wall -Wextra -Wpedantic -o qla qla.c

clean:
	rm -f qla_hole_test *.o qla
