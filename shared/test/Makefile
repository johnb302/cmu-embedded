# Build and run the shared/ self-tests on the Mac: make -C shared/test
CFLAGS = -std=c11 -Wall -Wextra -I..
SRC    = host_test.c $(wildcard ../*.c)

run: host_test
	./host_test

host_test: $(SRC) $(wildcard ../*.h)
	$(CC) $(CFLAGS) -o $@ $(SRC)

.PHONY: run
