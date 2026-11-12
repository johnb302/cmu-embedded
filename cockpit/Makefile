# N1 cockpit (Raspberry Pi). Build on the Pi: make -C ~/lab5/cockpit
# Anything in ../shared (portable Raft code, also built into the STM32 firmware) is linked in.

CC     = gcc
CFLAGS = -O2 -Wall -pthread -I../shared
SRC    = cockpit.c $(wildcard ../shared/*.c)
HDR    = $(wildcard *.h ../shared/*.h)

cockpit: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ $(SRC)

clean:
	rm -f cockpit

.PHONY: clean
