# Build and run the shared/ self-tests on the Mac: make -C shared/test
#   host_test: one node, frames fed in by hand
#   sim:       three nodes on simulated buses, each its own copy of shared/ (n1.so ...)
CFLAGS  = -std=c11 -Wall -Wextra -I..
SHARED  = $(wildcard ../*.c)
HEADERS = $(wildcard ../*.h)

ifeq ($(shell uname),Darwin)
SO_FLAGS  = -dynamiclib -undefined dynamic_lookup
SIM_FLAGS =
else
SO_FLAGS  = -shared -Wl,-Bsymbolic
SIM_FLAGS = -rdynamic -ldl
endif

run: host_test sim
	./host_test
	./sim

host_test: host_test.c $(SHARED) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ host_test.c $(SHARED)

# raft_platform_send is left undefined in each .so: the sim defines it
n%.so: $(SHARED) $(HEADERS)
	$(CC) $(CFLAGS) -fPIC $(SO_FLAGS) -o $@ $(SHARED)

sim: sim.c n1.so n2.so n3.so $(HEADERS)
	$(CC) $(CFLAGS) -o $@ sim.c $(SIM_FLAGS)

clean:
	rm -f host_test sim n*.so

.PHONY: run clean
