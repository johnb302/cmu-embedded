# Build and run the shared/ self-tests on the Mac: make -C shared/test
#   host_test: one node, frames fed in by hand
#   sim:       three nodes on simulated buses, each its own copy of shared/ (n1.so ...)
#              ./sim                default_leader = cockpit (n*.so)
#              ./sim random         LEADER=random (r*.so)
#              ./sim random-seed9   LEADER=random, seed 9: the Pi wins (p*.so)
#              ./sim watch          steps 6-9: link watching and relaying (n*.so)
CFLAGS  = -std=c11 -Wall -Wextra -I..
SHARED  = $(wildcard ../*.c)
HEADERS = $(wildcard ../*.h)
RANDOM  = -DRAFT_DEFAULT_LEADER=RAFT_NODE_NONE

ifeq ($(shell uname),Darwin)
SO_FLAGS  = -dynamiclib -undefined dynamic_lookup
SIM_FLAGS =
else
SO_FLAGS  = -shared -Wl,-Bsymbolic
SIM_FLAGS = -rdynamic -ldl
endif

LIBS = n1.so n2.so n3.so r1.so r2.so r3.so p1.so p2.so p3.so

run: host_test sim $(LIBS)
	./host_test
	./sim
	./sim random
	./sim random-seed9
	./sim watch

host_test: host_test.c $(SHARED) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ host_test.c $(SHARED)

# raft_platform_send is left undefined in each .so: the sim defines it
n%.so: $(SHARED) $(HEADERS)
	$(CC) $(CFLAGS) -fPIC $(SO_FLAGS) -o $@ $(SHARED)

r%.so: $(SHARED) $(HEADERS)
	$(CC) $(CFLAGS) $(RANDOM) -fPIC $(SO_FLAGS) -o $@ $(SHARED)

p%.so: $(SHARED) $(HEADERS)
	$(CC) $(CFLAGS) $(RANDOM) -DRAFT_SEED=9 -fPIC $(SO_FLAGS) -o $@ $(SHARED)

sim: sim.c $(HEADERS)
	$(CC) $(CFLAGS) -o $@ sim.c $(SIM_FLAGS)

clean:
	rm -f host_test sim *.so

.PHONY: run clean
