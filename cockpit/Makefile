# N1 cockpit (Raspberry Pi). Build on the Pi: make -C ~/lab5/cockpit
# Anything in ../shared (portable Raft code, also built into the STM32 firmware) is linked in.
#
# Part 2 default_leader and the election seed, the same values as the STM32 builds:
#   make RAFT_LEADER=random RAFT_SEED=9      (defaults: cockpit, 1)
# deploy.sh passes them through and always rebuilds, since make can't see a changed flag.

CC     = gcc
CFLAGS = -O2 -Wall -pthread -I../shared
SRC    = cockpit.c $(wildcard ../shared/*.c)
HDR    = $(wildcard *.h ../shared/*.h)

LEADER_cockpit    = RAFT_NODE_COCKPIT
LEADER_steering   = RAFT_NODE_STEERING
LEADER_drivetrain = RAFT_NODE_DRIVETRAIN
LEADER_random     = RAFT_NODE_NONE

ifdef RAFT_LEADER
ifeq ($(LEADER_$(RAFT_LEADER)),)
$(error RAFT_LEADER must be cockpit, steering, drivetrain or random, not $(RAFT_LEADER))
endif
CFLAGS += -DRAFT_DEFAULT_LEADER=$(LEADER_$(RAFT_LEADER))
endif
ifdef RAFT_SEED
CFLAGS += -DRAFT_SEED=$(RAFT_SEED)
endif

cockpit: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ $(SRC)

clean:
	rm -f cockpit

.PHONY: clean
