# Lab 5 Plan: Time Synchronization and Consensus

Due 2026-11-18. Steps 2–6 are Raft, steps 7–8 are time sync. Time sync only needs step 3's heartbeat, so the two tracks can split after step 3.

## Steps

### 1. Network state machine — DONE
**Adds:** the graph of which nodes and CAN links are up. Log events are applied to it, and `getNetwork()` reads it back.
**Verify:** boot prints `netstate selftest: 11/11 PASS`. In the shell, `net node up 3`, `net link down 3 A`, `net show`.

### 2. Config and replicated log
**Adds:** the shared Raft config (N, default leader, heartbeat period, timeouts, debug). A log of `{term, index, event}` entries that update the network state only once committed.
**Verify:** `raft config` prints the same values on every node. `log append` doesn't change `net show`; `log commit` does.

### 3. Raft over CAN with a fixed leader
**Adds:** Raft CAN frames. Nodes boot with the cockpit as leader in term 1, no election. The leader sends REF every 20 ms and an empty APPEND every 100 ms. LEADER and SYNC LEDs.
**Verify:** `candump -c any` shows REF every 20 ms, APPEND every 100 ms, no REQ_VOTE. The LEADER LED is lit on the Pi only.

### 4. Log replication
**Adds:** an event sent to the leader is replicated to followers and committed once a majority has it. Events sent to a follower get redirected to the leader.
**Verify:** submit an event from any node's shell. `log show` and `net show` match on every node.

### 5. Leader election
**Adds:** when the leader's heartbeats stop, followers time out at random offsets, vote, and elect a new leader in a new term. A "random leader" boot mode with no preset leader.
**Verify:** power off the leader. A new LEADER LED lights within 1 s, the term goes up once, and logs still agree. Pulling a link must not trigger an election.

### 6. Link watching and routing
**Adds:** each node reports node and link failures and recoveries to the leader as log events. Routing (direct bus or relay) is built from `getNetwork()` instead of lab3's heartbeat ages.
**Verify:** pull an A jumper and you get one new log entry, at the same index on every node, with the term unchanged. Pull N1 off B and N3 off A, and steering, blinking and braking still work through the Pi relay.

### 7. Global time
**Adds:** the leader puts its clock in REF. Followers set their clock from it plus the CAN delay, and correct for drift between REFs. `global_gettime()`, a PPS pin pulsing every global second, and a console line per pulse with the drift info.
**Verify:** scope the PPS pins on all nodes. Build it in three stages and check each:
- local clock only: the edges drift apart
- with the offset from REF: the edges line up
- with drift correction: the edges stay together between REFs

### 8. Blinkers and failure mode on global time
**Adds:** blinkers time their flashes from `global_gettime()`. When the cockpit fails, the car enters failure mode: error state, motor braked, hazards at 4 Hz. A new leader keeps time sync going.
**Verify:** scope FL and RL: within 1 ms over ten edges. Power off the Pi: error within 100 ms, an STM32 becomes leader within 1 s, and the hazards stay in step.

### 9. Pi rejoin, demo, report
**Adds:** when the Pi powers back on, it rejoins as a follower and replays the events it missed. Then a dry run of the full checkoff, and the report.
**Verify:** the Pi shows the missed events once, in order. All 10 checkoff steps pass.

Build (repo root, west venv active):
```
west build -p always -b nucleo_f401re -d build/n3 . -- -DLAB5=ON -DNODE_ID=3 \
  -DCONF_FILE=prj_lab3.conf -DDTC_OVERLAY_FILE=boards/nucleo_f401re_lab3.overlay
```

Flash (`--serial` is silently ignored, so pass the ST-Link serial to OpenOCD):
```
west flash -d build/n3 --runner openocd \
  --cmd-pre-init "adapter serial <ST-Link serial>" \
  --cmd-pre-init "reset_config srst_only srst_nogate connect_assert_srst"
```

Console: `picocom -b 115200 /dev/ttyACMx` (quit: Ctrl-A, Ctrl-X).
