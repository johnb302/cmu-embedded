# Lab 5 Plan: Time Synchronization and Consensus

Due 2026-11-18. Steps 2–6 are Raft, steps 7–8 are time sync. Time sync only needs step 3's heartbeat, so the two tracks can split after step 3.

Step 3 was done before step 2 (it only needs zeros from the log).

Where things live:
- `shared/`: portable C (no Zephyr or Linux headers) built into all three nodes: `netstate`, `raft`, `raft_config.h`, and the step 2 log. `make -C shared/test` runs its checks on the Mac.
- `src_lab5/`: the STM32 side (`raft_node.c` is the RAFT thread and its CAN frames; `net_shell.c` is the `net` commands).
- `cockpit/`: the Pi (`cockpit.c`), built on the Pi with `shared/` linked in.

## Steps

### 1. Network state machine — DONE
**Adds:** the graph of which nodes and CAN links are up. Log events are applied to it, and `getNetwork()` reads it back.
**Verify:** boot prints `netstate selftest: 11/11 PASS`. In the shell, `net node up 3`, `net link down 3 A`, `net show`.

### 2. Config and replicated log — DONE
**Adds:** the shared Raft config (N, default leader, heartbeat period, timeouts, debug). A log of `{term, index, event}` entries that update the network state only once committed.
**Verify:** `make -C shared/test`: appending doesn't change the graph, committing applies each entry once and in order, and the empty APPEND carries the leader's log position. (The shell commands moved to step 4.)
**Result (2026-11-15):**
- `shared/raft_config.h` (the Part 2 keys)
- 1-byte event codes in `netstate`: `net_event_encode/decode`, 0 = no event. High hex digit = type, low = node (+4 on bus B), e.g. `0x56` = CANLINK_UP N2 B.
- `shared/raft_log.c/.h`: `term[]` and `event[]` arrays, indexes from 1. `raft_log_commit` applies `lastApplied+1 … commitIndex` through `netstate_apply`; `raft_log_truncate` is there for step 4. Host tests pass.
- `send_empty_append()` fills prevLogIndex, prevLogTerm (the leader's last entry) and leaderCommit from the log. The bus looks the same until the leader's log has entries.
- RAM (of 96 KB): the linker keeps only the arrays something reads. Now, with `term[]` only: N3 56%, N2 58%. Once step 4 reads `event[]` (+30 KB): N3 87% (measured), N2 about 88%, leaving about 11 KB.

### 3. Raft over CAN with a fixed leader — DONE
**Adds:** Raft CAN frames. Nodes boot with the cockpit as leader in term 1, no election. The leader sends REF every 20 ms and an empty APPEND every 100 ms. LEADER and SYNC LEDs.
**Verify:** `candump -c any` shows REF every 20 ms, APPEND every 100 ms, no REQ_VOTE. The LEADER LED is lit on the Pi only.
**Result (2026-11-12, both buses):**
- REF `0x001` every 20.00 ms (19.5–20.5), APPEND `0x041` every 100.0 ms, no REQ_VOTE or other Raft frames
- `raft status` on N2 and N3: `follower | term 1 | leader N1 | SYNC on | leader heard <20 ms ago`
- LEADER pin high on the Pi only; SYNC high on all three
- Pi stopped: SYNC off on both followers, term stays 1. Pi restarted: SYNC back on within ~20 ms.

Frames (`shared/raft.h`): one CAN ID per type per sender (REF `0x000`+node, APPEND `0x040`+node, …). Byte 0 is the term, so Raft frames skip the Lab 3 seq dedup. REF: b1 cycle counter, b2–7 master time in µs. **Confirm the IDs and layouts with the TA**: the handout doesn't define them.
**Pins:** STM32 LEADER PA0 (A0), SYNC PA8 (D7), REF_RX PC8 (CN10-2). Pi LEADER BCM 16 (pin 36), SYNC BCM 12 (pin 32), REF_RX BCM 27 (pin 13). LEDs still need wiring.
**Later:** REF_RX flips on both bus copies of a REF, so it pulses instead of toggling. In step 7, act only on the first copy (match the cycle counter).

### 4. Log replication — IN PROGRESS
**Adds:** an event sent to the leader is replicated to followers and committed once a majority has it. Events sent to a follower get redirected to the leader.
**Verify:** submit an event from any node's shell. `log show` and `net show` match on every node.
**Done (2026-11-15), in `shared/`, tested on the Mac:**
- Frames (`raft.h`): APPEND carries one entry. APPEND_RESP `0x048`+node: success + the index it matches through, or failure + how far it can match. EVENT `0x060`+node: sequence number + event code. Confirm with the TA along with the rest.
- `raft_submit(event)`: on the leader, appends; on a follower, sends an EVENT that only the leader acts on (the redirect).
- Leader: nextIndex/matchIndex per follower, one entry in flight to each (resent every 100 ms until acknowledged). Commits an entry from its own term once a majority holds it, then sends an empty APPEND right away so followers apply it too.
- Follower: Figure 2's AppendEntries receiver: check prev, drop conflicting uncommitted entries, append, commit up to min(leaderCommit, last matching index). One reply per APPEND.
- Each APPEND and EVENT is acted on once, not once per bus: each sender's last 8 frames are remembered, because back-to-back frames can arrive A1 A2 B1 B2 (the sim caught a single-slot version appending duplicates).
- `shared/test/sim.c`: all three nodes in one process on two simulated buses. Covers submits on the leader and followers, a node off and back, a reboot (empty log) catching up, and no commit without a majority.
- RAM with the whole log linked: N3 88%, N2 89% (10.4 KB free).

**Left:**
- Shell (moved from step 2): `log show`; `net node|link|cockpit` calls `raft_submit` instead of applying directly, holding the Raft lock (`net_shell.c` doesn't take it today). The Pi needs the same on stdin. `raft config` is optional: the config is one header, identical on every node by construction.
- Board check: the verify line above, plus `candump` showing APPEND_RESP and EVENT frames.

**Known limit (until step 9):** logs live in RAM. A rebooted follower catches up on its own (sim 4.7). A rebooted Pi comes back as leader of term 1 with an empty log while the followers keep theirs, so its new entry 1 has the same index and term as their old entry 1 and they'd keep the old one. After restarting the Pi, restart both STM32s too.

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

Or with STM32CubeProgrammer. `mode=UR` (connect under reset) is required; without it, flashing fails with "Unable to get core ID":
```
STM32_Programmer_CLI -c port=SWD sn=<ST-Link serial> mode=UR reset=HWrst -w build/n3/zephyr/zephyr.hex -v -hardRst
```

Console: `picocom -b 115200 /dev/ttyACMx` (quit: Ctrl-A, Ctrl-X). On a Mac: `screen /dev/cu.usbmodemXXXX 115200` (quit: Ctrl-A, K, y).

Pi (from the Mac, repo root): `cockpit/deploy.sh` copies `cockpit/` and `shared/` to `~/lab5` on the Pi and builds there. Then on the Pi, `~/lab5/cockpit/run_cockpit.sh` starts it; it also stops the Lab 3 cockpit if that's running. Log: `~/lab5/cockpit/cockpit.log`.

Shared code on the Mac, no board: `make -C shared/test` runs `host_test` (one node, frames by hand) and then `sim` (three nodes on two simulated buses). Both should end `ALL PASS`.
