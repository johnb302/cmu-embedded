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

### 4. Log replication — DONE
**Adds:** an event sent to the leader is replicated to followers and committed once a majority has it. Events sent to a follower get redirected to the leader.
**Verify:** submit an event from any node's console, e.g. `net link down 3 A`, and it prints `ok: committed as entry N`. `raft log` and `net show` match on every node.
**Done (2026-11-15), in `shared/`, tested on the Mac:**
- Frames (`raft.h`): APPEND carries one entry. APPEND_RESP `0x048`+node: success + the index it matches through, or failure + how far it can match. EVENT `0x060`+node: sequence number + event code. Confirm with the TA along with the rest.
- `raft_submit(event)`: on the leader, appends; on a follower, sends an EVENT that only the leader acts on (the redirect).
- Leader: nextIndex/matchIndex per follower, one entry in flight to each (resent every 100 ms until acknowledged). Commits an entry from its own term once a majority holds it, then sends an empty APPEND right away so followers apply it too.
- Follower: Figure 2's AppendEntries receiver: check prev, drop conflicting uncommitted entries, append, commit up to min(leaderCommit, last matching index). One reply per APPEND.
- Each APPEND and EVENT is acted on once, not once per bus: each sender's last 8 frames are remembered, because back-to-back frames can arrive A1 A2 B1 B2 (the sim caught a single-slot version appending duplicates).
- `shared/test/sim.c`: all three nodes in one process on two simulated buses. Covers submits on the leader and followers, a node off and back, a reboot (empty log) catching up, and no commit without a majority.
- RAM with the whole log linked: N3 88%, N2 89% (10.4 KB free).
- Consoles: the same commands on the STM32 shell and the Pi's stdin.
  - `net node|link|cockpit ...` submits through `raft_submit` with the Raft lock held, waits up to 500 ms for the entry to commit on that node (Figure 2's "ok"), then prints the graph. Nothing applies an event directly any more, so a board with no leader up prints `not committed after 500 ms`.
  - `net show`, `raft status`, `raft log [count|all]` (the newest 10 by default), `raft config` (the Part 2 keys). It's `raft log`, not `log show`: Zephyr's own `log` command would clash if logging is ever turned on.
  - The parser, the "ok" lookup and the formats are in `shared/` (host test 4.9; sim 4.4 checks the "ok" points at the same entry on every node).
  - Pi: `run_cockpit.sh` feeds stdin from `cmd.fifo`, like the fake proxy's `fp.fifo`.

**Board check (2026-11-15, all three nodes):**
- Submits from every node commit. N2 and N3 send theirs to the leader; the Pi appends its own. Each prints `ok: committed as entry N` and the new graph. After 18 entries, `raft log` and `net show` are identical on N1, N2 and N3.
- `candump`: EVENT `0x062` → APPEND `0x041` carrying the entry → APPEND_RESP `0x04A`/`0x04B` → empty APPEND with the new leaderCommit. 8.4 ms from the EVENT to the commit going out.
- A reset follower (N3, empty log) answers "no entry 9 here", then the leader replays entries 1–9 one per round trip: 90 ms in all, about 10 ms per entry.
- `raft config` prints the same line on every node. The Pi console works through `cmd.fifo`.
- Hardware: N3's ST-Link stopped passing typed input three times, each right after a submit typed on N3. Board output and Raft kept working, and replugging its USB fixed it. The same submits typed on N2 never did it, so it's N3's ST-Link, cable or port, not the firmware. Before the demo, try another cable or port, or update the ST-Link firmware (V2J33M25).

**Known limit (until step 9):** logs live in RAM. A rebooted follower catches up on its own (sim 4.7). Since step 5, a restarted Pi also rejoins as a follower and gets the log replayed (sim 5.5), because the STM32s have elected a leader in term 2 by the time it's back. The old problem remains only if the Pi restarts while just one STM32 is up: with no majority there's no election, the term stays 1, and the Pi's empty term-1 log is taken as current. In that case, restart the STM32s too.

### 5. Leader election — DONE except the jumper pulls (board check below)
**Adds:** when the leader's heartbeats stop, followers time out at random offsets, vote, and elect a new leader in a new term. A "random leader" boot mode with no preset leader.
**Verify:** power off the leader. A new LEADER LED lights within 1 s, the term goes up once, and logs still agree. Pulling a link must not trigger an election.
**Done (2026-11-16), in `shared/`, tested on the Mac:**
- Election timeout = 60 ms + U(0, 39) (Part 2). The U(...) is a hash of (seed, node, term): a fresh draw each term, but the same on every run, so the seed fixes which node times out first in each term. The F401 has no hardware random number generator, and checkoff step 8 asks for "the seed that makes the Pi win".
- **Pre-vote** (Raft thesis 9.6). A timed-out node first asks "would you vote for me in term+1?" without changing its own term. Nodes that heard a leader in the last 60 ms say no. Only with a majority of yeses does it take the term and ask for real votes. So a node that has lost its own link to the leader (checkoff steps 4, 5, 9, 10) never raises the term. With plain Raft, an isolated node raises its term every timeout, and when it reconnects it deposes the working leader, which fails checkoff step 10.
- RequestVote follows Figure 2: one vote per term, and only to a candidate whose log is at least as up to date. Votes are counted per voter in a bitmask, so the copy from the second bus counts once. A stale leader or candidate gets a "no" carrying the current term and steps down. For a stale APPEND, that "no" is an APPEND_RESP saying "I match through my last entry", which is true, so this term's leader, which also hears it, loses nothing.
- Frames (`raft.h`): REQ_VOTE `0x050`+node: term, pre-vote flag, last log index, last log term. VOTE `0x058`+node: term, granted, candidate, pre-vote flag. Confirm with the TA along with the rest.
- **Boot with a preset leader** (Part 2: "no election is held at boot"): followers don't start elections until they've heard a leader once, however long the Pi takes to boot. After that, its silence starts an election. One gap: if both STM32s reboot while the Pi is off, no election ever happens.
- **LEADER=random**: every node boots in term 0 with no leader, and timers run from boot. The STM32s are up ~30 s before the cockpit, so one of them wins ("An STM32 is leader"), and the Pi joins as a follower.
- **Seed 9 makes the Pi win.** In terms 0–2 the Pi times out at least 10 ms before both STM32s. So when that first STM32 leader is powered off, the Pi beats the other STM32 (sim `random-seed9`). Seed 1 (the default) ties N2 and N3 in term 0, so a simultaneous STM32 boot does a split vote first (sim R.2: elected in 145 ms).
- `raft status` is the same on every console: `follower | term 2 | leader N2 | SYNC on | heard 4 ms ago | election timeout 66 ms`. `raft config` ends with `seed=N`. Debug lines: `election timeout -> pre-candidate`, `pre-vote won -> candidate`, `vote for N2 in term 2`, `won the vote -> leader`. Pre-vote retries aren't printed (a cut-off node retries ~12 times/s).
- Sim (`make -C shared/test`), with per-node jumpers now:
  - one jumper out: no election
  - N1 off B and N3 off A: N2 turns down N3's pre-votes, term stays 1
  - a follower isolated: term stays 1
  - Pi off: N2 leads term 2 after 62 ms
  - Pi back with an empty log: steps down, gets all 8 entries replayed
  - STM32 leader isolated: the other two elect term 3 in 65 ms, and it steps down when reconnected
  - one node alone: term stays put
  - random mode, both seeds
- Build: `-DRAFT_LEADER=cockpit|steering|drivetrain|random -DRAFT_SEED=n` on the STM32s, and `RAFT_LEADER=... RAFT_SEED=... cockpit/deploy.sh` on the Pi. A typo fails the build. **All three nodes need the same values.** A follower that hears two leaders in one term prints `rebuild all nodes with one config` and stays with the first.
- RAM: N3 87.7%, N2 89.3%, about the same as step 4.

**Board check (2026-11-16, all three nodes).** The Pi was stopped with `pkill -x cockpit`, and an STM32 was "powered off" by halting its core through OpenOCD. Times are from `candump` on the Pi.
- **Cold start, default mode.** The STM32s wait (`no election until the leader is heard`) until the cockpit starts, then follow it in term 1. In 5 s on the bus: REF `0x001` at 50/s per bus, APPEND `0x041` at 10/s, no `0x05x`. Submits from all three nodes commit (entries 1–3).
- **Pi down.** N2 timed out and won term 2: pre-vote `0x052` at 65.4 ms after the Pi's last REF, N3's yes, vote request at 71.2 ms, N3's vote at 71.8 ms, N2's first REF at 76.5 ms. N2's REF period is then 19.4–20.6 ms.
- **Pi back.** It boots as leader of term 1. N3's stale-term answer makes it a follower in term 2, and N2 replays entries 1–3 to it. The logs match, and LEADER stays on N2.
- **STM32 leader down** (N2 halted). The Pi (66 ms) beat N3 (86 ms): N3 voted for it, and the Pi's first REF came 70.5 ms after N2's last. Released, N2 rebooted and caught up. Entries 4 (from N2) and 5 (from the Pi) then committed in term 3, and N1's and N2's logs are identical. A rebooted follower (N3) also caught up from an empty log.
- **LEADER=random, seed 9** (all three rebuilt). The STM32s cold-started and N3 won term 1 with N2's vote. The cockpit started and joined N3 as a follower. N3 halted: the Pi won term 2 in 71 ms. N3 back: it rejoined and caught up. Pi stopped: N2 won term 3, and N3, which timed out 5 ms later, voted for it instead of splitting. Then the default builds were restored on all three.
- **Debug lines block the RAFT thread** on the STM32: printf waits on the 115200-baud console, about 5 ms per line. Each frame now goes out before its debug line, which cut the first election from 90 ms to 76 ms. The remaining ~10 ms is a print holding up the next frame. For step 7: stamp the REF receive time in `raft_node_post()` (the RX callback), not in the RAFT thread.
- **N3's console input died again** right after typing a submit on N3. A board reset through OpenOCD didn't bring it back, so it's the ST-Link/USB side, as step 4 concluded. Replugging N3's USB fixes it.

**Still to check on the boards (needs hands on the jumpers; all covered in the sim):**
1. Pull N3's A jumper: no `0x05x` frames, term unchanged everywhere.
2. Pull N1 off B and N3 off A:
   - N3 shows `pre-candidate` and sends `0x053` (b1 = 1) every 60–99 ms
   - N2 answers `0x05A` with b1 = 0 (no)
   - term unchanged on all three
   - Put the jumpers back: N3 follows again and catches up.
3. Pull both jumpers of the STM32 leader: the Pi and the other STM32 elect, term +1. Reconnect: one term everywhere, one LEADER LED.

**Checkoff step 8 plan:**
1. Build all three with `RAFT_LEADER=random`: an STM32 leads. Power it off: a new leader, no failure mode.
2. Rebuild with `RAFT_SEED=9`. Power off the first STM32 leader: the Pi wins.
3. Power that STM32 back on, then do checkoff step 6 again.
4. Rebuild with `cockpit` afterwards.

**Ask the TA:** Is "the seed that makes the Pi win" about the election after an STM32 leader dies? At boot the Pi can't win, because the STM32s finish electing before the cockpit starts.

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
For checkoff step 8, add `-DRAFT_LEADER=random` (and `-DRAFT_SEED=9` for the Pi to win), and deploy the Pi with the same values: `RAFT_LEADER=random RAFT_SEED=9 cockpit/deploy.sh`. `raft config` on each node shows what it was built with.

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

Pi (from the Mac, repo root): `cockpit/deploy.sh` copies `cockpit/` and `shared/` to `~/lab5` on the Pi and builds there. Then on the Pi, `~/lab5/cockpit/run_cockpit.sh` starts it; it also stops the Lab 3 cockpit if that's running. Log: `~/lab5/cockpit/cockpit.log`. Console: `echo "raft log" > ~/lab5/cockpit/cmd.fifo` (`h` lists the commands), replies in the log. Or stop it (`pkill -x cockpit`) and run `./cockpit` in the foreground to type them.

Shared code on the Mac, no board: `make -C shared/test` runs `host_test` (one node, frames by hand) and then `sim` three times (three nodes on two simulated buses, with jumpers that can be pulled): with the Pi as preset leader, `random`, and `random-seed9`. All four should end `ALL PASS`.
