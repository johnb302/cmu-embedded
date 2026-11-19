# Lab 5 Plan: Time Synchronization and Consensus

Due 2026-11-18. Steps 2–6 are Raft, steps 7–8 are time sync. Time sync only needs step 3's heartbeat, so the two tracks can split after step 3.

Step 3 was done before step 2 (it only needs zeros from the log).

Where things live:
- `shared/`: portable C (no Zephyr or Linux headers) built into all three nodes: `netstate`, `raft`, `raft_config.h`, the step 2 log, `netwatch` (link watching and routing, step 6) and `timesync` (global time, step 7). `make -C shared/test` runs its checks on the Mac.
- `src_lab5/`: the STM32 side (`raft_node.c` is the RAFT thread and its CAN frames; `net_shell.c` is the `net` commands; `time_node.c` is global time, PPS and `time`; `relay.c` and `blink.c`).
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
**Pins:** STM32 LEADER PA0 (A0), SYNC PA8 (D7), REF_RX PC8 (CN10-2), PPS PC9 (CN10-1, step 7). Pi LEADER BCM 16 (pin 36), SYNC BCM 12 (pin 32), REF_RX BCM 27 (pin 13), PPS BCM 22 (pin 15), HZ BCM 23 (pin 16). LEDs still need wiring.
REF_RX toggles once per REF since step 7: on its first copy (bus A, bus B or relayed), matched by sender and cycle counter.

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

### 5. Leader election — DONE
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

**Jumper checks (2026-11-16):** done during the step 6 jumper tests and the checkoff walk-through (step 9 below):
- single jumpers: no election
- the hop: no term change
- the leader isolated: one election

**Checkoff step 8 plan:**
1. Build all three with `RAFT_LEADER=random`: an STM32 leads. Power it off: a new leader, no failure mode.
2. Rebuild with `RAFT_SEED=9`. Power off the first STM32 leader: the Pi wins.
3. Power that STM32 back on, then do checkoff step 6 again.
4. Rebuild with `cockpit` afterwards.

**Ask the TA:** Is "the seed that makes the Pi win" about the election after an STM32 leader dies? At boot the Pi can't win, because the STM32s finish electing before the cockpit starts.

### 6. Link watching and routing — DONE
**Adds:** each node reports node and link failures and recoveries to the leader as log events. Routing (direct bus or relay) is built from `getNetwork()` instead of lab3's heartbeat ages.
**Verify:** pull an A jumper and you get one new log entry, at the same index on every node, with the term unchanged. Pull N1 off B and N3 off A, and steering, blinking and braking still work through the Pi relay.
**Done (2026-11-16):** `shared/netwatch.c` (the Part 1 Raft client), tested in `host_test` 6.1-6.3 and `sim watch`.
- **Link watching.** Every 10 ms each node turns heartbeat ages into facts it can vouch for, compares them with the committed graph, and submits any difference with `raft_submit_change()` (retried every 300 ms while the graph still disagrees). The facts:
  - its own link on a bus is up if it hears another node's own (not relayed) heartbeat there
  - another node is up if heard at all, and down only if this node sees both buses and hears it on neither
  - another node's link: up iff its own heartbeat is heard on that bus
  - the cockpit: OK while N1 is up
- **One entry per change.** Every node reports the same change. The leader logs an event only if it changes its projected graph (committed entries plus the uncommitted tail). EVENT b3 = 1 marks such reports; console submits are never dropped.
- **Routing table** (`route_build`): forward S's frames from bus X to bus Y when the graph says this node is on both buses, S is on X and not on Y, and a third node is on Y and not on X. With three nodes at most one node qualifies. Part 5.2: until the log has a second link failure, the same rule also runs on direct heartbeat ages (`route_relay_observed`, sender stale after 30 ms, before the 50 ms timeout).
- **Relays.** Every node relays now: N2 and N3 (`relay.c`, a thread each) and the Pi (`relay_maybe()` in its RX thread, for checkoff step 9). Raft frames are relayed too, with bit 7 of b0 set. A relayed REF gets the first hop and its time in the relay added to its master time (a transparent clock).
- **Sim `watch`** (heartbeats modeled from the jumpers, relaying by each node's table):
  - cold start: exactly 10 entries (3 NODE_UP, 6 CANLINK_UP, COCKPIT_OK) on all three
  - one jumper: one entry each way
  - N1 off B, N3 off A: two entries; N3 gets them through N2's relay and keeps SYNC; an EVENT from N2 reaches N3's log
  - the STM32 leader cut off from the other STM32: the Pi bridges, term unchanged
  - a follower isolated: NODE_DOWN once, then NODE_UP plus its 2 links
- **Board (2026-11-16):** cold start logged exactly those 10 entries, the same on all three. No relaying with all links up (`relay none` in `raft status`).
- **Four rules the jumper pulls taught** (`netwatch.h`, each with a host test that replays the board case):
  - **Fresh proof** (`NETWATCH_FRESH_MS` 25): heartbeats age out at different phases, so for up to 20 ms after a node's own link dies one peer looks gone while another still looks heard. N3 blamed N2's A when its own A jumper came out. Blaming a peer's link now needs a third node heard on that bus within 25 ms.
  - **Settle** (`NETWATCH_SETTLE_MS` 50): the same race on reconnection. Blame nobody on a bus until our own link there has been up 50 ms.
  - **Hold** (`NETWATCH_DOWN_HOLD_MS` 1000): pulling N3's B jumper took bus B down for every node for under a second, and everyone logged their own B link down and up. Down is now reported only after holding 1 s. Up is immediate; relaying and failure mode don't wait for the log.
  - **No self-vouching:** after its B jumper went back in, N2's B transmitter stayed dead for seconds while its receiver worked. N2's "my B is up" fought N3's "N2's B is down", and the log flipped every 1.3 s. A node now reports its own link only as down; peers report it up.
- **Board results (all with the final rules):**
  - each jumper out and back: one entry each way, same index everywhere, term unchanged, including on the leader
  - N1 off B, N3 off A: two entries, N2 relays both ways (170 frames/s), N3 synced through the relay, and an EVENT from N2 reaches N3's log
  - N2 leading, N2 off B, N3 off A (checkoff step 9): the Pi bridges them; N3's per-REF corrections stay within +6 µs

### 7. Global time — DONE except the scope
**Adds:** the leader puts its clock in REF. Followers set their clock from it plus the CAN delay, and correct for drift between REFs. `global_gettime()`, a PPS pin pulsing every global second, and a console line per pulse with the drift info.
**Verify:** scope the PPS pins on all nodes. Build it in three stages and check each:
- local clock only: the edges drift apart
- with the offset from REF: the edges line up
- with drift correction: the edges stay together between REFs
**Done (2026-11-16):** `shared/timesync.c` (host test 7.1-7.3), `src_lab5/time_node.c`, and the same in `cockpit.c`.
- **Mapping.** global(t) = base_global + (t - base_local)(1 + ppb/1e9). The local clock is the 84 MHz cycle counter on the STM32s (`local_us()`) and CLOCK_MONOTONIC on the Pi.
  - At boot the leader's mapping is the identity.
  - A node that wins an election keeps its mapping, so global time has no step at a handover.
  - Followers re-anchor at every REF: follower_clock = L + delay, where delay = sender TX latency + 236 µs on the wire + receiver RX latency (`timesync.h`, adjustable with `time delay`).
- **Each correction** is applied half at a time (the clock slews). A REF more than 150 µs late is ignored (it waited behind another frame), unless three in a row agree. Beyond 2 ms: a hard step.
- **Drift:** (L2 - L1)/(F2 - F1) over >= 2 s windows, smoothed 1/4 per window, applied between REFs.
- **Stamps.**
  - Send: each REF is stamped per bus just before that copy is sent.
  - Receive, STM32: in the CAN RX callback.
  - Receive, Pi: the kernel's receive timestamp (SO_TIMESTAMPNS).
- **CLOCK_REALTIME** on the STM32s is set from global time at every PPS edge (`sys_clock_settime`). It has the 100 µs tick, so PPS, REF and the blinkers use `global_gettime()`. The Pi doesn't set its clock (no root, and NTP owns it).
- **PPS:** PC9 on the STM32s, BCM 22 on the Pi. Each sleeps until about 200 µs before the whole global second, then spins to the edge.
- **One console line per edge:** `PPS N2 | global 5797.000003 s | local 19.740884 s | drift ratio 1.000041373 (+41.373 ppm) | since REF +0 us | last step +52 us | synced to N1`. `time status` shows the sync counters.
- **Board (2026-11-16):**
  - STM32 PPS edges land 3-5 µs after each global second; the Pi's about 20 µs after.
  - The STM32s run about +40 ppm against the Pi.
  - Per-REF corrections from the Pi: up to ±100 µs. N2's and N3's are nearly identical, since they hear the same frame. From an STM32 master: ±10 µs.
  - About 12 % of the Pi's REFs are ignored as late.

- **From 743 PPS lines (2026-11-16):**
  - drift median +39.7 ppm
  - per-REF correction: median 13 µs (90 %: 94 µs) with the Pi as master, 4 µs (90 %: 10 µs) with an STM32
  - PPS edge vs. its global second: median 4 µs, 90 % within 30 µs, about 1 % over 260 µs (the CAN driver thread preempting the PPS spin)
- **Handover printout:** reports only a REF that was used. A new leader's first REF usually waits behind the election's last frames and is discarded as late; it used to print as a fake -354 µs handover.
- **FL/RL during the hop:** the scope showed the FL/RL gap shrink while N3 synced through N2's relay. N3 then shares N2's receive error, leaving only the accurate STM32-to-STM32 hop. The normal gap is the two STM32s receiving the Pi's REF differently; trim it with `time delay 1 <us>` on N3 if wanted.

**Still to check:** scope the PPS pins (PC9 on N2 and N3, BCM 22 on the Pi), and tune `time delay` if the Pi's edge is offset from the STM32s'.

### 8. Blinkers and failure mode on global time — DONE except the scope
**Adds:** blinkers time their flashes from `global_gettime()`. When the cockpit fails, the car enters failure mode: error state, motor braked, hazards at 4 Hz. A new leader keeps time sync going.
**Verify:** scope FL and RL: within 1 ms over ten edges. Power off the Pi: error within 100 ms, an STM32 becomes leader within 1 s, and the hazards stay in step.
**Done (2026-11-16):**
- **Blinkers.** The Pi's BLINK frame now only says which lamps blink, sent on every change and every 500 ms. Each STM32 switches its lamps at multiples of the half period in global time, so the edges are the same instants everywhere; the last ~200 µs before an edge is spun.
- **Rates:** turn signals 1 Hz (Lab 3), hazards 2 Hz, failure mode 4 Hz (`config.h`).
- **Failure mode:** no N1 heartbeat on either bus (relayed copies count). The STM32s blink hazards at 4 Hz on their own. ERROR and motor braking are Lab 3's existing 60 ms heartbeat loss.
- **HZ on the Pi** (BCM 23) shows the hazard lamp on the same edges.
- **`blink`** in the STM32 shell shows the state and the last 8 edges in global time.
- **Board (2026-11-16):**
  - Cockpit up without the proxy (error state): hazards at a 250 ms half period, edges at global .000010-.000041 s.
  - Cockpit stopped:
    - within 1 s, a 125 ms half period on N2 (`failure mode: cockpit lost`)
    - N2 leads term 2; NODE_DOWN N1 and COCKPIT_ERROR logged once each; `holding 2 for N1`
    - N3 prints `master N1 -> N2, clocks -81 us apart at its first REF`, applied half at a time, so the largest jump at the lamps is ~40 µs, under the 100 µs limit
- **Ask the TA:** "hazards at 4 Hz ... return to 2 Hz", taken literally (Lab 3 hazards were 1 Hz). The rates are the three constants in `config.h` (and `BLINK_HAZARD_HALF_MS` in `cockpit.c`). Also: what "HZ" is in checkoff step 1. Here it's the Pi's hazard lamp output.

**Still to check:** scope FL (N2 PC11) and RL (N3 PC11) over ten edges, then power off the Pi and watch the handover.

### 9. Pi rejoin, demo, report — DONE except the LEDs and scope checks (list below)
**Adds:** when the Pi powers back on, it rejoins as a follower and replays the events it missed. Then a dry run of the full checkoff, and the report.
**Verify:** the Pi shows the missed events once, in order. All 10 checkoff steps pass.
**Done (2026-11-16):**
- **Listen before leading.** The preset leader listens for `RAFT_CLAIM_LISTEN_MS` (198 ms, longer than a lone node's pre-vote period) before taking term 1. If it hears any Raft frame, it rejoins as a follower instead. This also closes step 4's known limit: a rebooted Pi can no longer lead term 1 with an empty log.
- **Replay marking.** A follower marks entries that reach it already committed as replayed (`raft_replayed_through()`). The Pi's display prints every committed entry once, in order: `event    11  term 2   NODE_DOWN N1  [replayed]`, then `[live]`.
- **Board (2026-11-16):** the Pi restarted under N2's leadership. It printed `cluster heard -> follower`, replayed 12 entries marked `[replayed]`, synced its clock to N2, and LEADER stayed on N2.
- **Report:** `lab5_report/lab5_report.tex` (and `.pdf`); captures in `lab5_report/captures/`, console logs in `lab5_report/logs/`, one page per section. All measured numbers come from the board consoles. The handout asks for drift and jitter from the PPS printouts, not scope captures, so nothing is left to fill in.
- **Not done: the extra credit** (cloud VM with Redis and a GUI). It needs a cloud account and a deployed service.

**Checkoff walk-through on the boards (2026-11-16), steps 1-10 in order: all pass in software.**

| Step | Result |
|---|---|
| 1 | Cold start, proxy stopped: all in ERROR, hazards in step, Pi leads, SYNC on (from the consoles; the LEDs aren't wired). Proxy started: all OK. |
| 2 | `candump`: every catalog ID; REF every 19.0-21.0 ms; APPEND every 99-101 ms; no `0x05x`. FAULT appears only on a fault (step 10). |
| 3 | Left, right, cancel on both nodes; FL/RL edges 1-3 µs apart in global time. |
| 4 | One entry per change, term unchanged. The jumpers pulled were not the ones intended: label them. |
| 5 | Two entries, N2 relays, wheel works, term unchanged. EVENT through the relay: shown in jumper test 2. |
| 6 | STM32 leads in 78-106 ms; NODE_DOWN N1 and COCKPIT_ERROR once each; 4 Hz on both; handover -57 µs. |
| 7 | Pi rejoins as a follower, shows missed entries once, in order, `[replayed]`. |
| 8 | Seed 1: an STM32 wins at boot; halted leader replaced in 80 ms, no failure mode. Seed 9: the Pi wins (72 ms). Step 6 again: 106 ms, handover -54 µs. |
| 9 | Jumper test 3. |
| 10 | Follower isolated: term unchanged. Leader (Pi) isolated: term +1, exactly once. Test button: all ERROR, cleared by double press. |

**Fixed during the walk-through:**
- `cockpit.c`:
  - the turn buttons are 5 = left, 4 = right with the proxy's IOKit backend (they were mirrored)
  - the brake rest point is learned (this pedal rests at 30582, not 0, and read as 93 % braking)
- `run_cockpit.sh` keeps the previous log as `cockpit.prev.log`.

**Before the demo:**
- [ ] Wire the six LEADER/SYNC LEDs (pins under step 3).
- [ ] Label every jumper by node and bus.
- [ ] Check N3's bus B connector and terminator: pulling N3's B jumper takes bus B down for everyone, once for over 1 s.
- [ ] Scope: PPS spread, FL/RL over ten edges, no step at the handover (steps 6 and 9).
- [ ] Flash the final images: `build/n2`, `build/n3` (default); `build/n2r`, `build/n3r` (random, seed 9); `build/n2r1`, `build/n3r1` (random, seed 1). They're built but not flashed, and only the handover printout changed.

**Demo commands:**
- **Wheel proxy** (Mac, proxy folder, `.venv` active):
  `SDL_JOYSTICK_MFI=0 python3 mac_proxy.py --remote 172.26.58.163 --steer 0 --throttle 3 --brake 2 --interval-ms 50`
  Without `SDL_JOYSTICK_MFI=0`, macOS's game-controller backend hides the wheel ("No joystick found").
- **If `team5.local` stops resolving**, use the Pi's IP. ssh only knows the key under the name, so add `-o HostKeyAlias=team5.local`.
- **Step 8:** flash the random images, then rebuild the Pi with the same values (`make -B -C lab5/cockpit RAFT_LEADER=random RAFT_SEED=9` on the Pi, or via `deploy.sh`). Afterwards, flash `build/n2`, `build/n3` and rebuild the Pi without the variables.
- **Simulating power loss:** `pkill -x cockpit` on the Pi is the same as powering it off, on the bus. Halting an STM32's core through OpenOCD (`reset halt`, then `reset run`) is the same as powering it off and on.

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
