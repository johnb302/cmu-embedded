# Lab 5 report assets

Hardware evidence for `lab5_report.tex`, captured 2026-11-19 on the real boards.
Handout: *18-449/649 Lab 5 - Time and Consensus* (v1.0). Nodes: N1 = cockpit (Pi), N2 = steering, N3 = drivetrain.

## Map: handout → report section → evidence

| Handout | Graded item (Deliverables) | Report | Evidence |
|---|---|---|---|
| Part 6 (6.1–6.4) | Time drift adjustment, 13 pts | §1 | `captures/pps_stats.txt`, `logs/*.log` (PPS lines), `captures/s3_blink_*` |
| Parts 2–4, Part 7 | Raft state machine, 13 pts | §2 | `captures/s1_*`, `s2_*`, `s3_*` |
| Parts 1, 5 (5.1, 5.2) | Raft client and routing, 10 pts | §3 | `captures/hop_*` |
| — | GitHub link, 4 pts | header | — |

## Files

### Part 6: time sync (§1)
- `captures/pps_stats.txt`: the §1 results table. Regenerate with
  `tools/pps_stats.py lab5_report/logs/n2.log lab5_report/logs/n3.log lab5_report/logs/pi.log`.
  Rows are (node, master). `drift` = the drift ratio (6.2) in ppm, `|step|` = the per-REF offset correction, `edge` = the PPS edge's offset from its whole global second (6.4).
- `logs/n2.log`, `logs/n3.log`: the full STM32 consoles, with host timestamps. The 6.4 PPS lines read like:
  `PPS N2 | global 563.000005 s | local 576.492182 s | drift ratio 1.000063657 (+63.657 ppm) | since REF +1 us | last step -21 us | synced to N1`
  - The Pi (N1) is master until 18:27, N3 from 18:27:37, N2 from 18:30:44.
- `logs/pi.log`: the Pi console in default mode (Pi = master, `PPS N1 ... master N1`).
- `logs/pi_random.log`: the Pi console in random mode, as a follower. It has no PPS lines (see Caveats).
- `captures/s3_blink_before_*.txt`, `s3_blink_after_*.txt`: the last 8 blinker edges on N2 (front) and N3 (rear), in global time, before and after the leader handover. The two boards match within 5 µs.
  - This is the Part 7 scenario 3 check: "the blinkers, to confirm time synchronization survives the election".

### Part 7 demo scenarios: Raft state machine (§2)
Each file holds one node's `net show` (= `getNetwork()`), `raft log` and `raft status`. Suffix `_n2`/`_n3`/`_pi` = node.

| Scenario (Part 7) | Mode | Files | What it shows |
|---|---|---|---|
| 1. Link failure | default (Pi leads) | `s1_*` | entry 11 `CANLINK_DOWN N3 B`, term 1, all three identical |
| 2. Link failure | leader setup = `LEADER=random` (N3 leads) | `s2_fail_*` | entry 11 `CANLINK_DOWN N2 A`, term 1 |
| 2. …and recovery | same | `s2_recover_*` | entries 12–16; the jumper bounced, so there are 2 extra down/up pairs, identical on all nodes |
| 3. Leader failure | random (N3 leads) | `s3_fail_n2`, `s3_fail_pi` | N3 halted; N2 wins term 2; entry 17 `NODE_DOWN N3`. The election debug lines are at the top of `s3_fail_n2` |
| 3. …N3 back | same | `s3_rejoin_*` | N3 rejoins as a follower; logs identical through entry 20 |

### Part 5: routing during a multi-link failure (§3)
- `captures/hop_*.txt`: the two-link failure (Pi off bus B, N3 off bus A, Pi leading).
  - Entries 13 and 14 are on all nodes, term 1, `getNetwork()` = `N1 up A+ B- | N2 up A+ B+ | N3 up A- B+`.
  - `raft status` on N2 shows `relay N1 A->B, N3 B->A`.
- `captures/hop_timeline_n2.txt`, `hop_timeline_n3.txt`: from the first failure to the relay starting, with timestamps. `relayed N (5 s)` goes from 0 to 809 once entry 14 commits.
  - This is the 5.2 two-failure edge case.

## Caveats
- **No scope captures.** The handout says the report quantifies drift and jitter from the PPS printouts. The scope is for the live demo.
- **No `s3_fail_n3`.** N3 was the halted leader.
- **Pi PPS bug in random mode** (not fixed yet). When the Pi joins a cluster whose global time is behind its own uptime, `wait_global()` in `cockpit/cockpit.c` waits for the old target second, so its PPS and HZ outputs stall for up to about 20 min. That is why `pi_random.log` has no PPS lines.
