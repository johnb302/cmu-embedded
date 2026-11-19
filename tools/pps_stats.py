#!/usr/bin/env python3
"""Drift and jitter from PPS console lines, grouped by node and time master.

    tools/pps_stats.py lab5_report/logs/n2.log lab5_report/logs/n3.log lab5_report/logs/pi.log
"""
import re
import statistics
import sys
from collections import defaultdict

# STM32: "... | synced to N1"; Pi: "... | master N1".
PPS = re.compile(
    r"PPS (N\d) \| global \d+\.(\d{6}) s .*?\(([+-][\d.]+) ppm\) \| since (?:REF|last sync) [+-]\d+ us"
    r" \| last step ([+-]\d+) us \| (?:synced to|master) (N\d)"
)


def pct(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(p / 100 * len(ordered)))]


groups = defaultdict(lambda: {"edge": [], "ppm": [], "step": []})
for path in sys.argv[1:]:
    for line in open(path, errors="replace"):
        m = PPS.search(line)
        if not m:
            continue
        node, frac_us, ppm, step, master = m.groups()
        if master == "N0":  # not synced yet
            continue
        # Edge offset from the whole second; .999xxx means the edge came early.
        edge_us = int(frac_us) if int(frac_us) < 500000 else int(frac_us) - 1000000
        g = groups[(node, master)]
        g["edge"].append(edge_us)
        g["ppm"].append(float(ppm))
        g["step"].append(abs(int(step)))

print(f"{'node':5}{'master':7}{'n':>5}  {'edge med/p90/p99 us':>21}  {'drift med [min,max] ppm':>26}  {'|step| med/p90 us':>18}")
for (node, master), g in sorted(groups.items()):
    edge = [abs(e) for e in g["edge"]]
    print(f"{node:5}{master:7}{len(edge):5}"
          f"  {statistics.median(edge):7.0f}{pct(edge, 90):7}{pct(edge, 99):7}"
          f"  {statistics.median(g['ppm']):+10.2f} [{min(g['ppm']):+.2f},{max(g['ppm']):+.2f}]"
          f"  {statistics.median(g['step']):9.0f}{pct(g['step'], 90):9}")
