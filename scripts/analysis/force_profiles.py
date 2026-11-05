#!/usr/bin/env python3
"""Force profiles of the physical runs: contact force vs time during INSERT and ROTATE, per variant and level.

  python3 scripts/analysis/force_profiles.py [--plans stage10_depthfix] [--logs results/physical/full_cycle]
                                             [--out results/analysis/force_profiles.png]

Reads each run's wrench_log.csv (the full per-sample log, gitignored, laptop only) and status.json.
Plotted value: dF(t) = |F(t) - F(stage start)| (force vector change, tool0 frame; same metric as wrench_summary.json), i.e. the change caused by contact,
with the gripper/part weight and sensor bias removed. Time is seconds from the stage start.
Recording only: the runner never used these forces to control or stop motion.
"""
import argparse
import csv
import glob
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
LEVELS = {"L2": ("#2a78d6", "-"), "CAD": ("#eb6834", "--"), "L1": ("#1baf7a", "-."), "L1D": ("#eda100", ":"),
          "L0": ("#e87ba4", (0, (5, 1, 1, 1)))}       # fixed slot order; line style = secondary encoding
STAGES = ("INSERT", "ROTATE")


def load(run):
    t, s, f = [], [], []
    with open(run / "wrench_log.csv") as fh:
        for r in csv.DictReader(fh):
            if r["stage"] in STAGES:
                t.append(float(r["recv_monotonic_s"])); s.append(r["stage"])
                f.append([float(r["fx"]), float(r["fy"]), float(r["fz"])])
    t, s, f = np.array(t), np.array(s), np.array(f).reshape(-1, 3)
    out = {}
    for st in STAGES:
        m = s == st
        if m.sum() > 10:
            out[st] = (t[m] - t[m][0], np.linalg.norm(f[m] - f[m][0], axis=1))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plans", default="stage10_depthfix")
    ap.add_argument("--logs", default=str(ROOT / "results/physical/full_cycle"))
    ap.add_argument("--out", default=str(ROOT / "results/analysis/force_profiles.png"))
    a = ap.parse_args()
    runs = {}
    for d in sorted(glob.glob(os.path.join(a.logs, "V*_*"))):
        d = Path(d)
        if not (d / "wrench_log.csv").exists() or not (d / "status.json").exists():
            continue
        st = json.load(open(d / "status.json"))
        if a.plans and os.path.basename(str(st.get("plans", "")).rstrip("/")) != a.plans:
            continue
        prof = load(d)
        if len(prof) == 2:                                  # INSERT and ROTATE both executed
            runs.setdefault((st["variant"], st.get("level", "L2")), []).append((d.name, prof))
    if not runs:
        raise SystemExit(f"no runs with INSERT+ROTATE wrench logs for plans '{a.plans}' in {a.logs}")
    variants = sorted({v for v, _ in runs})
    fig, axes = plt.subplots(len(variants), 2, figsize=(10, 2.1 * len(variants) + 0.8), sharey=True, squeeze=False)
    plt.rcParams.update({"font.size": 9})
    ymax = max(np.max(p[s][1]) for rs in runs.values() for _, p in rs for s in STAGES)
    for i, v in enumerate(variants):
        for j, stg in enumerate(STAGES):
            ax = axes[i][j]
            peaks = []
            for lvl in LEVELS:
                for name, prof in runs.get((v, lvl), []):
                    t, df = prof[stg]
                    c, ls = LEVELS[lvl]
                    ax.plot(t, df, color=c, ls=ls, lw=1.6, label=lvl)
                    peaks.append(f"{lvl} {df.max():.1f}")
            ax.text(0.99, 0.95, "peak N: " + " · ".join(peaks), transform=ax.transAxes, ha="right", va="top",
                    fontsize=8, color="#444444")
            ax.set_ylim(-3, ymax * 1.15)
            ax.grid(axis="y", color="#e6e6e3", lw=0.6); ax.set_axisbelow(True)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
            for sp in ("left", "bottom"):
                ax.spines[sp].set_color("#b5b4ad")
            if i == 0:
                ax.set_title(stg, fontsize=10, color="#222222")
            if j == 0:
                ax.set_ylabel(f"{v}\n|ΔF| (N)", color="#444444")
            if i == len(variants) - 1:
                ax.set_xlabel("time from stage start (s)", color="#444444")
    h = {l: ln for ax in axes.flat for ln, l in zip(*ax.get_legend_handles_labels()[::-1][::-1])}
    fig.legend([h[l] for l in LEVELS if l in h], [l for l in LEVELS if l in h], loc="upper center", ncol=5,
               frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle("Contact force during INSERT and ROTATE (change from stage start), physical runs 5 Nov",
                 y=1.03, fontsize=11, color="#222222")
    fig.tight_layout()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=200, bbox_inches="tight", facecolor="#fcfcfb")
    print(f"{sum(len(r) for r in runs.values())} runs -> {a.out}")
    for (v, lvl), rs in sorted(runs.items()):
        for name, prof in rs:
            print(f"  {name:28s} INSERT peak {prof['INSERT'][1].max():5.1f} N   ROTATE peak {prof['ROTATE'][1].max():5.1f} N")


if __name__ == "__main__":
    main()
