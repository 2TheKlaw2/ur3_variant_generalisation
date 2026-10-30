#!/usr/bin/env python3
"""Report figures for Stage 9 (V1 demo -> V1-V5 under L0/L1/L2), from the saved results.

  python3 scripts/figures/make_stage9_figures.py [--out docs/figures]
Kinematic simulation results only. Writes PNG (300 dpi) and SVG.
"""
import argparse
import csv
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
RES = os.path.join(PKG, "results/sim_reference/stage9_offline_20261030_fixture12p5")
CAD = os.path.join(PKG, "results/cad_features")
C = {"L0": "#2a78d6", "L1": "#eb6834", "L2": "#1baf7a"}
INK, INK2, GRID, SURF, FREE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb", "#dcdad3"
NAMES = {"L0": "L0 no adaptation", "L1": "L1 CAD goal scaling", "L2": "L2 CAD geometry-conditioned"}


def style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
                         "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": SURF, "axes.facecolor": SURF, "legend.frameon": False,
                         "lines.linewidth": 2.0, "savefig.facecolor": SURF})


def fig_outcomes(out):
    S = json.load(open(os.path.join(RES, "stage9_summary.json")))
    fig, ax = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
    w = 0.26
    for j, lev in enumerate(("L0", "L1", "L2")):
        xs, ys, labels = [], [], []
        for i in range(1, 6):
            s = S[f"V{i}_{lev}"]
            xs.append(i + (j - 1) * w)
            ys.append(100 * s["max_insertion_depth_mm"] / s["cad_seat_depth_mm"])
            labels.append(s["functional_outcome"])
        ax.bar(xs, ys, width=w - 0.04, color=C[lev], label=NAMES[lev], edgecolor=SURF, linewidth=2)
        for x, y, lab in zip(xs, ys, labels):
            ax.text(x, y + 2, "✓" if lab == "PASS" else "✗", ha="center", va="bottom", color=INK, fontsize=10)
    ax.set_xticks(range(1, 6)); ax.set_xticklabels([f"V{i}" for i in range(1, 6)])
    ax.set_ylim(0, 118); ax.set_ylabel("Insertion depth reached\n(% of CAD seat depth)")
    ax.set_title("V1 demonstration transferred to V1–V5 (✓ assembled and locked, ✗ failed)", loc="left", fontsize=10, color=INK)
    ax.grid(axis="x", visible=False)
    fig.legend(loc="outside lower center", ncol=3, fontsize=8)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(out, f"stage9_outcomes.{ext}"), dpi=300)
    plt.close(fig)


def corridor(i):
    rows = {}
    for r in csv.DictReader(open(os.path.join(CAD, f"V{i}_corridor.csv"))):
        rows.setdefault(float(r["z_mm"]), [])
        if int(r["interval"]) >= 0:
            rows[float(r["z_mm"])].append((float(r["lo_deg"]), float(r["hi_deg"])))
    return rows


def traj(i, lev):
    rows = list(csv.DictReader(open(os.path.join(RES, f"V{i}_{lev}", "trajectory.csv"))))
    return rows


def fig_corridors(out):
    feats = json.load(open(os.path.join(CAD, "cad_features.json")))
    fig, axs = plt.subplots(1, 3, figsize=(7.2, 3.6), sharey=False, constrained_layout=True)
    for ax, i in zip(axs, (1, 4, 5)):
        rows = corridor(i); top = feats[f"V{i}"]["fitting_top_mm"]
        for z, ints in rows.items():
            for lo, hi in ints:
                ax.fill_betweenx([top - z - 0.05, top - z + 0.05], max(-hi, -40), min(-lo, 40), color=FREE, lw=0)  # CW positive
        for lev in ("L1", "L2"):
            tr = [r for r in traj(i, lev)]
            d = [top - float(r["v1_lowest_z_mm"]) for r in tr if r["feasible"] == "True"]
            y = [-float(r["v1_yaw_deg"]) for r in tr if r["feasible"] == "True"]
            ax.plot(y, d, color=C[lev], lw=1.8, label=NAMES[lev])
            bad = [r for r in tr if r["feasible"] == "False"]
            if bad:
                ax.plot(-float(bad[0]["v1_yaw_deg"]), top - float(bad[0]["v1_lowest_z_mm"]), "X", color=C[lev], ms=8,
                        mec=SURF, mew=1.2, zorder=5)
        ax.axhline(top - 2.0, color=INK2, lw=1)
        ax.set_ylim(top - 2.0 + 0.6, -1.0)
        ax.set_xlim(-5, 32)
        ax.set_title(f"V{i}", loc="left", fontsize=10, color=INK)
        ax.set_xlabel("rotation (° CW)")
    axs[0].set_ylabel("part bottom below fitting top (mm)")
    h, l = axs[1].get_legend_handles_labels()
    from matplotlib.patches import Patch
    h.append(Patch(color=FREE)); l.append("CAD collision-free yaw (exact FCL, 0.02 mm tol.)")
    fig.legend(h, l, loc="outside lower center", ncol=3, fontsize=8)
    fig.suptitle("Part path vs CAD free corridor (floor = solid line); ROTATE shown at the seat", x=0.01, ha="left",
                 fontsize=10, color=INK)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(out, f"stage9_corridors.{ext}"), dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=os.path.join(PKG, "docs/figures"))
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); style()
    fig_outcomes(a.out); fig_corridors(a.out)
    print("figures ->", a.out)


if __name__ == "__main__":
    main()
