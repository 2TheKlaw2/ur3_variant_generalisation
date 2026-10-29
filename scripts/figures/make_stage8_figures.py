#!/usr/bin/env python3
"""Report figures for Stage 8 (V1 A / B1 / B2 / B2R), read from the saved result CSVs.

  python3 scripts/figures/make_stage8_figures.py [--out docs/figures]
Kinematic simulation results only. Writes PNG (300 dpi) and SVG.
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
REF = os.path.join(PKG, "results/sim_reference/stage8_20261029_233555")
B2R = os.path.join(PKG, "results/sim_reference/stage8_b2r_offline_20261030/v1_geometry_conditioned_rotate_unseated")
# categorical slots 1-4 (fixed order), light surface; text never in series colour
C = {"A": "#2a78d6", "B1": "#eb6834", "B2": "#1baf7a", "B2R": "#eda100"}
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
FLOOR_Z, RING_Z, CLEAR = 2.0, 2.5, 0.25


def style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
                         "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": SURF, "axes.facecolor": SURF, "legend.frameon": False,
                         "lines.linewidth": 2.0, "savefig.facecolor": SURF})


def load(path):
    with open(os.path.join(path, "trajectory.csv")) as f:
        return list(csv.DictReader(f))


def col(rows, stage, key):
    r = [x for x in rows if x["stage"] == stage]
    return [float(x["t_s"]) for x in r], [float(x[key]) for x in r], r


def mark_stop(ax, rows, stage, key, color, label, offset=0.0):
    bad = [x for x in rows if x["stage"] == stage and x["feasible"] == "False"]
    if bad:
        b = bad[0]
        ax.plot(float(b["t_s"]), float(b[key]) - offset, marker="X", ms=9, color=color, mec=SURF, mew=1.5, zorder=5)
        ax.annotate(label, (float(b["t_s"]), float(b[key]) - offset), xytext=(6, 4), textcoords="offset points", color=INK, fontsize=8)


def fig_insert(out):
    runs = {"A": load(os.path.join(REF, "v1_original_alignment")), "B1": load(os.path.join(REF, "v1_corrected_cdmp")),
            "B2": load(os.path.join(REF, "v1_geometry_conditioned"))}
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.9), constrained_layout=True)
    ax = axs[0]
    for k, rows in runs.items():
        t, y, _ = col(rows, "INSERT", "v1_centre_offset_mm")
        ax.plot(t, y, color=C[k], label=k)
        mark_stop(ax, rows, "INSERT", "v1_centre_offset_mm", C[k], f"{k} stops")
    ax.axhline(CLEAR, color=INK2, lw=1, ls="--")
    ax.text(12.9, CLEAR, "radial clearance 0.25 mm", ha="right", va="bottom", color=INK2, fontsize=8)
    ax.set_yscale("symlog", linthresh=0.1); ax.set_ylim(0, 5)
    ax.set_yticks([0, 0.05, 0.1, 0.25, 0.5, 1, 2, 4]); ax.set_yticklabels(["0", "0.05", "0.1", "0.25", "0.5", "1", "2", "4"])
    ax.set_xlabel("INSERT time (s)"); ax.set_ylabel("V1 centre offset from fitting axis (mm)")
    ax.set_title("Lateral alignment during INSERT", loc="left", fontsize=10, color=INK)
    ax = axs[1]
    for k in ("A", "B2", "B1"):
        rows = runs[k]
        t, y, _ = col(rows, "INSERT", "v1_lowest_z_mm")
        ax.plot(t, [v - FLOOR_Z for v in y], color=C[k], label=k, lw=2.0 if k != "B1" else 1.6)
        mark_stop(ax, rows, "INSERT", "v1_lowest_z_mm", C[k], "", FLOOR_Z)
    ax.axhline(0, color=INK2, lw=1)
    ax.text(0.2, 0.08, "fitting floor", color=INK2, fontsize=8)
    ax.text(12.9, 0.45, "B2 ends 0.286 mm\nabove the floor", ha="right", color=INK, fontsize=8)
    ax.text(4.3, 6.3, "B1 follows B2 until it stops", color=INK, fontsize=8)
    ax.set_ylim(-0.2, 7)
    ax.set_xlabel("INSERT time (s)"); ax.set_ylabel("V1 lowest point above floor (mm)")
    ax.set_title("Insertion depth", loc="left", fontsize=10, color=INK)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, ["A original", "B1 corrected start + rigid CDMP", "B2 geometry-conditioned"], loc="outside lower center",
               ncol=3, fontsize=8)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(out, f"stage8_insert_A_B1_B2.{ext}"), dpi=300)
    plt.close(fig)


def fig_b2r(out):
    rows = load(B2R)
    ti, zi, _ = col(rows, "INSERT", "v1_lowest_z_mm")
    tr, zr, rr = col(rows, "ROTATE", "v1_lowest_z_mm")
    _, yr, _ = col(rows, "ROTATE", "v1_yaw_deg")
    T0 = ti[-1]
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 4.2), sharex=True, constrained_layout=True)
    ax = axs[0]
    ax.plot([T0 + t for t in tr], [-y for y in yr], color=C["B2R"])
    ax.axhspan(23.5, 26.5, color=GRID, lw=0, zorder=0)
    ax.text(T0 + 0.3, 25.0, "accepted 23.5-26.5° CW", va="center", color=INK2, fontsize=8)
    ax.axhline(26.5, color=INK2, lw=1, ls="--"); ax.text(T0 + 16.8, 27.0, "CAD stop ~26.5° CW", ha="right", color=INK2, fontsize=8)
    ax.plot([T0 + tr[-1]], [-yr[-1]], "o", color=C["B2R"], ms=6, mec=SURF, mew=1.5)
    ax.annotate(f"{-yr[-1]:.2f}° CW", (T0 + tr[-1], -yr[-1]), xytext=(-60, -14), textcoords="offset points", color=INK, fontsize=8)
    ax.set_ylabel("Rotation (° CW)"); ax.set_ylim(-1, 29)
    ax.set_title("B2R: V1 rotation (ROTATE starts from B2's unseated INSERT end)", loc="left", fontsize=10, color=INK)
    ax = axs[1]
    ax.plot(ti, [z - FLOOR_Z for z in zi], color=C["B2"], label="INSERT (= B2)")
    ax.plot([T0 + t for t in tr], [z - FLOOR_Z for z in zr], color=C["B2R"], label="ROTATE")
    ax.axhline(0, color=INK2, lw=1); ax.text(0.3, 0.06, "floor", color=INK2, fontsize=8)
    ax.axhline(RING_Z - FLOOR_Z, color=INK2, lw=1, ls="--")
    ax.text(0.3, RING_Z - FLOOR_Z + 0.06, "tab top meets ring underside (0.50 mm)", color=INK2, fontsize=8)
    touch = [x for x in rr if x["status"] == "touching"]
    if touch:
        ax.axvspan(T0 + float(touch[0]["t_s"]), T0 + float(touch[-1]["t_s"]), color=GRID, lw=0, zorder=0)
        ax.text(T0 + float(touch[0]["t_s"]) - 0.2, 0.75, "touching, max overlap 0.015 mm\n(tolerance 0.02 mm)", ha="right",
                color=INK, fontsize=8)
    ax.set_ylim(-0.1, 1.5)
    ax.set_xlabel("time from INSERT start (s)"); ax.set_ylabel("V1 lowest point above floor (mm)")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 0.85), fontsize=8)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(out, f"stage8_b2r_rotate.{ext}"), dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=os.path.join(PKG, "docs/figures"))
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); style()
    fig_insert(a.out); fig_b2r(a.out)
    print("figures ->", a.out)


if __name__ == "__main__":
    main()
