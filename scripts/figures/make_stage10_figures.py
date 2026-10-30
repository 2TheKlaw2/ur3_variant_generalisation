#!/usr/bin/env python3
"""Stage 10 report figure: TRANSPORT arc height, demo amplitude vs the selected z forcing scale (V4, V5).

  python3 scripts/figures/make_stage10_figures.py --library <rs2_dmp_demo> [--out docs/figures]
Kinematic simulation only. CDMP rollouts of the V1 transport demo (Victor's CartesianDMPs), tau 2.
The X marks where the calibrated-UR3e IK fails (elbow singularity) on the demo-amplitude path.
"""
import argparse
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "phase10_full_sequence"))
import retarget_reach_transport as R  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

np = R.np
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
C_DEMO, C_SEL = "#2a78d6", "#1baf7a"
REF = Path(R.sm.PKG) / "results/sim_reference/stage10_retarget_20261031_zscale/stage10_summary.json"
FAILS = {4: 579, 5: 472}          # TRANSPORT sample where IK fails at k = 1 (30 Oct reference, tau 2, 20 ms)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--library", type=Path, required=True)
    ap.add_argument("--out", default=os.path.join(R.sm.PKG, "docs/figures"))
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    import yaml
    calib = yaml.safe_load(open(Path(R.sm.PKG) / "config/ur3e_lab_calibration.yaml"))
    objs = R.sm.scene_objects(); G, _ = R.rvo.grasps(objs, calib)
    q1, T1, z1, yaw0 = R.rvo.v1_start(objs, calib, G[1])
    tr, _, pt, qt = R.train(a.library, Path(R.sm.PKG) / R.TRANSPORT_CSV)
    sel = {int(k[1]): v["transport_z_scale"] for k, v in json.load(open(REF))["variants"].items()}
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
                         "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": SURF, "axes.facecolor": SURF, "legend.frameon": False, "savefig.facecolor": SURF})
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.0), sharey=True, constrained_layout=True)
    w0 = tr.wp[2, :].copy()
    for ax, i in zip(axs, (4, 5)):
        Tf = objs[f"fitting_V{i}"]["T"]; geom = R.vc.VariantGeometry(i, objs, Tf)
        Ts_ = R.vc.part_pose(Tf, geom.top_mm + (z1 - 7.0), yaw0) @ np.linalg.inv(G[i])
        Tg = objs[f"pickup_V{i}"]["T"] @ np.linalg.inv(G[i]); Tl = Tg.copy(); Tl[2, 3] += 0.03
        for k, col, ls, lab in ((1.0, C_DEMO, (0, (4, 2)), "demo amplitude (k = 1)"),
                                (sel[i], C_SEL, "-", f"selected k = {sel[i]}")):
            tr.wp[2, :] = w0 * k
            t, Ts, _ = R.rollout(tr, pt, qt, Tl, Ts_, 2.0)
            t, Ts = R.resample(t, Ts)
            P = np.array([T[:3, 3] for T in Ts]) * 1000
            ax.plot(-P[:, 0], P[:, 2], color=col, lw=2, ls=ls, label=lab)
            if k == 1.0:
                f = P[FAILS[i]]
                ax.plot(-f[0], f[2], "X", color=col, ms=9, mec=SURF, mew=1.2, zorder=5)
                ax.annotate("IK fails (elbow straight)", (-f[0], f[2]), xytext=(-6, 10), textcoords="offset points",
                            fontsize=8, color=INK2, ha="right")
            else:
                xm = (-P[:, 0].min() - P[:, 0].max()) / 2 * -1 if False else -(P[:, 0].min() + P[:, 0].max()) / 2
                j = int(np.argmin(np.abs(-P[:, 0] - xm) + 1e3 * (P[:, 2] < P[:, 2].max() - 40e0)))
                ax.annotate(lab, (-P[j, 0], P[j, 2]), xytext=(0, -16), textcoords="offset points",
                            fontsize=8, color=INK2, ha="center")
        tr.wp[2, :] = w0
        ax.set_title(f"V{i}", loc="left", fontsize=10, color=INK)

    axs[0].set_ylabel("TCP height z (mm)")
    for ax in axs:
        ax.set_xlabel("TCP along the row, −x (mm)  →  fitting")
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h[:1] + [plt.Line2D([], [], color=C_SEL, lw=2)], ["demo amplitude (k = 1)", "selected z forcing scale k"],
               loc="outside lower center", ncol=2, fontsize=8)
    fig.suptitle("TRANSPORT arc height: learned z forcing term scaled by k (same rule for all variants)",
                 x=0.01, ha="left", fontsize=10, color=INK)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(a.out, f"stage10_arc_height.{ext}"), dpi=300)
    print("->", a.out)


if __name__ == "__main__":
    main()
