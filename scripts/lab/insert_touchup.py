#!/usr/bin/env python3
"""READ-ONLY touch-up of one fitting's position with the ROBOT'S OWN grasp (never commands motion).

Why: the fittings are registered from hand-gripped calibration poses. If the robot's grasp holds the part slightly
differently, the part arrives off-centre at the fitting (5 Nov, V1: consistently toward the robot side, rim contact).
Procedure (real robot, ROS_DOMAIN_ID=0):
  1. run_full_cycle_physical.py --plans <plans> --variant <i> --level <lvl> --until ALIGN_INSERT --execute
     (the part stops gripped just above the fitting entry)
  2. bash scripts/lab/freedrive.sh  -> gently guide the part straight down 2-3 mm INTO the bore so the bore centres it
     (no twist, no pushing). Let go of the arm.
  3. python3 scripts/lab/insert_touchup.py --plans <plans> --variant <i>            # measures, prints the correction
     python3 scripts/lab/insert_touchup.py --plans <plans> --variant <i> --save     # adds it to the session
  4. Ctrl+C freedrive; open the RG2; put the part back; rebuild that variant's plans (the script prints how).
The correction = live TCP xy - planned ALIGN_INSERT end TCP xy (base frame). It is ADDED to the fitting's offset
via config/calibration/<session>/touchup_offsets_mm.yaml, which compute_scene_registration.py applies.
"""
import argparse
import csv
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/scene"))
import ur_kinematics as uk  # noqa: E402

JOINTS = [f"{x}_joint" for x in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
MAX_SANE_MM = 3.0          # a larger xy correction means the part is not in its bore: refuse to save


def planned_align_end(plans, i):
    rows = [r for r in csv.DictReader(open(Path(plans) / f"V{i}" / "joint_trajectory.csv")) if r["stage"] == "ALIGN_INSERT"]
    if not rows:
        raise SystemExit(f"no ALIGN_INSERT stage in {plans}/V{i}/joint_trajectory.csv")
    return [float(rows[-1][j]) for j in JOINTS]


def live_joints(timeout=5.0):
    if os.environ.get("ROS_DOMAIN_ID", "0") != "0":
        raise SystemExit("real robot only: unset ROS_DOMAIN_ID (domain 0)")
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    rclpy.init(); n = rclpy.create_node("insert_touchup"); got = {}

    def cb(m):
        if all(j in m.name for j in JOINTS):
            got["q"] = [float(m.position[m.name.index(j)]) for j in JOINTS]
    n.create_subscription(JointState, "/joint_states", cb, qos_profile_sensor_data)
    t0 = time.monotonic()
    while "q" not in got and time.monotonic() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.shutdown()
    if "q" not in got:
        raise SystemExit("no /joint_states (driver running?)")
    return got["q"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plans", required=True, help="the plans folder used for the run, e.g. results/simulation/stage10_fitfix")
    ap.add_argument("--variant", type=int, required=True, choices=range(1, 6))
    ap.add_argument("--save", action="store_true", help="add the measured xy correction to the session's touch-up file")
    a = ap.parse_args()
    calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
    reg = yaml.safe_load(open(ROOT / "config/scene/scene_registration.yaml"))
    sess = reg.get("poses_dir")
    if not sess:
        raise SystemExit("registration is not from a calibration session")
    Tp = uk.tcp(planned_align_end(a.plans, a.variant), calib)
    Tl = uk.tcp(live_joints(), calib)
    d = (Tl[:3, 3] - Tp[:3, 3]) * 1000
    tilt = np.degrees(np.arccos(np.clip(Tl[:3, 2] @ Tp[:3, 2], -1, 1)))
    print(f"V{a.variant}: live TCP - planned ALIGN_INSERT end TCP (base frame): dx {d[0]:+.2f}  dy {d[1]:+.2f}  dz {d[2]:+.2f} mm"
          f"   (xy {np.hypot(d[0], d[1]):.2f} mm; tool axis differs by {tilt:.2f} deg)")
    print("  dz < 0: the part went that much deeper than the planned insert start (expected: about -2 to -3 mm if you"
          " lowered it into the bore).")
    if np.hypot(d[0], d[1]) > MAX_SANE_MM:
        raise SystemExit(f"xy correction > {MAX_SANE_MM} mm: the part is probably not in its bore. Not saved.")
    if not a.save:
        print("  Not saved (add --save once the part sits freely centred in the bore).")
        return 0
    f = ROOT / sess / "touchup_offsets_mm.yaml"
    cur = yaml.safe_load(open(f)) if f.exists() else {}
    cur = cur or {}
    old = cur.get(f"V{a.variant}", [0.0, 0.0])
    cur[f"V{a.variant}"] = [round(old[0] + float(d[0]), 3), round(old[1] + float(d[1]), 3)]
    yaml.safe_dump(cur, open(f, "w"), sort_keys=True)
    print(f"  saved: {f.relative_to(ROOT)}  V{a.variant} touch-up now {cur[f'V{a.variant}']} mm (base frame, added to the fit)")
    print("  Rebuild this variant's plans (about 8 min):\n"
          f"    bash scripts/lab/rebuild_variant.sh {a.variant}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
