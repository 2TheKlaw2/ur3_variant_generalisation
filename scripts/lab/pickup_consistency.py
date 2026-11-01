#!/usr/bin/env python3
"""Hand-pickup SETUP CONSISTENCY check (READ-ONLY: never calls rg2.sh, never commands motion or the gripper).

  python3 scripts/lab/pickup_consistency.py                  # 5 V1 pickups by hand (default)
  python3 scripts/lab/pickup_consistency.py --trials 8 --variant 1
  python3 scripts/lab/pickup_consistency.py --self-test      # offline check of the summary maths (no ROS)

The operator does N pickups of the part by hand: freedrive (bash scripts/lab/freedrive.sh) and the RG2
(bash scripts/lab/rg2.sh grip|open), both in OTHER terminals. At the same checkpoint every time
(gripper closed on the part, BEFORE lifting) this script only LISTENS and records:
  * the TCP pose (TF base -> gripper_tcp), averaged over ~1 s,
  * the TOOL tilt (gripper_tcp z vs base -z; tool, not part),
  * the measured RG2 width (finger_width in /joint_states).
Then it asks for the operator's observations (y/n): did the part shift while closing? is it tilted after
lifting? did the pickup plate lift?

SCOPE: the spread it reports is SETUP CONSISTENCY ONLY (how repeatably the operator placed the tool and how
repeatably the RG2 closed). It does NOT measure part-to-gripper repeatability or slip: nothing here sees
the part. Missing data are recorded as unavailable (empty), never as zero.
Output: results/physical/pickup_consistency/<time>/trials.csv + summary.yaml (written after every trial).
"""
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tool_tilt as tt  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SCOPE = ("SETUP CONSISTENCY ONLY: spread of the TCP pose, TOOL tilt and RG2 width at the closed-gripper checkpoint "
         "over hand pickups. It does NOT measure part-to-gripper repeatability or slip.")
QUESTIONS = [("part_shifted_while_closing", "Did the part SHIFT while the RG2 was closing?"),
             ("part_tilted_after_lifting", "Lift ~30 mm by freedrive and look: is the part TILTED in the gripper?"),
             ("pickup_plate_lifted", "Did the PICKUP PLATE lift (or move) during the lift?")]
FIELDS = (["trial", "time", "ros_domain_id", "tcp_x_mm", "tcp_y_mm", "tcp_z_mm", "qx", "qy", "qz", "qw",
           "tool_tilt_deg", "lean_about_base_x_deg", "lean_about_base_y_deg", "tool_yaw_deg",
           "rg2_width_mm", "tf_samples", "width_samples", "tcp_motion_in_window_mm", "data_note"]
          + [k for k, _ in QUESTIONS] + ["operator_note"])
MOVING_MM = 0.5     # TCP range inside the ~1 s window above which the trial is flagged as "arm moving"


# ------------------------------------------------------------------ pure maths (tested offline)
def tool_yaw_deg(q_xyzw):
    """Heading of the tool x-axis in the base x-y plane [deg] (yaw about base z)."""
    R = tt.quat_xyzw_to_matrix(q_xyzw)
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def _num(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _stats(vals):
    v = [x for x in (_num(x) for x in vals) if x is not None]
    if not v:
        return {"n": 0, "mean": None, "std": None, "min": None, "max": None, "range": None}
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1)) if len(v) > 1 else 0.0
    r = lambda x: round(x, 4)  # noqa: E731
    return {"n": len(v), "mean": r(m), "std": r(sd), "min": r(min(v)), "max": r(max(v)), "range": r(max(v) - min(v))}


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def summarise(trials):
    """Spread over the recorded trials. Trials without data contribute nothing (never zeros)."""
    out = {"scope": SCOPE, "trials_total": len(trials)}
    for k in ("tcp_x_mm", "tcp_y_mm", "tcp_z_mm", "tool_tilt_deg", "lean_about_base_x_deg",
              "lean_about_base_y_deg", "rg2_width_mm"):
        out[k] = _stats([t.get(k) for t in trials])
    pos = [(t["tcp_x_mm"], t["tcp_y_mm"], t["tcp_z_mm"]) for t in trials
           if all(_num(t.get(k)) is not None for k in ("tcp_x_mm", "tcp_y_mm", "tcp_z_mm"))]
    if pos:
        c = [sum(p[j] for p in pos) / len(pos) for j in range(3)]
        out["tcp_max_distance_from_mean_mm"] = round(max(math.dist(p, c) for p in pos), 4)
    else:
        out["tcp_max_distance_from_mean_mm"] = None
    yaws = [_num(t.get("tool_yaw_deg")) for t in trials]
    yaws = [y for y in yaws if y is not None]
    if yaws:
        rel = [_wrap_deg(y - yaws[0]) for y in yaws]          # unwrap around the first trial
        out["tool_yaw_range_deg"] = round(max(rel) - min(rel), 4)
    else:
        out["tool_yaw_range_deg"] = None
    out["observations_yes"] = {k: sum(1 for t in trials if t.get(k) == "y") for k, _ in QUESTIONS}
    out["observations_answered"] = {k: sum(1 for t in trials if t.get(k) in ("y", "n")) for k, _ in QUESTIONS}
    out["trials_without_tcp"] = sum(1 for t in trials if _num(t.get("tcp_x_mm")) is None)
    out["trials_without_width"] = sum(1 for t in trials if _num(t.get("rg2_width_mm")) is None)
    out["trials_flagged_arm_moving"] = sum(1 for t in trials if (_num(t.get("tcp_motion_in_window_mm")) or 0) > MOVING_MM)
    return out


def trial_row(k, domain, tf_samples, widths):
    """One trial from raw samples. tf_samples: [(xyz_m, quat_xyzw)], widths: [m]. Missing -> empty, never 0."""
    row = {f: "" for f in FIELDS}
    row.update(trial=k, time=datetime.now().isoformat(timespec="seconds"), ros_domain_id=domain,
               tf_samples=len(tf_samples), width_samples=len(widths))
    notes = []
    if tf_samples:
        xyz = [sum(s[0][j] for s in tf_samples) / len(tf_samples) for j in range(3)]
        q = list(tf_samples[-1][1])
        t = tt.tool_tilt_from_quat(q)
        motion = max(math.dist(s[0], xyz) for s in tf_samples) * 2000.0   # ~range in mm
        row.update(tcp_x_mm=round(xyz[0] * 1000, 3), tcp_y_mm=round(xyz[1] * 1000, 3), tcp_z_mm=round(xyz[2] * 1000, 3),
                   qx=q[0], qy=q[1], qz=q[2], qw=q[3], tool_tilt_deg=round(t["tool_tilt_deg"], 3),
                   lean_about_base_x_deg=round(t["lean_about_base_x_deg"], 3),
                   lean_about_base_y_deg=round(t["lean_about_base_y_deg"], 3),
                   tool_yaw_deg=round(tool_yaw_deg(q), 3), tcp_motion_in_window_mm=round(motion, 3))
        if motion > MOVING_MM:
            notes.append(f"arm moving during the record window ({motion:.2f} mm)")
    else:
        notes.append("TCP unavailable (no fresh TF)")
    if widths:
        row["rg2_width_mm"] = round(sum(widths) / len(widths) * 1000, 3)
    else:
        notes.append("RG2 width unavailable (no fresh finger_width)")
    row["data_note"] = "; ".join(notes)
    return row


def write_outputs(outdir, trials, meta):
    import yaml
    with open(outdir / "trials.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(trials)
    with open(outdir / "summary.yaml", "w") as f:
        yaml.safe_dump({**meta, **summarise(trials)}, f, sort_keys=False)


def print_summary(s):
    print("\n================ PICKUP SETUP CONSISTENCY ================")
    print(s["scope"])
    for k, lab in (("tcp_x_mm", "TCP x [mm]"), ("tcp_y_mm", "TCP y [mm]"), ("tcp_z_mm", "TCP z [mm]"),
                   ("tool_tilt_deg", "TOOL tilt [deg]"), ("rg2_width_mm", "RG2 width [mm]")):
        v = s[k]
        print(f"  {lab:16s} n={v['n']}  " + ("unavailable" if v["n"] == 0 else
              f"mean {v['mean']:.3f}  std {v['std']:.3f}  range {v['range']:.3f}"))
    print(f"  TCP max distance from mean: {s['tcp_max_distance_from_mean_mm']} mm; tool yaw range: "
          f"{s['tool_yaw_range_deg']} deg")
    print(f"  operator 'yes' answers: {s['observations_yes']} (of {s['observations_answered']})")
    if s["trials_without_tcp"] or s["trials_without_width"] or s["trials_flagged_arm_moving"]:
        print(f"  data gaps: {s['trials_without_tcp']} without TCP, {s['trials_without_width']} without width, "
              f"{s['trials_flagged_arm_moving']} flagged 'arm moving'")


# ------------------------------------------------------------------ ROS (listen only)
def run(a):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener

    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    outdir = Path(a.out) if a.out else ROOT / "results/physical/pickup_consistency" / datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir.mkdir(parents=True, exist_ok=False)
    meta = {"type": "PICKUP SETUP CONSISTENCY (hand pickups, read-only)", "variant": f"V{a.variant}",
            "ros_domain_id": domain, "started": datetime.now().isoformat(timespec="seconds"),
            "checkpoint": "RG2 closed on the part, before lifting", "tool_tilt_sign_convention": tt.SIGN_CONVENTION,
            "tool_tilt_note": "TOOL tilt (gripper_tcp z vs base -z), not measured part tilt"}
    if domain == "73":
        meta["warning"] = "ROS_DOMAIN_ID=73 is the fake-hardware domain: NOT physical data"
    rclpy.init()
    node = rclpy.create_node("pickup_consistency_readonly")
    buf = Buffer(); TransformListener(buf, node)
    st = {"w": None, "w_at": 0.0}

    def on_js(m):
        if "finger_width" in m.name and len(m.name) == len(m.position):
            w = float(m.position[m.name.index("finger_width")])
            if math.isfinite(w):
                st["w"], st["w_at"] = w, time.monotonic()
    node.create_subscription(JointState, "/joint_states", on_js, qos_profile_sensor_data)

    def sample(window):
        t0 = time.monotonic(); tfs, ws = [], []; last_stamp = None
        while time.monotonic() - t0 < window:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                tf = buf.lookup_transform(a.base_frame, a.tool_frame, Time(), timeout=Duration(seconds=0.0))
                stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
                age = (node.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
                if stamp != last_stamp and age < 0.5:
                    p, r = tf.transform.translation, tf.transform.rotation
                    tfs.append(((p.x, p.y, p.z), (r.x, r.y, r.z, r.w))); last_stamp = stamp
            except Exception:  # noqa: BLE001 - recorded as unavailable below
                pass
            if st["w"] is not None and st["w_at"] >= t0:
                ws.append(st["w"]); st["w_at"] = 0.0
        return tfs, ws

    print(f"\nPICKUP SETUP CONSISTENCY, V{a.variant}, {a.trials} trials. READ-ONLY: this script never moves the arm "
          f"or the RG2.\nROS_DOMAIN_ID={domain}{'  (FAKE HARDWARE: not physical data)' if domain == '73' else ''}\n"
          f"{SCOPE}\nLog: {outdir}\n")
    trials = []
    try:
        for k in range(1, a.trials + 1):
            print(f"---------------- trial {k}/{a.trials} ----------------")
            print(f"  RG2 open (rg2.sh open, other terminal). Freedrive onto part V{a.variant} in its pocket exactly as "
                  "for pickup. Close the RG2 (rg2.sh grip). Let go of the arm; do NOT lift yet.")
            r = input("  Checkpoint reached (closed, before lifting)? [Enter = record, s = skip trial, q = quit] ").strip().lower()
            if r == "q":
                break
            if r == "s":
                continue
            tfs, ws = sample(a.window)
            row = trial_row(k, domain, tfs, ws)
            if row["tcp_x_mm"] != "":
                print(f"  recorded: TCP [{row['tcp_x_mm']:.2f} {row['tcp_y_mm']:.2f} {row['tcp_z_mm']:.2f}] mm, "
                      f"TOOL tilt {row['tool_tilt_deg']:.2f} deg, width "
                      f"{row['rg2_width_mm'] if row['rg2_width_mm'] != '' else 'UNAVAILABLE'} mm")
            if row["data_note"]:
                print(f"  NOTE: {row['data_note']}")
            for key, q in QUESTIONS:
                ans = ""
                while ans not in ("y", "n"):
                    ans = input(f"  {q} [y/n] ").strip().lower()
                row[key] = ans
            row["operator_note"] = input("  Any note (Enter = none): ").strip()
            trials.append(row)
            write_outputs(outdir, trials, meta)
            input(f"  Put V{a.variant} back in its pocket and open the RG2 (rg2.sh open). Enter when done. ")
    except (KeyboardInterrupt, EOFError):
        print("\nStopped by the operator.")
    finally:
        meta["ended"] = datetime.now().isoformat(timespec="seconds")
        write_outputs(outdir, trials, meta)
        print_summary(summarise(trials))
        print(f"\nLog -> {outdir}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=5, help="number of hand pickups (default 5)")
    ap.add_argument("--variant", type=int, default=1, choices=[1, 2, 3, 4, 5], help="part to pick (default V1)")
    ap.add_argument("--window", type=float, default=1.0, help="record window at the checkpoint [s] (default 1.0)")
    ap.add_argument("--base-frame", default="base")
    ap.add_argument("--tool-frame", default="gripper_tcp")
    ap.add_argument("--out", help="output folder (default results/physical/pickup_consistency/<time>)")
    ap.add_argument("--self-test", action="store_true", help="run the offline tests and exit (no ROS)")
    a = ap.parse_args()
    if a.self_test:
        import test_lab_tools
        return test_lab_tools.run_all()
    if a.trials < 1 or not 0.2 <= a.window <= 5:
        raise SystemExit("--trials >= 1 and --window in [0.2, 5] s")
    return run(a)


if __name__ == "__main__":
    sys.exit(main())
