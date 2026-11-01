#!/usr/bin/env python3
"""Powered HOLD TEST: does the gripped part stay straight while the ROBOT (not freedrive) moves it?

Start with the part gripped in its pocket (freedrive onto it, rg2.sh grip, then END freedrive with Ctrl+C so
scaled_joint_trajectory_controller is active). From that live pose the script plans, each move typed-confirmed:
  LIFT    straight up --lift mm (tool orientation held)
  CARRY   sideways +/- --carry mm along base x and back, at the lifted height
  TWIST   wrist_3 +/- --twist deg and back (torque on the grasp)
  LOWER   straight back down the same path into the pocket (skip with --no-lower)
After each move: look at the part (straight? slipped? turned in the jaws?) and answer.
The RG2 is never commanded. Nothing is planned against the scene: keep the area above the plates clear.

  python3 scripts/lab/hold_test.py                       # OFFLINE preview from the 27 Oct V1 pickup pose (no ROS)
  python3 scripts/lab/hold_test.py --execute             # REAL robot (ROS_DOMAIN_ID=0), staged
  python3 scripts/lab/hold_test.py --execute --fake      # fake hardware (ROS_DOMAIN_ID=73) to rehearse at home
  ... --speed 0.3                                        # faster repeat (hard cap 0.35 rad/s); default 0.14
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/scene"))
import ur_kinematics as uk  # noqa: E402

JOINTS = [f"{x}_joint" for x in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
HARD_MAX_SPEED = 0.35
START_TOL, END_TOL = 0.006, 0.012
STEP_M, STEP_RAD = 0.001, math.radians(0.5)


def err(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def cart_path(q0, T0, T1, calib):
    """Straight-line TCP path T0 -> T1 (same orientation), IK seeded point to point."""
    n = max(2, int(math.ceil(np.linalg.norm(T1[:3, 3] - T0[:3, 3]) / STEP_M)) + 1)
    qs, q = [], np.array(q0, float)
    for s in np.linspace(0, 1, n):
        T = T0.copy(); T[:3, 3] = T0[:3, 3] + s * (T1[:3, 3] - T0[:3, 3])
        q = uk.ik_local(q, T, calib); qs.append(q.copy())
    return qs


def timed(segs, vmax):
    """Each segment gets its own min-jerk timing (zero speed at both ends, so turnarounds are smooth);
    segments are concatenated; then everything is scaled so the peak joint speed is <= vmax."""
    t_all, q_all = [], []
    for seg in segs:
        p = _timed_one(seg, vmax); off = t_all[-1] if t_all else 0.0
        k = 1 if q_all else 0
        t_all += [off + x for x in p["t"][k:]]; q_all += p["q"][k:]
    peak = max(err(a, b) / (t1 - t0) for a, b, t0, t1 in zip(q_all, q_all[1:], t_all, t_all[1:]) if t1 > t0)
    f = max(1.0, peak / vmax)
    t_all = [x * f for x in t_all]
    return {"t": t_all, "q": q_all, "duration_s": t_all[-1], "peak_rad_s": peak / f}


def _timed_one(qs, vmax):
    qs = [list(map(float, q)) for q in qs]
    n = len(qs); d = max(err(qs[0], qs[-1]), 1e-6)
    T = max(2.0, 1.875 * d / vmax)
    # map path index -> time with a min-jerk profile (slow start/stop)
    s = np.linspace(0, 1, n); tau = np.interp(s, [10 * x**3 - 15 * x**4 + 6 * x**5 for x in np.linspace(0, 1, 400)],
                                             np.linspace(0, 1, 400))
    t = list(T * tau)
    peak = max(err(a, b) / (t1 - t0) for a, b, t0, t1 in zip(qs, qs[1:], t, t[1:]) if t1 > t0)
    f = max(1.0, peak / vmax)
    t = [x * f for x in t]
    return {"t": t, "q": qs, "duration_s": t[-1], "peak_rad_s": peak / f}


def plan(q0, calib, a):
    T0 = uk.tcp(q0, calib)
    up = T0.copy(); up[2, 3] += a.lift / 1000
    moves = []
    lift = cart_path(q0, T0, up, calib); moves.append(("LIFT", f"straight up {a.lift:.0f} mm", [lift]))
    if a.carry > 0:
        side = up.copy(); side[0, 3] += a.carry / 1000
        other = up.copy(); other[0, 3] -= a.carry / 1000
        s1 = cart_path(lift[-1], up, side, calib)
        s2 = cart_path(s1[-1], side, other, calib)
        s3 = cart_path(s2[-1], other, up, calib)
        moves.append(("CARRY", f"sideways +{a.carry:.0f} / -{a.carry:.0f} mm along base x and back", [s1, s2, s3]))
    if a.twist > 0:
        q = np.array(moves[-1][2][-1][-1]); w = math.radians(a.twist)
        rot = lambda x0, x1: [q + np.r_[0, 0, 0, 0, 0, x] for x in np.linspace(x0, x1, 41)]
        moves.append(("TWIST", f"wrist_3 +{a.twist:.0f} / -{a.twist:.0f} deg and back",
                      [rot(0, w), rot(w, -w), rot(-w, 0)]))
    if not a.no_lower:
        moves.append(("LOWER", f"straight back down {a.lift:.0f} mm into the pocket (same path as LIFT)",
                      [list(reversed(lift))]))
    out = []
    for name, what, qs in moves:
        if any(abs(q[2]) > math.radians(160) for seg in qs for q in seg):
            raise RuntimeError(f"{name}: elbow beyond +-160 deg")
        p = timed(qs, a.speed); p.update(name=name, what=what); out.append(p)
    for x, y in zip(out, out[1:]):
        assert err(x["q"][-1], y["q"][0]) < 1e-6, "moves do not chain"
    return out


def preview(moves, q0, calib, src):
    print(f"\nHOLD TEST PREVIEW (no ROS, no motion) from {src}")
    T0 = uk.tcp(q0, calib)
    print(f"start TCP {np.round(T0[:3, 3] * 1000, 1)} mm")
    for p in moves:
        Ts = [uk.tcp(q, calib) for q in p["q"]]
        tilt = max(math.degrees(math.acos(np.clip(T[:3, 2] @ T0[:3, 2], -1, 1))) for T in Ts) if p["name"] != "TWIST" else 0.0
        print(f"  {p['name']:6s} {p['what']:58s} {len(p['q']):4d} pts  {p['duration_s']:5.1f} s  "
              f"peak {p['peak_rad_s']:.3f} rad/s  tool axis change {tilt:.3f} deg")
    print(f"  total {sum(p['duration_s'] for p in moves):.1f} s of motion, peak {max(p['peak_rad_s'] for p in moves):.2f} rad/s")


def execute(moves_fn, calib, a):
    import rclpy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    dom = os.environ.get("ROS_DOMAIN_ID", "0")
    if a.fake and dom != "73":
        raise SystemExit("--fake needs ROS_DOMAIN_ID=73 (the fake-hardware domain)")
    if not a.fake and dom != "0":
        raise SystemExit(f"Real robot needs ROS_DOMAIN_ID=0, not {dom}")
    action = "/joint_trajectory_controller/follow_joint_trajectory" if a.fake else \
        "/scaled_joint_trajectory_controller/follow_joint_trajectory"

    class N(Node):
        def __init__(self):
            super().__init__("hold_test")
            self.q = None; self.at = 0.0; self.width = None; self.rec = []; self.label = ""
            self.create_subscription(JointState, "/joint_states", self.js, qos_profile_sensor_data)
            self.hw = self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
            self.ctl = self.create_client(ListControllers, "/controller_manager/list_controllers")
            self.arm = ActionClient(self, FollowJointTrajectory, action)

        def js(self, m):
            if "finger_width" in m.name:
                self.width = float(m.position[m.name.index("finger_width")])
            if all(j in m.name for j in JOINTS):
                self.q = [float(m.position[m.name.index(j)]) for j in JOINTS]; self.at = time.monotonic()
                if self.label:
                    self.rec.append([datetime.now().isoformat(timespec="milliseconds"), self.label] + self.q + [self.width])

        def call(self, cli, req, timeout=8):
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError("controller_manager service unavailable")
            f = cli.call_async(req); end = time.monotonic() + timeout
            while not f.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not f.done():
                raise RuntimeError("service timeout")
            return f.result()

        def check(self):
            types = [h.class_type for h in self.call(self.hw, ListHardwareComponents.Request()).component]
            st = {c.name: c.state.lower() for c in self.call(self.ctl, ListControllers.Request()).controller}
            if a.fake:
                if st.get("joint_trajectory_controller") != "active":
                    raise RuntimeError("fake: joint_trajectory_controller not active")
                return
            if "ur_robot_driver/URPositionHardwareInterface" not in types:
                raise RuntimeError("Real UR hardware interface not found")
            if any("mock_components" in t for t in types):
                raise RuntimeError("Mock hardware present: REJECTED")
            if st.get("scaled_joint_trajectory_controller") != "active":
                raise RuntimeError("scaled_joint_trajectory_controller not active (freedrive still on? Ctrl+C freedrive.sh)")

        def fresh(self):
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.q and self.at > t0:
                    return list(self.q)
            raise RuntimeError("no live joint state")

        def run(self, p):
            live = self.fresh()
            if err(live, p["q"][0]) > START_TOL:
                raise RuntimeError(f"{p['name']}: robot is {err(live, p['q'][0]):.4f} rad from the planned start; nothing sent")
            self.check()
            tr = JointTrajectory(); tr.joint_names = JOINTS
            for k, (t, q) in enumerate(zip(p["t"], p["q"])):
                pt = JointTrajectoryPoint(); pt.positions = list(live if k == 0 else q)
                pt.time_from_start = Duration(seconds=float(t)).to_msg(); tr.points.append(pt)
            g = FollowJointTrajectory.Goal(); g.trajectory = tr; g.goal_time_tolerance = Duration(seconds=5).to_msg()
            if not self.arm.wait_for_server(timeout_sec=5):
                raise RuntimeError("arm action server unavailable")
            self.label = p["name"]; h = None
            try:
                f = self.arm.send_goal_async(g)
                while not f.done():
                    rclpy.spin_once(self, timeout_sec=0.1)
                h = f.result()
                if not h.accepted:
                    raise RuntimeError(f"{p['name']}: goal rejected")
                r = h.get_result_async(); end = time.monotonic() + p["duration_s"] * 3 + 30
                while not r.done() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if time.monotonic() - self.at > 1.0:
                        raise RuntimeError("live joint state lost")
                if not r.done() or r.result().status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f"{p['name']}: controller did not succeed")
            except BaseException:
                if h is not None and h.accepted:
                    print("\nCANCELLING the goal (not an E-stop: use the pendant stop if it keeps moving)", file=sys.stderr)
                    c = h.cancel_goal_async(); end = time.monotonic() + 5
                    while not c.done() and time.monotonic() < end:
                        rclpy.spin_once(self, timeout_sec=0.1)
                raise
            finally:
                self.label = ""
            e = err(self.fresh(), p["q"][-1])
            if e > END_TOL:
                raise RuntimeError(f"{p['name']}: endpoint error {e:.4f} rad")
            print(f"{p['name']}: done ({p['duration_s']:.1f} s, endpoint err {e:.5f} rad, RG2 width "
                  f"{'?' if self.width is None else f'{self.width * 1000:.2f} mm'})")

    rclpy.init(); n = N()
    log = ROOT / "results/physical/hold_test" / datetime.now().strftime("%Y%m%d_%H%M%S"); log.mkdir(parents=True)
    status = {"type": "HOLD TEST (fake)" if a.fake else "HOLD TEST (real)", "args": vars(a), "moves": []}
    try:
        q0 = n.fresh(); n.check()
        print(f"Live start q = {np.round(q0, 4).tolist()}  RG2 width {n.width and round(n.width * 1000, 2)} mm")
        moves = moves_fn(q0)
        preview(moves, q0, calib, "the live robot pose")
        print("\nPart gripped in its pocket? Freedrive OFF? Area above the plates clear? Pendant stop in reach?")
        for p in moves:
            print(f"\n--- {p['name']}: {p['what']} ({p['duration_s']:.1f} s) ---")
            if input(f"Type RUN_{p['name']} to move (anything else stops): ").strip() != f"RUN_{p['name']}":
                raise RuntimeError("operator stopped")
            n.run(p)
            obs = input("  Part still straight and not slipped/turned in the jaws? [y/n/notes] ").strip()
            status["moves"].append({"name": p["name"], "observation": obs, "duration_s": p["duration_s"]})
            if obs.lower().startswith("n"):
                print("  Recorded. Stopping here: the arm stays where it is (lower it by freedrive, or rerun).")
                break
        status["status"] = "DONE"
    except BaseException as e:
        status["status"] = "STOPPED"; status["reason"] = str(e)
        print(f"\nSTOPPED: {e}", file=sys.stderr)
    finally:
        json.dump(status, open(log / "status.json", "w"), indent=1)
        with open(log / "joint_log.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["time", "move"] + JOINTS + ["finger_width"]); w.writerows(n.rec)
        print(f"Log -> {log}")
        n.destroy_node(); rclpy.shutdown()
    return 0 if status["status"] == "DONE" else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lift", type=float, default=80.0, help="mm straight up (default 80; tallest part is 40 mm)")
    ap.add_argument("--carry", type=float, default=50.0, help="mm sideways each way along base x (0 = skip)")
    ap.add_argument("--twist", type=float, default=20.0, help="deg wrist_3 each way (0 = skip)")
    ap.add_argument("--no-lower", action="store_true", help="stay lifted at the end")
    ap.add_argument("--speed", type=float, default=0.14, help="peak joint speed rad/s (default 0.14 = runner guard)")
    ap.add_argument("--execute", action="store_true", help="send motion (staged); default is an offline preview")
    ap.add_argument("--fake", action="store_true", help="with --execute: fake hardware on ROS_DOMAIN_ID=73")
    a = ap.parse_args()
    if not 0 < a.speed <= HARD_MAX_SPEED:
        raise SystemExit(f"--speed must be in (0, {HARD_MAX_SPEED}]")
    if not 0 < a.lift <= 150 or not 0 <= a.carry <= 100 or not 0 <= a.twist <= 45:
        raise SystemExit("limits: lift (0,150] mm, carry [0,100] mm, twist [0,45] deg")
    calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
    if a.execute:
        return execute(lambda q0: plan(q0, calib, a), calib, a)
    src = ROOT / "config/saved_poses/v1_pickup_V1_geometry.yaml"
    q0 = yaml.safe_load(open(src))["arm"]["positions_rad"]
    moves = plan(q0, calib, a)
    preview(moves, q0, calib, src.name)
    print("\nNo motion. On the robot: grip the part in its pocket, end freedrive, then --execute.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
