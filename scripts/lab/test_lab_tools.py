#!/usr/bin/env python3
"""Offline tests (no ROS) for the pre-lab tools: TOOL tilt maths, the calibrate_plates tilt/redo flow, the
pickup-consistency summary and the physical runner's wrench RECORDING helpers.

  python3 scripts/lab/test_lab_tools.py          # plain python, prints PASS/FAIL per test, exit 1 on failure
  python3 -m pytest scripts/lab/test_lab_tools.py
Needs numpy, pyyaml (scipy only for the quaternion cross-check, skipped if absent).
"""
import builtins
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for d in ("scripts/lab", "scripts/scene", "scripts/phase11_physical"):
    sys.path.insert(0, str(ROOT / d))
import tool_tilt as tt  # noqa: E402


def Rx(a):
    c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


UP = Rx(math.pi)                      # tool upright: tool z = base -z
close = lambda a, b, tol=1e-9: abs(a - b) <= tol  # noqa: E731


def mat_to_quat(R):
    """Quaternion [x, y, z, w] from a rotation matrix (Shepperd)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2; w = s / 4
        x, y, z = (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            x, y, z, w = s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
        elif i == 1:
            s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            x, y, z, w = (R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
        else:
            s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            x, y, z, w = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s
    return [float(x), float(y), float(z), float(w)]


# ------------------------------------------------------------------ 1. TOOL tilt maths
def test_upright_is_zero():
    for R in (UP, Ry(math.pi)):
        t = tt.tool_tilt_from_matrix(R)
        assert close(t["tool_tilt_deg"], 0, 1e-6) and close(t["lean_about_base_x_deg"], 0, 1e-6) \
            and close(t["lean_about_base_y_deg"], 0, 1e-6), t
    assert close(tt.tool_tilt_from_quat([0, 1, 0, 0])["tool_tilt_deg"], 0, 1e-6)


def test_known_tilt_about_base_x():
    for deg in (0.5, 1.0, 2.4, -3.0, 30.0, -89.0):
        t = tt.tool_tilt_from_matrix(Rx(math.radians(deg)) @ UP)
        assert close(t["tool_tilt_deg"], abs(deg), 1e-6), (deg, t)
        assert close(t["lean_about_base_x_deg"], deg, 1e-6) and close(t["lean_about_base_y_deg"], 0, 1e-6), (deg, t)
        if deg > 0:                                    # sign convention: fingertips swing toward base +y
            assert t["tool_z_in_base"][1] > 0


def test_known_tilt_about_base_y():
    for deg in (0.5, 1.0, 2.4, -3.0, 45.0):
        t = tt.tool_tilt_from_matrix(Ry(math.radians(deg)) @ UP)
        assert close(t["tool_tilt_deg"], abs(deg), 1e-6), (deg, t)
        assert close(t["lean_about_base_y_deg"], deg, 1e-6) and close(t["lean_about_base_x_deg"], 0, 1e-6), (deg, t)
        if deg > 0:                                    # fingertips swing toward base -x
            assert t["tool_z_in_base"][0] < 0


def test_combined_tilt_magnitude():
    R = Rx(math.radians(1.5)) @ Ry(math.radians(-2.0)) @ UP
    z = R[:, 2]
    expect = math.degrees(math.acos(-z[2]))
    t = tt.tool_tilt_from_matrix(R)
    assert close(t["tool_tilt_deg"], expect, 1e-9) and 2.0 < t["tool_tilt_deg"] < 3.6


def test_yaw_only_is_zero_tilt():
    for psi in (0.3, 1.7, -2.9, math.pi):
        for R in (UP @ Rz(psi), Rz(psi) @ UP):        # yaw about the tool axis and about base z
            t = tt.tool_tilt_from_matrix(R)
            assert close(t["tool_tilt_deg"], 0, 1e-6), (psi, t)
            assert close(t["lean_about_base_x_deg"], 0, 1e-6) and close(t["lean_about_base_y_deg"], 0, 1e-6)
        t = tt.tool_tilt_from_matrix(Rz(psi) @ Rx(math.radians(2)) @ UP)   # tilt magnitude survives a base yaw
        assert close(t["tool_tilt_deg"], 2.0, 1e-6)


def test_inverted_is_180():
    for R in (np.eye(3), Rz(1.0)):
        assert close(tt.tool_tilt_from_matrix(R)["tool_tilt_deg"], 180.0, 1e-6)
    assert close(tt.tool_tilt_from_quat([0, 0, 0, 1])["tool_tilt_deg"], 180.0, 1e-6)


def test_never_near_180_for_upright_family():
    rng = np.random.default_rng(0)
    for _ in range(200):
        a, b, c = rng.uniform(-math.radians(5), math.radians(5), 2).tolist() + [rng.uniform(-math.pi, math.pi)]
        t = tt.tool_tilt_from_matrix(Rz(c) @ Rx(a) @ Ry(b) @ UP @ Rz(c))
        assert t["tool_tilt_deg"] < 7.1, t


def test_quaternion_path_matches_matrix():
    rng = np.random.default_rng(1)
    for _ in range(100):
        R = Rz(rng.uniform(-3, 3)) @ Rx(rng.uniform(-0.1, 0.1)) @ Ry(rng.uniform(-0.1, 0.1)) @ UP
        q = mat_to_quat(R)
        assert np.allclose(tt.quat_xyzw_to_matrix(q), R, atol=1e-9)
        q2 = [-v for v in q]                            # q and -q are the same rotation
        assert close(tt.tool_tilt_from_quat(q2)["tool_tilt_deg"], tt.tool_tilt_from_matrix(R)["tool_tilt_deg"], 1e-9)
    try:
        from scipy.spatial.transform import Rotation
    except ImportError:
        return
    for _ in range(50):
        q = Rotation.random(random_state=int(rng.integers(1e6))).as_quat()
        assert np.allclose(tt.quat_xyzw_to_matrix(q), Rotation.from_quat(q).as_matrix(), atol=1e-9)


def test_axis_definition_against_robot_model():
    """gripper_tcp z from the URDF chain (ur_kinematics FK: tool0 -> onrobot_base_link yaw -pi/2 -> +0.218 z)
    equals the z of every recorded gripper_tcp quaternion, and points DOWN for the recorded poses."""
    import ur_kinematics as uk
    calib = yaml.safe_load(open(ROOT / "config/ur3e_lab_calibration.yaml"))
    n = 0
    for f in sorted((ROOT / "config/saved_poses").glob("*.yaml")):
        d = yaml.safe_load(open(f))
        if not (d.get("tcp") or {}).get("quaternion_xyzw") or d.get("tool_frame") != "gripper_tcp":
            continue
        T = uk.tcp(d["arm"]["positions_rad"], calib)
        z_fk = T[:3, 2]
        z_saved = tt.quat_xyzw_to_matrix(d["tcp"]["quaternion_xyzw"])[:, 2]
        ang = math.degrees(math.acos(float(np.clip(z_fk @ z_saved, -1, 1))))
        assert ang < 0.05, (f.name, ang)
        assert z_saved[2] < -0.99, (f.name, z_saved)
        n += 1
    assert n >= 5, n
    z = tt.quat_xyzw_to_matrix(yaml.safe_load(open(ROOT / "config/saved_poses/v1_pickup_V1_geometry.yaml"))
                               ["tcp"]["quaternion_xyzw"])[:, 2]
    assert np.allclose(z, [0.005, -0.016, -0.9999], atol=0.002), z


def test_old_pickup_poses_report_08_to_24_deg():
    vals = [tt.tool_tilt_from_pose_yaml(ROOT / f"config/saved_poses/v1_pickup_V{i}_geometry.yaml")["tool_tilt_deg"]
            for i in range(1, 6)]
    assert 0.75 <= min(vals) and max(vals) <= 2.5, vals


# ------------------------------------------------------------------ 2. calibrate_plates tilt / redo flow
def _calib_env(answers, quats, fail_first=0):
    """Patch calibrate_plates: save_named_pose writes the next quaternion; input() returns scripted answers."""
    import calibrate_plates as cp
    seq = list(quats); ans = list(answers); calls = {"n": 0}

    def fake_run(cmd, *a, **k):
        calls["n"] += 1
        if calls["n"] <= fail_first:
            return subprocess.CompletedProcess(cmd, 1)
        name = cmd[4]; out = [x for x in cmd if x.startswith("output_dir:=")][0].split(":=", 1)[1]
        with open(os.path.join(out, f"{name}.yaml"), "w") as f:
            yaml.safe_dump({"name": name, "tcp": {"position_m": [0.2, -0.3, 0.03], "quaternion_xyzw": seq.pop(0)}}, f)
        return subprocess.CompletedProcess(cmd, 0)

    def fake_input(msg=""):
        return ans.pop(0)
    return cp, fake_run, fake_input, ans, seq


def _with_patches(cp, fake_run, fake_input, fn):
    orig_run, orig_input = cp.subprocess.run, builtins.input
    cp.subprocess.run, builtins.input = fake_run, fake_input
    try:
        return fn()
    finally:
        cp.subprocess.run, builtins.input = orig_run, orig_input


def q_tilt_x(deg):
    return mat_to_quat(Rx(math.radians(deg)) @ UP)


def test_calib_upright_no_redo_prompt():
    with tempfile.TemporaryDirectory() as s:
        cp, run, inp, ans, _ = _calib_env([], [q_tilt_x(0.3)])
        notes = {}; nf = os.path.join(s, "notes.yaml")
        assert _with_patches(cp, run, inp, lambda: cp.save_with_tilt(s, "v1_pickup_V1_geometry", notes, nf))
        e = yaml.safe_load(open(nf))["tool_tilt"]["v1_pickup_V1_geometry"]
        assert e["status"] == "saved" and close(e["tool_tilt_deg"], 0.3, 1e-3) and not e["over_warn_1deg"]
        assert close(e["lean_about_base_x_deg"], 0.3, 1e-3)


def test_calib_tilted_redo_uses_new_pose():
    with tempfile.TemporaryDirectory() as s:
        cp, run, inp, ans, seq = _calib_env(["y"], [q_tilt_x(2.0), q_tilt_x(0.4)])
        notes = {}; nf = os.path.join(s, "notes.yaml")
        assert _with_patches(cp, run, inp, lambda: cp.save_with_tilt(s, "v1_pickup_V2_geometry", notes, nf))
        e = yaml.safe_load(open(nf))["tool_tilt"]["v1_pickup_V2_geometry"]
        assert close(e["tool_tilt_deg"], 0.4, 1e-3) and not e["over_warn_1deg"], e
        assert any(".replaced_" in f for f in os.listdir(s)) and not ans and not seq
        # the kept file is the redo, not the first save
        assert close(tt.tool_tilt_from_pose_yaml(os.path.join(s, "v1_pickup_V2_geometry.yaml"))["tool_tilt_deg"], 0.4, 1e-3)


def test_calib_tilted_kept_is_recorded():
    with tempfile.TemporaryDirectory() as s:
        cp, run, inp, ans, _ = _calib_env(["n"], [q_tilt_x(-1.8)])
        notes = {}; nf = os.path.join(s, "notes.yaml")
        assert _with_patches(cp, run, inp, lambda: cp.save_with_tilt(s, "v1_place_V3_geometry", notes, nf))
        e = yaml.safe_load(open(nf))["tool_tilt"]["v1_place_V3_geometry"]
        assert e["over_warn_1deg"] and e["operator_kept_despite_warning"] and close(e["lean_about_base_x_deg"], -1.8, 1e-3)


def test_calib_resume_kept_existing_session_pose():
    """--resume: the operator answers N to 'already saved. Redo it?': that session file is reported as kept."""
    import time
    with tempfile.TemporaryDirectory() as s:
        f = os.path.join(s, "v1_pickup_V1_geometry.yaml")
        yaml.safe_dump({"tcp": {"quaternion_xyzw": q_tilt_x(0.6)}}, open(f, "w"))
        old = time.time() - 60; os.utime(f, (old, old))
        cp, run, inp, ans, seq = _calib_env(["n"], [])
        notes = {}; nf = os.path.join(s, "notes.yaml")
        assert _with_patches(cp, run, inp, lambda: cp.save_with_tilt(s, "v1_pickup_V1_geometry", notes, nf))
        e = yaml.safe_load(open(nf))["tool_tilt"]["v1_pickup_V1_geometry"]
        assert e["status"] == "kept_existing" and close(e["tool_tilt_deg"], 0.6, 1e-3)


def test_calib_failed_save_skip_is_missing_no_fallback():
    with tempfile.TemporaryDirectory() as s:
        cp, run, inp, ans, _ = _calib_env(["r", "s"], [], fail_first=99)
        notes = {}; nf = os.path.join(s, "notes.yaml")
        assert not _with_patches(cp, run, inp, lambda: cp.save_with_tilt(s, "v1_pickup_V4_geometry", notes, nf))
        e = yaml.safe_load(open(nf))["tool_tilt"]["v1_pickup_V4_geometry"]
        assert e["status"] == "missing" and "tool_tilt_deg" not in e
        rep = cp.tilt_report(s)                 # report never reads config/saved_poses
        assert rep["pickup_tool_tilt_deg"]["V4"] is None and rep["max_pickup_tool_tilt_deg"] is None


def test_calib_operator_skip_is_missing():
    import calibrate_plates as cp
    with tempfile.TemporaryDirectory() as s:
        notes = {}; nf = os.path.join(s, "notes.yaml")
        cp.note_skipped(s, "v1_place_V5_geometry", notes, nf)
        assert yaml.safe_load(open(nf))["tool_tilt"]["v1_place_V5_geometry"]["status"] == "missing"


def test_calib_limits_unchanged():
    import calibrate_plates as cp
    assert cp.LIMITS == {"pickup_fit_rms_mm": 1.5, "placement_fit_rms_mm": 0.5, "prong_mismatch_mm": 1.5,
                         "plate_yaw_diff_deg": 0.5}


# ------------------------------------------------------------------ 3. pickup consistency
def test_pickup_summary():
    import pickup_consistency as pc
    q = mat_to_quat(UP)
    rows = []
    for k, (dx, w) in enumerate([(0.0, 0.0101), (0.4, 0.0102), (-0.2, 0.0100)], 1):
        rows.append(pc.trial_row(k, "0", [((0.2284 + dx / 1000, -0.3476, 0.0357), q)] * 3, [w, w]))
        rows[-1].update(part_shifted_while_closing="n", part_tilted_after_lifting="y" if k == 2 else "n",
                        pickup_plate_lifted="n")
    rows.append(pc.trial_row(4, "0", [], []))          # no data at all: must NOT become zeros
    s = pc.summarise(rows)
    assert s["tcp_x_mm"]["n"] == 3 and close(s["tcp_x_mm"]["range"], 0.6, 1e-6)
    assert close(s["rg2_width_mm"]["mean"], 10.1, 1e-6) and close(s["rg2_width_mm"]["range"], 0.2, 1e-6)
    assert s["tool_tilt_deg"]["max"] < 1e-6 and s["trials_without_tcp"] == 1 and s["trials_without_width"] == 1
    assert rows[3]["tcp_x_mm"] == "" and rows[3]["rg2_width_mm"] == "" and "unavailable" in rows[3]["data_note"]
    assert s["observations_yes"]["part_tilted_after_lifting"] == 1
    assert "SETUP CONSISTENCY ONLY" in s["scope"] and "does NOT measure part-to-gripper" in s["scope"]


def test_pickup_yaw_wrap():
    import pickup_consistency as pc
    rows = [{"tool_yaw_deg": 179.5}, {"tool_yaw_deg": -179.7}]
    assert close(pc.summarise(rows)["tool_yaw_range_deg"], 0.8, 1e-6)


def test_pickup_readonly_source():
    src = (HERE / "pickup_consistency.py").read_text() + (HERE / "tilt_meter.py").read_text()
    for bad in ("rg2.sh grip\"", "subprocess", "FollowJointTrajectory", "ActionClient", "create_publisher",
                "send_goal", "switch_controllers"):
        assert bad not in src, bad


# ------------------------------------------------------------------ 4. runner wrench RECORDING helpers
def _rows(n, t0=100.0, dt=0.002, f=(1.0, 2.0, 2.0), tq=(0.1, 0.0, 0.0), stage="INSERT"):
    return [{"time": "", "recv_s": t0 + k * dt, "stage": stage, "stamp_s": 5.0, "age_s": 0.001, "frame_id": "tool0",
             "fx": f[0], "fy": f[1], "fz": f[2] + (0.5 if k == n // 2 else 0.0), "tx": tq[0], "ty": tq[1], "tz": tq[2]}
            for k in range(n)]


def test_wrench_summary_values():
    import run_full_cycle_physical as r
    s = r.wrench_stage_summary(_rows(500), 100.0, 101.0)
    assert s["available"] and s["n_samples"] == 500 and s["frame_ids"] == ["tool0"]
    assert close(s["max_abs_force_N"], round(math.sqrt(1 + 4 + 2.5 ** 2), 3), 1e-9)
    assert close(s["max_abs_torque_Nm"], 0.1, 1e-9) and close(s["max_abs_force_change_from_stage_start_N"], 0.5, 1e-9)
    assert s["stale"] is False


def test_wrench_unavailable_is_none_not_zero():
    import run_full_cycle_physical as r
    s = r.wrench_stage_summary([], 0.0, 5.0)
    assert s["available"] is False and s["max_abs_force_N"] is None and s["max_abs_torque_Nm"] is None
    assert s["stale"] is None and "UNAVAILABLE" in r.wrench_line("X", s)
    bad = _rows(3); [row.update(fx=float("nan")) for row in bad]
    s = r.wrench_stage_summary(bad, 99.0, 101.0)
    assert s["available"] is False and s["n_nonfinite"] == 3 and s["max_abs_force_N"] is None


def test_wrench_stale_detection():
    import run_full_cycle_physical as r
    rows = _rows(10, t0=100.0, dt=0.002)                 # data only in the first 20 ms of a 2 s stage
    s = r.wrench_stage_summary(rows, 100.0, 102.0)
    assert s["available"] and s["stale"] is True and s["max_gap_s"] > 1.9
    old = _rows(100, t0=100.0, dt=0.01); [row.update(age_s=0.5) for row in old]
    assert r.wrench_stage_summary(old, 100.0, 101.0)["stale"] is True


def test_wrench_log_files():
    import run_full_cycle_physical as r
    with tempfile.TemporaryDirectory() as d:
        rows = _rows(3); rows.append({**_rows(1)[0], "fx": float("nan"), "age_s": None, "stamp_s": None})
        r.write_wrench_logs(Path(d), {"topic": r.WRENCH_TOPIC, "stages": []}, rows)
        lines = open(os.path.join(d, "wrench_log.csv")).read().splitlines()
        assert lines[0].split(",")[-6:] == r.WRENCH_FIELDS and len(lines) == 5
        last = lines[-1].split(",")
        assert last[3] == "" and last[4] == "" and last[6] == ""      # unavailable -> empty, never 0
        assert json.load(open(os.path.join(d, "wrench_summary.json")))["topic"] == "/force_torque_sensor_broadcaster/wrench"


def test_runner_guards_unchanged():
    import run_full_cycle_physical as r
    assert (r.START_TOL, r.END_TOL, r.HARD_MAX_SPEED, r.MAX_POINT_STEP) == (0.006, 0.012, 0.35, 0.09)
    src = (ROOT / "scripts/phase11_physical/run_full_cycle_physical.py").read_text()
    assert 'ap.add_argument("--max-joint-speed", type=float, default=0.14)' in src
    assert 'f"RUN_{label}"' in src


# ------------------------------------------------------------------ 5. --help of every new / changed script
def test_help_works():
    for p in ("scripts/lab/tilt_meter.py", "scripts/lab/pickup_consistency.py", "scripts/scene/calibrate_plates.py",
              "scripts/phase11_physical/run_full_cycle_physical.py", "scripts/lab/test_lab_tools.py"):
        r = subprocess.run([sys.executable, str(ROOT / p), "--help"], capture_output=True, text=True, timeout=60)
        assert r.returncode == 0 and "usage" in r.stdout.lower() + r.stderr.lower() or "Offline tests" in r.stdout, p


# ------------------------------------------------------------------ 6. ROS paths of the two new tools, with fake rclpy/tf2
def _fake_ros_modules(quat, width=0.0101, ok_calls=40, tf_fail=False):
    import types
    W = lambda **k: types.SimpleNamespace(**k)  # noqa: E731
    clock = {"ns": 10_000_000_000, "ok": ok_calls}
    subs = []

    class Node:
        def create_subscription(self, typ, topic, cb, qos):
            subs.append(cb)

        def get_clock(self):
            return W(now=lambda: W(nanoseconds=clock["ns"]))

        def destroy_node(self):
            pass

    def spin_once(node, timeout_sec=None):
        clock["ns"] += 20_000_000
        for cb in subs:
            cb(W(name=["shoulder_pan_joint", "finger_width"], position=[0.0, width]))

    def ok():
        clock["ok"] -= 1
        return clock["ok"] > 0

    class Time:
        def __init__(self, *a, **k):
            self.ns = 0

        @staticmethod
        def from_msg(m):
            t = Time(); t.ns = m.sec * 1_000_000_000 + m.nanosec; return t

        def __rsub__(self, other):
            return W(nanoseconds=other.nanoseconds - self.ns)

    class Buffer:
        def lookup_transform(self, a, b, t, timeout=None):
            if tf_fail:
                raise RuntimeError("no tf")
            ns = clock["ns"] - 5_000_000
            return W(header=W(stamp=W(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)),
                     transform=W(translation=W(x=0.2284, y=-0.3476, z=0.0357),
                                 rotation=W(x=quat[0], y=quat[1], z=quat[2], w=quat[3])))

    mods = {"rclpy": W(init=lambda: None, shutdown=lambda: None, ok=ok, spin_once=spin_once,
                       create_node=lambda name: Node()),
            "rclpy.duration": W(Duration=lambda seconds=0: None), "rclpy.time": W(Time=Time),
            "rclpy.qos": W(qos_profile_sensor_data=None), "sensor_msgs": W(), "sensor_msgs.msg": W(JointState=object),
            "tf2_ros": W(Buffer=Buffer, TransformListener=lambda buf, node: None)}
    return mods


def _run_with_modules(mods, fn):
    saved = {k: sys.modules.get(k) for k in mods}
    sys.modules.update(mods)
    try:
        return fn()
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def test_pickup_consistency_fake_ros_run():
    import argparse
    import pickup_consistency as pc
    q = q_tilt_x(1.5)
    answers = ["", "n", "y", "n", "note A", "", "s", "", "n", "n", "n", "", ""]
    with tempfile.TemporaryDirectory() as d:
        a = argparse.Namespace(trials=3, variant=1, window=0.2, base_frame="base", tool_frame="gripper_tcp",
                               out=os.path.join(d, "out"))
        orig = builtins.input; builtins.input = lambda m="": answers.pop(0)
        try:
            assert _run_with_modules(_fake_ros_modules(q, ok_calls=10**6), lambda: pc.run(a)) == 0
        finally:
            builtins.input = orig
        rows = list(__import__("csv").DictReader(open(os.path.join(d, "out", "trials.csv"))))
        assert [r["trial"] for r in rows] == ["1", "3"], rows                     # trial 2 skipped
        assert close(float(rows[0]["tool_tilt_deg"]), 1.5, 1e-3) and close(float(rows[0]["rg2_width_mm"]), 10.1, 1e-6)
        assert rows[0]["part_tilted_after_lifting"] == "y" and rows[0]["operator_note"] == "note A"
        s = yaml.safe_load(open(os.path.join(d, "out", "summary.yaml")))
        assert s["trials_total"] == 2 and "SETUP CONSISTENCY ONLY" in s["scope"] and s["variant"] == "V1"


def test_pickup_consistency_fake_ros_no_data_is_unavailable():
    import argparse
    import pickup_consistency as pc
    answers = ["", "n", "n", "n", "", ""]
    with tempfile.TemporaryDirectory() as d:
        a = argparse.Namespace(trials=1, variant=1, window=0.2, base_frame="base", tool_frame="gripper_tcp",
                               out=os.path.join(d, "out"))
        orig = builtins.input; builtins.input = lambda m="": answers.pop(0)
        try:
            mods = _fake_ros_modules(q_tilt_x(0), width=float("nan"), ok_calls=10**6, tf_fail=True)
            assert _run_with_modules(mods, lambda: pc.run(a)) == 0
        finally:
            builtins.input = orig
        r = list(__import__("csv").DictReader(open(os.path.join(d, "out", "trials.csv"))))[0]
        assert r["tcp_x_mm"] == "" and r["rg2_width_mm"] == "" and r["tool_tilt_deg"] == "" and "unavailable" in r["data_note"]


def test_tilt_meter_fake_ros_prints():
    import io
    import contextlib
    import tilt_meter as tm
    buf = io.StringIO()
    argv = sys.argv; sys.argv = ["tilt_meter.py", "--rate", "20"]
    try:
        with contextlib.redirect_stdout(buf):
            assert _run_with_modules(_fake_ros_modules(q_tilt_x(-2.0), ok_calls=200), tm.main) == 0
    finally:
        sys.argv = argv
    out = buf.getvalue()
    assert "TOOL tilt   2.00 deg" in out and "lean about base x  -2.00" in out and "TILTED > 1.0 deg" in out, out[-500:]
    assert "STALE" not in out and "NOT the part's tilt" in out


def run_all():
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    fails = 0
    for name, fn in tests:
        try:
            fn(); print(f"PASS {name}")
        except Exception as e:  # noqa: BLE001
            fails += 1; print(f"FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - fails}/{len(tests)} passed")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--help" in sys.argv or "-h" in sys.argv:
        print(__doc__); sys.exit(0)
    sys.exit(run_all())
