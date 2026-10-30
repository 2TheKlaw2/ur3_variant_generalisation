#!/usr/bin/env python3
"""Guided, READ-ONLY plate calibration for the lab (sends NO motion commands to the robot).

  bash CALIBRATE_PLATES.sh                      # new dated session config/calibration/<YYYYmmdd_HHMM>/
  bash CALIBRATE_PLATES.sh --resume <session>   # continue / redo poses in an existing session
  python3 scripts/scene/calibrate_plates.py --test-from-saved   # offline self-test, no ROS

Protocol (per variant i = 1..5, part i is its own probe, so its grasp offset cancels):
  1. With the RG2 open, freedrive the gripper onto part i in its pocket exactly as for pickup
     (fingers centred on the grip block, tool vertical); close the RG2 from the pendant.
     -> saves v1_pickup_V<i>_geometry
  2. Lift the part out, freedrive it into fitting i until the bore centres it (tabs on the slots,
     entry depth only, NO twist, never force it).                 -> saves v1_place_V<i>_geometry
  3. Lift it back out, put it back in its pocket, open the RG2.
Then the plates are fitted (dry run, nothing applied) and checked against limits. The old poses in
config/saved_poses are never touched. Apply the result with UPDATE_SIM.sh <session>.
"""
import argparse
import datetime
import os
import shutil
import subprocess
import sys

import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LIMITS = {"pickup_fit_rms_mm": 1.5, "placement_fit_rms_mm": 0.5, "prong_mismatch_mm": 1.5, "plate_yaw_diff_deg": 0.5}


def ask(msg, choices="", default=""):
    while True:
        r = input(msg).strip().lower() or default
        if not choices or r in choices:
            return r


def save_pose(session, name):
    f = os.path.join(session, f"{name}.yaml")
    if os.path.exists(f):
        if ask(f"  {name} already saved in this session. Redo it? [y/N] ", "yn", "n") != "y":
            return True
        os.rename(f, f + ".replaced_" + datetime.datetime.now().strftime("%H%M%S"))
    r = subprocess.run(["ros2", "run", "ur3_variant_generalisation", "save_named_pose", name, "--ros-args",
                        "-p", f"output_dir:={session}", "-p", "base_frame:=base", "-p", "tool_frame:=gripper_tcp"])
    ok = r.returncode == 0 and os.path.exists(f)
    print(f"  {'SAVED' if ok else 'NOT SAVED'}: {f}")
    return ok


def fit_and_check(session, thickness_m):
    cmd = [sys.executable, os.path.join(PKG, "scripts/scene/compute_scene_registration.py"), "--session", session,
           "--dry-run", "--fixture-thickness", str(thickness_m)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:], r.stderr[-2000:]); return None
    cand = yaml.safe_load(open(os.path.join(session, "scene_registration_candidate.yaml")))
    old = yaml.safe_load(open(os.path.join(PKG, "config/scene/scene_registration.yaml")))
    rep = {"pickup_fit_rms_mm": cand["pickup_plate"].get("fit_rms_mm"),
           "placement_fit_rms_mm": cand["placement_plate"].get("fit_rms_mm"),
           "prong_mismatch_mm": abs((cand.get("prong_check") or {}).get("placement_prong_tip_from_far_edge_mm", 0.0)),
           "plate_yaw_diff_deg": abs(cand["pickup_plate"]["yaw_deg"] - cand["placement_plate"]["yaw_deg"])}
    print("\n================ CALIBRATION RESULT (dry run, nothing applied) ================")
    ok_all = True
    for k, lim in LIMITS.items():
        v = rep[k]
        ok = v is not None and v <= lim
        ok_all &= ok
        print(f"  [{'PASS' if ok else 'CHECK'}] {k:24s} {v if v is None else round(v, 3)}  (limit {lim})")
    for key in ("pickup_plate", "placement_plate"):
        a, b = cand[key]["xyz_m"], old[key]["xyz_m"]
        d = [round((x - y) * 1000, 2) for x, y in zip(a[:2], b[:2])]
        print(f"  {key}: moved {d} mm (x, y) vs the current simulation ({old.get('layout')} layout); "
              f"yaw {cand[key]['yaw_deg']:.3f} deg; residuals {cand[key].get('fit_residual_mm')}")
    rep["all_within_limits"] = bool(ok_all)
    with open(os.path.join(session, "calibration_report.yaml"), "w") as f:
        yaml.safe_dump(rep, f, sort_keys=False)
    print(f"\n  {'ALL WITHIN LIMITS' if ok_all else 'SOME CHECKS OUTSIDE LIMITS - redo those poses (--resume) or inspect'}")
    print(f"  Apply with:  bash UPDATE_SIM.sh {os.path.relpath(session, PKG)}")
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resume", help="existing session folder or name under config/calibration/")
    ap.add_argument("--variants", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--test-from-saved", action="store_true",
                    help="offline self-test: build a session from config/saved_poses (+ synthetic place poses)")
    a = ap.parse_args()
    root = os.path.join(PKG, "config", "calibration")
    if a.resume:
        session = a.resume if os.path.isdir(a.resume) else os.path.join(root, a.resume)
        if not os.path.isdir(session):
            raise SystemExit(f"no such session: {session}")
    else:
        session = os.path.join(root, datetime.datetime.now().strftime("%Y%m%d_%H%M") + ("_TEST" if a.test_from_saved else ""))
        os.makedirs(session, exist_ok=False)
    notes_f = os.path.join(session, "notes.yaml")
    notes = yaml.safe_load(open(notes_f)) if os.path.exists(notes_f) else {}

    if a.test_from_saved:
        return self_test(session)

    print(f"\nCALIBRATION SESSION: {session}\nREAD-ONLY: this script never moves the robot. You move it in freedrive "
          "and operate the RG2 from the pendant.\n")
    for item in ("Both blue plates are bolted down (M5 through the slots) with their prongs butted against the far edge",
                 "The robot is in freedrive-capable mode, E-stop within reach, speed slider low",
                 "Parts V1-V5 are in their pockets"):
        if ask(f"  [ ] {item}?  [y/n] ", "yn") != "y":
            raise SystemExit("Stopped: fix that first.")
    if "fixture_thickness_mm" not in notes:
        notes["fixture_thickness_mm"] = float(input("  Aluminium fixture plate thickness at its edge [mm]: "))
    notes.setdefault("started", datetime.datetime.now().isoformat(timespec="seconds"))
    yaml.safe_dump(notes, open(notes_f, "w"))

    for i in a.variants:
        print(f"\n---------------- V{i} ----------------")
        print(f"  1) RG2 open. Freedrive the gripper onto part V{i} in its pocket exactly as for pickup "
              "(fingers centred on the grip block, tool vertical). Close the RG2 (8.2 mm) from the pendant.")
        if ask("     Ready to save the PICKUP pose? [Enter = save, s = skip] ", "s", "save") != "s":
            save_pose(session, f"v1_pickup_V{i}_geometry")
        print(f"  2) Lift V{i} out and freedrive it into fitting V{i}: tabs on the slots, entry depth only, "
              "NO twist, never force it. Let the bore centre it.")
        if ask("     Ready to save the PLACE pose? [Enter = save, s = skip] ", "s", "save") != "s":
            save_pose(session, f"v1_place_V{i}_geometry")
        print(f"  3) Lift V{i} back out, return it to its pocket, open the RG2.")
        ask("     Press Enter when V{i} is back in its pocket and the RG2 is open. ")

    fit_and_check(session, notes["fixture_thickness_mm"] / 1000.0)
    return 0


def self_test(session):
    """Copy the old pickup poses and synthesise place poses from the CURRENT model (V<i> in fitting V<i>
    with its own grasp). Expect: pickup RMS = the old 2.07 mm, placement RMS ~0, and a prong mismatch equal
    to the difference of the old plates' far edges (they were not butted)."""
    sys.path[:0] = [os.path.join(PKG, "scripts", d) for d in ("scene", "phase9_variants", "phase8_insert_sim",
                                                            "phase7_transport_sim", "phase6_pickup_sim")]
    import numpy as np
    import scene_model as sm
    import ur_kinematics as uk
    calib = yaml.safe_load(open(os.path.join(PKG, "config/ur3e_lab_calibration.yaml")))
    objs = sm.scene_objects(measured=True)
    for i in range(1, 6):
        src = os.path.join(PKG, "config/saved_poses", f"v1_pickup_V{i}_geometry.yaml")
        shutil.copy(src, os.path.join(session, f"v1_pickup_V{i}_geometry.yaml"))
        q = yaml.safe_load(open(src))["arm"]["positions_rad"]
        G = {i: np.linalg.inv(uk.tcp(q, calib)) @ objs[f"pickup_V{i}"]["T"]}       # this part's own grasp
        T_part = objs[f"fitting_V{i}"]["T"].copy(); T_part[2, 3] += 0.003
        T_tcp = T_part @ np.linalg.inv(G[i])
        from scipy.spatial.transform import Rotation
        doc = {"name": f"v1_place_V{i}_geometry", "base_frame": "base", "tool_frame": "gripper_tcp",
               "note": "SYNTHETIC (self-test): model fitting pose with the part's own grasp",
               "tcp": {"position_m": T_tcp[:3, 3].tolist(), "quaternion_xyzw": Rotation.from_matrix(T_tcp[:3, :3]).as_quat().tolist()}}
        yaml.safe_dump(doc, open(os.path.join(session, f"v1_place_V{i}_geometry.yaml"), "w"))
    yaml.safe_dump({"fixture_thickness_mm": 12.5, "self_test": True}, open(os.path.join(session, "notes.yaml"), "w"))
    rep = fit_and_check(session, 0.0125)
    print("\nSELF-TEST session:", session, "(delete it afterwards; never apply it)")
    return 0 if rep else 1


if __name__ == "__main__":
    sys.exit(main())
