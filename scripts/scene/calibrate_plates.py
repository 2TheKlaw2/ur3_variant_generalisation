#!/usr/bin/env python3
"""Guided, READ-ONLY plate calibration for the lab (sends NO motion commands to the robot).

  bash CALIBRATE_PLATES.sh                      # new dated session config/calibration/<YYYYmmdd_HHMM>/
  bash CALIBRATE_PLATES.sh --resume <session>   # continue / redo poses in an existing session
  python3 scripts/scene/calibrate_plates.py --test-from-saved   # offline self-test, no ROS

Protocol (per variant i = 1..5, part i is its own probe, so its grasp offset cancels):
  1. With the RG2 open, freedrive the gripper onto part i in its pocket exactly as for pickup
     (fingers centred on the grip block, tool vertical); close the RG2 with bash scripts/lab/rg2.sh grip.
     -> saves v1_pickup_V<i>_geometry
  2. Lift the part out, freedrive it into fitting i until the bore centres it (tabs on the slots,
     entry depth only, NO twist, never force it).                 -> saves v1_place_V<i>_geometry
  3. Lift it back out, put it back in its pocket, open the RG2.
After every successful save the TOOL tilt of that pose (gripper_tcp z vs base -z, from the newly saved
yaml) is printed and recorded in the session notes.yaml; above 1.0 deg a redo is recommended and offered.
This is TOOL tilt, not measured part tilt, and it is a warning only: it is not one of the LIMITS.
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
sys.path.insert(0, os.path.join(PKG, "scripts", "lab"))
import tool_tilt as tt  # noqa: E402  (pure numpy, TOOL tilt only)

LIMITS = {"pickup_fit_rms_mm": 1.5, "placement_fit_rms_mm": 0.5, "prong_mismatch_mm": 1.5, "plate_yaw_diff_deg": 0.5}


def ask(msg, choices="", default=""):
    while True:
        r = input(msg).strip().lower() or default
        # Enter returns the default even when it is not one of the single-letter choices (e.g. "save");
        # list(choices) so a typed word is never accepted by a substring match.
        if not choices or r == default or r in list(choices):
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


def save_checked(session, name):
    """Save a pose; on failure ask to retry / skip / quit, so no step continues on a missing or old pose."""
    while not save_pose(session, name):
        r = ask(f"  Saving {name} FAILED (driver running? /joint_states?). [r]etry / [s]kip / [q]uit: ", "rsq", "r")
        if r == "s":
            print(f"  SKIPPED: {name} is missing; redo it later with --resume")
            return False
        if r == "q":
            raise SystemExit(f"Stopped. Resume with: bash CALIBRATE_PLATES.sh --resume {os.path.basename(session)}")
    return True


# ------------------------------------------------------------------ TOOL tilt (read-only, warning only)
def tilt_report(session):
    """TOOL tilt of every pickup pose saved in this session (read from the session yamls only)."""
    per = {}
    for i in range(1, 6):
        f = os.path.join(session, f"v1_pickup_V{i}_geometry.yaml")
        try:
            per[f"V{i}"] = round(tt.tool_tilt_from_pose_yaml(f)["tool_tilt_deg"], 3) if os.path.exists(f) else None
        except ValueError:
            per[f"V{i}"] = None
    vals = [v for v in per.values() if v is not None]
    return {"pickup_tool_tilt_deg": per,
            "max_pickup_tool_tilt_deg": max(vals) if vals else None,
            "tool_tilt_warn_deg": tt.WARN_DEG,
            "tool_tilt_note": "TOOL tilt (gripper_tcp z vs base -z) from the saved TCP orientation; NOT measured part "
                              "tilt; a warning only, not a LIMIT and not part of all_within_limits; None = pose missing"}


def write_notes(notes_f, notes):
    with open(notes_f, "w") as f:
        yaml.safe_dump(notes, f, sort_keys=False)


def record_tilt(notes, name, entry):
    notes.setdefault("tool_tilt", {})[name] = entry


def save_with_tilt(session, name, notes, notes_f):
    """save_checked(), then the TOOL tilt of the NEWLY saved yaml; warn > 1 deg and offer a redo.

    Never reads any older pose: only the file save_checked() just wrote in this session. If the operator
    kept an existing session pose at save_pose()'s 'Redo it?' prompt, that session file is reported as kept.
    A skipped save is recorded in notes.yaml as missing."""
    f = os.path.join(session, f"{name}.yaml")
    while True:
        before = os.path.getmtime(f) if os.path.exists(f) else None
        t_call = datetime.datetime.now().timestamp()
        if not save_checked(session, name):
            record_tilt(notes, name, {"status": "missing", "reason": "save failed and was skipped",
                                      "at": datetime.datetime.now().isoformat(timespec="seconds")})
            write_notes(notes_f, notes)
            return False
        if not os.path.exists(f):            # cannot happen after save_checked() == True; never fall back
            record_tilt(notes, name, {"status": "missing", "reason": "file absent after save"})
            write_notes(notes_f, notes)
            return False
        after = os.path.getmtime(f)
        kept = before is not None and after == before and after < t_call
        try:
            t = tt.tool_tilt_from_pose_yaml(f)
        except ValueError as e:
            print(f"  TOOL tilt: cannot read tcp.quaternion_xyzw from {os.path.basename(f)} ({e})")
            record_tilt(notes, name, {"status": "kept_existing" if kept else "saved", "tool_tilt_deg": None,
                                      "reason": "no tcp.quaternion_xyzw in the saved yaml"})
            write_notes(notes_f, notes)
            return True
        warn = t["tool_tilt_deg"] > tt.WARN_DEG
        record_tilt(notes, name, {"status": "kept_existing" if kept else "saved",
                                  "tool_tilt_deg": round(t["tool_tilt_deg"], 3),
                                  "lean_about_base_x_deg": round(t["lean_about_base_x_deg"], 3),
                                  "lean_about_base_y_deg": round(t["lean_about_base_y_deg"], 3),
                                  "over_warn_1deg": bool(warn),
                                  "at": datetime.datetime.now().isoformat(timespec="seconds")})
        write_notes(notes_f, notes)
        print(f"  {name}: {tt.fmt(t)}{'  [kept existing session pose]' if kept else ''}")
        if not warn:
            return True
        print(f"  WARNING: TOOL tilt {t['tool_tilt_deg']:.2f} deg > {tt.WARN_DEG} deg: redo recommended "
              "(straighten the tool, then save again). This is TOOL tilt, not measured part tilt.")
        if ask("  Redo this pose now? [y/N] ", "yn", "n") != "y":
            notes["tool_tilt"][name]["operator_kept_despite_warning"] = True
            write_notes(notes_f, notes)
            return True
        os.rename(f, f + ".replaced_" + datetime.datetime.now().strftime("%H%M%S"))


def note_skipped(session, name, notes, notes_f):
    """Operator skipped this pose: missing unless this session already holds one (then its entry is kept)."""
    if not os.path.exists(os.path.join(session, f"{name}.yaml")):
        record_tilt(notes, name, {"status": "missing", "reason": "skipped by the operator",
                                  "at": datetime.datetime.now().isoformat(timespec="seconds")})
        write_notes(notes_f, notes)


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
    rep.update(tilt_report(session))            # TOOL tilt: reported only, not a limit, not in all_within_limits
    print(f"  TOOL tilt (gripper_tcp z vs base -z; NOT part tilt; warning > {tt.WARN_DEG} deg, not a limit):")
    for k, v in rep["pickup_tool_tilt_deg"].items():
        print(f"    {k}: {'MISSING' if v is None else f'{v:.2f} deg'}"
              f"{'  (redo recommended)' if v is not None and v > tt.WARN_DEG else ''}")
    print(f"    max pickup TOOL tilt: {rep['max_pickup_tool_tilt_deg']} deg")
    with open(os.path.join(session, "calibration_report.yaml"), "w") as f:
        yaml.safe_dump(rep, f, sort_keys=False)
    print(f"\n  {'ALL WITHIN LIMITS' if ok_all else 'SOME CHECKS OUTSIDE LIMITS - redo those poses (--resume) or inspect'}")
    print("  NOTE: these limits measure how well the poses fit the rigid plate model. They do NOT prove insertion accuracy\n"
          "  (radial clearance 0.30-0.35 mm): the inspection above the fitting at ALIGN_INSERT still decides.")
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

    print(f"\nCALIBRATION SESSION: {session}\nREAD-ONLY: this script never moves the robot. Freedrive: bash scripts/lab/freedrive.sh "
          "(other terminal); gripper: bash scripts/lab/rg2.sh grip|open.\n")
    for item in ("Both blue plates are bolted down (M5 through the slots) with their prongs butted against the far edge",
                 "Freedrive is on (scripts/lab/freedrive.sh running), E-stop within reach",
                 "Parts V1-V5 are in their pockets"):
        if ask(f"  [ ] {item}?  [y/n] ", "yn") != "y":
            raise SystemExit("Stopped: fix that first.")
    if "fixture_thickness_mm" not in notes:
        notes["fixture_thickness_mm"] = float(input("  Aluminium fixture plate thickness at its edge [mm]: "))
    notes.setdefault("started", datetime.datetime.now().isoformat(timespec="seconds"))
    write_notes(notes_f, notes)

    for i in a.variants:
        print(f"\n---------------- V{i} ----------------")
        print(f"  1) RG2 open. Freedrive the gripper onto part V{i} in its pocket exactly as for pickup "
              "(fingers centred on the grip block, tool vertical). Close the RG2: bash scripts/lab/rg2.sh grip")
        if ask("     Ready to save the PICKUP pose? [Enter = save, s = skip] ", "s", "save") != "s":
            save_with_tilt(session, f"v1_pickup_V{i}_geometry", notes, notes_f)
        else:
            note_skipped(session, f"v1_pickup_V{i}_geometry", notes, notes_f)
        print(f"  2) Lift V{i} out and freedrive it into fitting V{i}: tabs on the slots, entry depth only, "
              "NO twist. Lower only to the entry position; if it catches or needs pressure, STOP: never use the arm to push it in.")
        if ask("     Ready to save the PLACE pose? [Enter = save, s = skip] ", "s", "save") != "s":
            save_with_tilt(session, f"v1_place_V{i}_geometry", notes, notes_f)
        else:
            note_skipped(session, f"v1_place_V{i}_geometry", notes, notes_f)
        print(f"  3) Lift V{i} back out, return it to its pocket, open the RG2.")
        ask(f"     Press Enter when V{i} is back in its pocket and the RG2 is open. ")

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
    notes = {"fixture_thickness_mm": 12.5, "self_test": True}
    print("\nTOOL tilt of the session poses (from each session yaml; NOT part tilt; warning > 1.0 deg, not a limit):")
    for i in range(1, 6):
        for kind in ("pickup", "place"):
            name = f"v1_{kind}_V{i}_geometry"
            t = tt.tool_tilt_from_pose_yaml(os.path.join(session, f"{name}.yaml"))
            warn = t["tool_tilt_deg"] > tt.WARN_DEG
            status = "copied from config/saved_poses (self-test)" if kind == "pickup" else "synthetic (self-test)"
            record_tilt(notes, name, {"status": status,
                                      "tool_tilt_deg": round(t["tool_tilt_deg"], 3),
                                      "lean_about_base_x_deg": round(t["lean_about_base_x_deg"], 3),
                                      "lean_about_base_y_deg": round(t["lean_about_base_y_deg"], 3),
                                      "over_warn_1deg": bool(warn)})
            print(f"  {name}: {tt.fmt(t)}{'  -> redo recommended (reported only in the self-test)' if warn else ''}")
    write_notes(os.path.join(session, "notes.yaml"), notes)
    rep = fit_and_check(session, 0.0125)
    print("\nSELF-TEST session:", session, "(delete it afterwards; never apply it)")
    return 0 if rep else 1


if __name__ == "__main__":
    sys.exit(main())
