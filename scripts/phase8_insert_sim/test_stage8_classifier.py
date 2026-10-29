#!/usr/bin/env python3
"""Known-pose checks for the Stage 8 V1<->fitting contact classifier (no ROS).

  python3 scripts/phase8_insert_sim/test_stage8_classifier.py
Each case places V1 directly in the fitting frame (CAD geometry only) and states the
expected classification. Exit code 1 if any case is wrong.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stage8_core as s8  # noqa: E402
import scene_model as sm  # noqa: E402

np = sm.np
CASES = [  # name, dx, dy, z(bottom, mm), yaw deg, tilt deg, stage, expected status
    ("seated exactly on floor", 0, 0, 2.00, 0, 0, "INSERT", "intended_contact"),
    ("0.03 mm into floor (within 0.05 tolerance)", 0, 0, 1.97, 0, 0, "INSERT", "intended_contact"),
    ("0.10 mm into floor", 0, 0, 1.90, 0, 0, "INSERT", "EXCESSIVE_PENETRATION"),
    ("mid-depth, centred", 0, 0, 4.00, 0, 0, "INSERT", "clear"),
    ("mid-depth, 0.24 mm off-axis (< 0.25 clearance)", 0.24, 0, 4.00, 0, 0, "INSERT", "clear"),
    ("mid-depth, 0.40 mm off-axis", 0.40, 0, 4.00, 0, 0, "INSERT", "INTERFERENCE"),
    ("seated, rotated 25 deg CW", 0, 0, 2.00, -25, 0, "ROTATE", "intended_contact"),
    ("seated, rotated 26 deg CW", 0, 0, 2.00, -26, 0, "ROTATE", "intended_contact"),
    ("seated, rotated 27 deg CW (past stop)", 0, 0, 2.00, -27, 0, "ROTATE", "INTERFERENCE"),
    ("seated, rotated 3 deg CCW (wrong way)", 0, 0, 2.00, 3, 0, "ROTATE", "INTERFERENCE"),
    ("Stage 7 arrival, lowered to 6.9 mm", 1.07, 2.37, 6.90, 0, 3.8, "INSERT", "INTERFERENCE"),
]


def main():
    objs = sm.scene_objects(); F = objs["fitting_V1"]["T"]; g = s8.Geometry(objs, F)
    bad = 0
    for name, dx, dy, z, yaw, tilt, stage, expect in CASES:
        L = np.eye(4); L[:3, :3] = sm.rpy_matrix(math.radians(tilt), 0, math.radians(yaw)); L[:3, 3] = [dx / 1e3, dy / 1e3, z / 1e3]
        e = g.evaluate(F @ L, stage, yaw)
        ok = e["status"] == expect; bad += not ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name:48s} expected {expect:22s} got {e['status']:22s} "
              f"{e['contact']:24s} pen {e['pen_mm']:.3f} mm")
    print("ALL PASS" if not bad else f"{bad} FAIL")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
