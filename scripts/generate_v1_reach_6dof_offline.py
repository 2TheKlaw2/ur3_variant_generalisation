#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import numpy as np
import rclpy

ROOT = Path.home() / "ros2_ws/src/ur3_variant_generalisation"
OLD = Path.home() / "ros2_ws/src/ur3_lfd"

SOURCE = ROOT / "demonstrations/processed/v1_reach_001_motion.csv"
OUTPUT = ROOT / "demonstrations/processed/v1_reach_cdmp_6dof_001.csv"
MOVEIT = ROOT / "demonstrations/processed/v1_reach_cdmp_moveit_001.csv"

sys.path.insert(0, str(OLD / "scripts"))
from cdmp_servo_replay import CDMPServoReplay

if not SOURCE.exists():
    raise SystemExit(f"Missing demonstration: {SOURCE}")

if OUTPUT.exists() or MOVEIT.exists():
    raise SystemExit("Output already exists. Refusing to overwrite.")

rclpy.init()
node = None

try:
    node = CDMPServoReplay()

    node.input_demo = str(SOURCE)
    node.output_path = str(OUTPUT)

    # Reproduce the original path.
    node.spatial_scale = 1.0
    node.goal_offset = np.zeros(3)

    # Slower than the demonstration.
    node.tau_scale = 2.0

    dem_time, dem_pos = node.load_demo()

    # Offline generation uses recorded orientation,
    # never the live robot orientation.
    node.get_tcp_quaternion = lambda: node.dem_quat[0].copy()

    rep_time, rep_pos, rep_vel = node.build_cdmp(
        dem_time,
        dem_pos,
        dem_pos[0].copy()
    )

    node.save_cdmp_path(rep_time, rep_pos, rep_vel)

    # Convert to the CSV expected by our C++ MoveIt planner.
    with OUTPUT.open() as f:
        rows = list(csv.DictReader(f))

    columns = ["time", "x", "y", "z", "qx", "qy", "qz", "qw"]

    with MOVEIT.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()

        for r in rows:
            writer.writerow({
                "time": r["time"],
                "x": r["tcp_x"],
                "y": r["tcp_y"],
                "z": r["tcp_z"],
                "qx": r["tcp_qx"],
                "qy": r["tcp_qy"],
                "qz": r["tcp_qz"],
                "qw": r["tcp_qw"],
            })

    print("\n=== V1 REACH CDMP COMPLETE ===")
    print("Demonstration samples:", len(dem_time))
    print("Generated samples:", len(rep_time))
    print("Original endpoint:", dem_pos[-1])
    print("Learned endpoint:", rep_pos[-1])
    print("Endpoint error (mm):",
          np.linalg.norm(rep_pos[-1] - dem_pos[-1]) * 1000)
    print("CDMP duration:", float(rep_time[-1]), "seconds")
    print("Saved CDMP:", OUTPUT)
    print("Saved MoveIt path:", MOVEIT)
    print("\nNO ROBOT MOVEMENT COMMANDED")

finally:
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
