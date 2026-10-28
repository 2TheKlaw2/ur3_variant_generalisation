# Implementation status — 26 October 2026

## Included now

- A ROS 2 Humble `ament_cmake` package with `package.xml` and `CMakeLists.txt`.
- C++17 geometry-CSV reader with validation. Values missing from fixture centre/yaw stay **unknown**.
- Synthetic V1 insertion 5 mm and rotation -10 degrees; trained by the **existing** `ur3_lfd::DiscreteDMP1D` (not a hand-drawn interpolation at rollout).
- One learned insertion DMP and one learned yaw DMP; automatic goal retargeting for all five variants.
- V5 coupled 5 mm descent based on the learned rotational progress.
- Two experimental time policies: `fixed` and `speed` (nominal tip path length).
- Unit tests and fixture-relative CSV outputs. **Never send these analysis files to MoveIt**.

## Not yet included

- Training from a real 6-DOF freedrive demonstration; actual reference yaw extraction and unwrap.
- Robot-base/fixture registration, tool TCP-to-assembly-axis calibration, and height-safe approach.
- Whole-robot MoveIt planning / collision checks; full quaternion orientation adaptation in base frame.
- Live gripper control, robot execution, physical success metrics.
- Automatic arbitrary STEP-file feature recognition (we currently read a prescribed CAD-parameters CSV).
- CDMP-MTM skill taxonomic integration. This is a two-phase task-specific DMP prototype.

## Why it is intentionally analysis-only

The five fixture centre coordinates, fixture-to-robot transform and grasp-to-rotation-axis transform are not yet calibrated. CSV header is intentionally different from the CSV accepted by `ur3_lfd`'s MoveIt executor, to prevent unsafe accidental execution.

Conventions to verify before base-frame conversions: fixture +Z upward; insertion is -Z; yaw is about fixture +Z; viewed from +Z looking down, clockwise is negative. Additional rotation is **not** a 45-degree tool tilt on V5.
