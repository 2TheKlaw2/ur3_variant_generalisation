# Stage 2: Independent pickup and placement plate frames (C++ / offline)

## Confirmed CAD-relative facts

- Pickup V1–V5 centres: 0, 40, 80, 120, 160 mm along **pickup plate-local +X**.
- Placement V1–V5 centres: 0, 50, 100, 150, 200 mm along **placement plate-local +X**.
- The V1 and V2 component-top references have identical elevation on each plate; V3/V4/V5 are +5 mm. There is no assertion that the two plates' absolute V1 elevations coincide.
- All components have the same initial **relative** orientation within their own plate frame. `yaw_deg=0` is a modelling convention; it is not a calibrated robot orientation.
- Motion geometry remains in `config/variant_motion_parameters.csv` and is unchanged.
- Body radius is **approximately 15 mm** according to the latest CAD clarification. The **28.8 mm** dimension measured from centre of rotation to the *rectangular tab centre* is a different radius. Verify tab path and adjacent-part clearance in CAD. The old synthetic timing model's `kTabRadiusM = 0.0288` models tab path length; it must not be used as a body collision radius.

## Coordinate frames and what is *not* calibrated

`pickup_plate_local` origin = V1 pickup component reference at its reference height.
`placement_plate_local` origin = V1 fitting centre at its top reference height.
`+X` follows each row by **convention** (not necessarily UR3e +X). `+Z` is an assumed upright reference. Each plate may have a different actual pose and orientation in the robot's `base` frame.

The source V1 DMP analysis uses a local yaw around the fitted piece's nominal +Z. The plate-local analysis output adds the variant's measured +X offset, but remains an offline fixture-local preview.

`PlateRegistration` supports a calibrated full rigid transform (origin + quaternion), but `transformToBase()` throws unless `verified=true`. Do not mark a plate verified or supply fabricated base poses. No robot-base trajectory is written by this update.

`ANALYSIS_ONLY_plate_references.csv` gives *top/centre reference points*, not actual RG2 gripping positions. Pickup requires a calibrated block contact point, orientation, gripper opening and TCP-to-part-axis transform, which are all outside this update.

`ANALYSIS_ONLY_V*_placement_local_*.csv` gives the five transformed local assembly trajectories. Its deliberately incompatible header must never be converted to MoveIt format without proper calibration and clearance validation.

## How to run

```bash
cd ~/ros2_ws/src/ur3_variant_generalisation
bash RUN_PLATE_STAGE2.sh

# Native ROS2 build should also work:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ur3_variant_generalisation
source install/setup.bash
colcon test --packages-select ur3_variant_generalisation
colcon test-result --verbose
```

The stage 2 offline test suite verifies five offsets, five elevations, phases, orientation, time monotonicity inherited from Stage 1, protection against double-height offsets, yaw rotation of local coordinates, rejection of unverified plate-to-base transforms, and a **mock** rigid transform. Passing does not validate physical safety or measured assembly performance.

## Lab checklist before robot execution

1. Verify true CAD centre-to-centre spacing and **full swept outlines**, including the tabs, at the 40/50 mm centre spacings.
2. Verify that the ~15 mm circular-body radius and the 28.8 mm tab-centre radius describe the same CAD reference geometry correctly.
3. Establish both `base -> pickup_plate_local` and `base -> placement_plate_local` transforms from measured physical fiducials; confirm axis directions, surface tilt and Z origins.
4. Measure RG2 TCP -> gripped part centre/rotation-axis transform and grip contact height.
5. Validate both tabs' groove clearance, the 10° engagement rotation, and V5's 10°/5 mm helical descent.
6. Only after registration and full collision validation, integrate six-DOF trajectories with the existing MoveIt planning and safety diagnostics. Current files are **not robot executable**.

### Plate registration template

`config/plate_base_registration.csv` includes separate `pickup` and `placement` entries with **blank measurements** and `verified=false`. It is never safe to set the flag to `true` without physically registering that plate to the correct UR3e base frame and validating the result. Orientation is configured as roll/pitch/yaw in degrees and converted internally to a quaternion using ZYX convention.
