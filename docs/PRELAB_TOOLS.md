# Pre-lab tools (3 Nov 2026): TOOL tilt, pickup setup consistency, wrench recording

All three tools are **read-only**. They send no motion commands and no gripper commands. Only
`scripts/phase11_physical/run_full_cycle_physical.py` moves the robot, and the wrench recording added to it does not
change what it sends.

## TOOL tilt: definition and check against the robot model

This is the tilt of the **tool** (the `gripper_tcp` frame), computed from the TCP pose. It is **not** a measurement of
the part's tilt.

The robot model is the ur_onrobot description. `tool0 -> onrobot_base_link` is `rpy (0, 0, -pi/2)`, a yaw only
(tonydle/UR_OnRobot_ROS2, `ur_onrobot_macro.xacro`). `onrobot_base_link -> gripper_tcp` is `xyz (0, 0, 0.218)`,
`rpy (0, 0, 0)` (tonydle/OnRobot_ROS2_Description, `rg2_macro.xacro`). The gripper_tcp z-axis is therefore the tool0
z-axis: the approach axis out of the flange through the fingers. With the tool upright it points **down**.

Offline evidence, from `scripts/lab/test_lab_tools.py`:

- FK from `scripts/scene/ur_kinematics.py` (the same chain) gives the same z-axis as every recorded `gripper_tcp`
  quaternion in `config/saved_poses`, within 0.05°.
- All of those z-axes point down (z < -0.99). `v1_pickup_V1_geometry` gives `[0.005, -0.016, -0.9999]`.

| Quantity | Formula (tool z = `[zx, zy, zz]` in base) | Range |
|---|---|---|
| TOOL tilt | `acos(-zz)` | 0° upright … 180° inverted |
| lean about base x | `atan2(zy, -zz)` | signed |
| lean about base y | `atan2(-zx, -zz)` | signed |

Sign convention (right-hand rule about the **base** axes): a positive lean about base +x swings the fingertips toward
base +y, and a positive lean about base +y swings them toward base -x. Yaw about the approach axis changes none of the
three numbers. Tested cases: upright = 0; known x and y tilts; yaw-only = 0; inverted = 180.

## Usage

```bash
# live meter while freedriving (own terminal; freedrive.sh in another)
python3 scripts/lab/tilt_meter.py                    # ~5 Hz; flags > 1.0 deg and stale TF

# calibration: tilt is printed after every successful save, recorded in the session notes.yaml (status
# saved / kept_existing / missing), max pickup tilt in calibration_report.yaml; > 1.0 deg -> "redo recommended"
bash CALIBRATE_PLATES.sh

# hand-pickup setup consistency (N = 5 by default); RG2 via rg2.sh in another terminal
python3 scripts/lab/pickup_consistency.py --trials 5

# offline tests (no ROS)
python3 scripts/lab/test_lab_tools.py
python3 scripts/phase11_physical/test_runner_fake_ros.py
```

The 1.0° tilt warning is not one of the calibration LIMITS and does not affect `all_within_limits`. The old poses
in `config/saved_poses` have TOOL tilts of 0.79–2.43° (V1 0.95, V2 2.43, V3 0.79, V4 2.06, V5 0.80).
`--test-from-saved` reports them and still exits 0.

## Pickup setup consistency: scope

The spread it reports covers the TCP pose, TOOL tilt and RG2 width at the "closed, before lifting" checkpoint. It shows
how consistently the operator placed the tool and how consistently the RG2 closed. **It does not measure
part-to-gripper repeatability or slip.** The y/n answers are the operator's observations.

## Wrench recording in the physical runner

**This is recording only.** The runner does no contact detection, no force-controlled insertion and no force-triggered
stopping. Nothing in the motion logic reads the wrench.

- Topic `--wrench-topic` (default `/force_torque_sensor_broadcaster/wrench`), or `--no-wrench`. Checked against the
  Humble sources: ros2_controllers `ForceTorqueSensorBroadcaster` publishes `geometry_msgs/WrenchStamped` on `~/wrench`
  with `SystemDefaultsQoS` (reliable, volatile). The `topic_name: ft_data` key in `ur_onrobot_controllers.yaml` is
  not a parameter of that broadcaster.
- The subscriber is best-effort (`sensor_data`), which is compatible with the reliable publisher. At start the runner
  records the publishers it sees, with their type and QoS, in `wrench_summary.json`.
- The UR driver rotates the values into the pendant TCP frame (`tool0_controller`); `frame_id` is the broadcaster's
  label (`tool0` in ur_onrobot). The values are raw and not zeroed.
- For every commanded stage it records samples, frame ids, max |F|, max |T|, the force at the start of the stage, the
  max change from that start, the max gap, the max header age and `stale` (gap or age > 0.1 s).
- Outputs go to `wrench_log.csv`, `wrench_summary.json` and `status.json["wrench_recording"]` in the run's log folder.
  Missing data are written as empty or null, never as 0. If the topic is absent the runner warns and continues.
