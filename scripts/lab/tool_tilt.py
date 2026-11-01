#!/usr/bin/env python3
"""TOOL tilt of the RG2 from a base -> gripper_tcp orientation (pure numpy, no ROS).

This is the tilt of the TOOL (the gripper_tcp frame), computed from the robot's TCP pose.
It is NOT a measurement of the part's tilt: a part can sit crooked in an upright gripper, or
straight in a tilted one. Nothing here sees the part.

Axis definition (checked against the robot model, see docs/PRELAB_TOOLS.md):
  ur_onrobot:  tool0 -> onrobot_base_link   rpy (0, 0, -pi/2)   (yaw only)
  OnRobot RG2: onrobot_base_link -> gripper_tcp   xyz (0, 0, 0.218), rpy (0, 0, 0)
so the gripper_tcp z-axis is the tool0 z-axis, i.e. the approach axis out of the flange through
the fingers. With the tool upright it points DOWN: saved poses give z = [0.005, -0.016, -0.9999]
in base.

  tool tilt           = angle between the tool z-axis and base -z, in [0, 180] deg
                        (0 = upright; 180 = tool pointing straight up, i.e. inverted)
  lean about base x   = atan2( z_y, -z_z)   [deg]
  lean about base y   = atan2(-z_x, -z_z)   [deg]

Sign convention (right-hand rule about the BASE axes): a positive lean about base +x is the
rotation about base +x that swings the approach axis (fingertips) toward base +y; a positive lean
about base +y swings the fingertips toward base -x. For a tilt about a single base axis the
lean equals the signed tilt exactly; for a combined tilt each lean is the angle of the approach
axis projected onto the base y-z (or x-z) plane. Yaw about the approach axis does not change
any of the three numbers.
"""
import math

import numpy as np

WARN_DEG = 1.0  # calibration warning threshold (redo recommended); a warning, not a limit

SIGN_CONVENTION = ("lean about base +x > 0: fingertips swing toward base +y; "
                   "lean about base +y > 0: fingertips swing toward base -x (right-hand rule)")


def quat_xyzw_to_matrix(q):
    """Rotation matrix from a quaternion [x, y, z, w] (normalised here)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError(f"invalid quaternion {q}")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tool_tilt_from_matrix(R):
    """TOOL tilt dict from the 3x3 rotation of gripper_tcp in base."""
    z = np.asarray(R, float)[:3, 2]
    z = z / np.linalg.norm(z)
    c = float(np.clip(-z[2], -1.0, 1.0))           # cos(angle between tool z and base -z)
    return {"tool_tilt_deg": math.degrees(math.acos(c)),
            "lean_about_base_x_deg": math.degrees(math.atan2(z[1], -z[2])),
            "lean_about_base_y_deg": math.degrees(math.atan2(-z[0], -z[2])),
            "tool_z_in_base": [float(v) for v in z]}


def tool_tilt_from_quat(q_xyzw):
    return tool_tilt_from_matrix(quat_xyzw_to_matrix(q_xyzw))


def tool_tilt_from_pose_yaml(path):
    """TOOL tilt from a save_named_pose yaml (tcp.quaternion_xyzw). Raises if the field is missing."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)
    q = ((doc or {}).get("tcp") or {}).get("quaternion_xyzw")
    if q is None or len(q) != 4:
        raise ValueError(f"{path}: no tcp.quaternion_xyzw")
    return tool_tilt_from_quat(q)


def fmt(t):
    return (f"TOOL tilt {t['tool_tilt_deg']:6.2f} deg  (lean about base x {t['lean_about_base_x_deg']:+6.2f}, "
            f"about base y {t['lean_about_base_y_deg']:+6.2f} deg)")
