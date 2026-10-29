"""Lift geometry + timing shared by the offline preview and the fake-hardware run.

Geometry copied from scripts/phase5_integration/run_v1_lift_bridge_transport_staged.py
(physically tested): +30 mm along base Z, orientation held constant, 15 Cartesian
waypoints (MoveIt feasibility check), wrist_3 2*pi correction. Simulation samples the
straight line at every 20 ms timestamp of the min-jerk profile (one IK solution per sample). Timing is NEW for simulation: 3.0 s
minimum-jerk on path length (the physical lift used a MoveIt-timed >= 5 s profile).
"""
import math

import numpy as np

LIFT_M = 0.030
N_WAYPOINTS = 15                  # as physical script
SIM_LIFT_DURATION_S = 3.0         # NEW simulation timing (physical: >= 5 s)
PHYSICAL_LIFT_MIN_DURATION_S = 5.0
CARTESIAN_MAX_STEP_M = 0.0005     # offline-preview spacing only (physical MoveIt max_step was 0.002)
SAMPLE_DT_S = 0.02                # timed IK samples: one IK solution per 20 ms timestamp
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def maxdiff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def lift_waypoints(origin_xyz, quat_xyzw):
    """15 poses (xyz, quat) straight up from the grip TCP pose, orientation constant."""
    return [((origin_xyz[0], origin_xyz[1], origin_xyz[2] + LIFT_M * k / N_WAYPOINTS), tuple(quat_xyzw))
            for k in range(1, N_WAYPOINTS + 1)]


def correct_wraps(raw, reference, label="LIFT"):
    """Physical-script wrist_3 2*pi correction; reject any other branch change."""
    offsets = []
    for i, j in enumerate(JOINTS):
        n = round((raw[0][i] - reference[i]) / (2 * math.pi))
        if abs(raw[0][i] - n * 2 * math.pi - reference[i]) > 0.012:
            raise RuntimeError(f"{label}: IK start mismatch on {j}")
        if n != 0 and j != "wrist_3_joint":
            raise RuntimeError(f"{label}: 2pi branch for unexpected joint {j}")
        offsets.append(n)
    out = [[v - n * 2 * math.pi for v, n in zip(row, offsets)] for row in raw]
    for k, (a, b) in enumerate(zip(out, out[1:])):
        if maxdiff(a, b) > 0.05:
            j = max(range(6), key=lambda i: abs(a[i] - b[i]))
            raise RuntimeError(f"{label}: excessively large joint step {abs(a[j]-b[j]):.4f} rad on {JOINTS[j]} "
                               f"between points {k} and {k+1} of {len(out)}")
    return out, offsets


def min_jerk_s(tau):
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def min_jerk_times(path_fraction, duration=SIM_LIFT_DURATION_S):
    """Map monotonic path fractions in [0,1] to times of a minimum-jerk profile."""
    f = np.clip(np.asarray(path_fraction, float), 0.0, 1.0)
    if np.any(np.diff(f) <= 0):
        raise RuntimeError("Lift path fraction is not strictly increasing")
    taus = np.linspace(0, 1, 20001)
    s = min_jerk_s(taus)
    t = np.interp(f, s, taus) * duration
    t[0], t[-1] = 0.0, duration
    return t


def min_jerk_grid(duration=SIM_LIFT_DURATION_S, dt=0.02):
    """Timestamps every dt and the min-jerk path fraction at each (0 -> 1, zero vel/acc at ends)."""
    t = np.arange(0.0, duration + 1e-9, dt)
    if t[-1] < duration - 1e-9:
        t = np.append(t, duration)
    return t, min_jerk_s(t / duration)


def resample_uniform(times, q, dt=0.02):
    """Resample a joint path to a fixed controller-friendly time grid (keeps endpoints)."""
    times = np.asarray(times); q = np.asarray(q)
    grid = np.arange(0.0, times[-1] + 1e-9, dt)
    if grid[-1] < times[-1] - 1e-9:
        grid = np.append(grid, times[-1])
    return grid, np.stack([np.interp(grid, times, q[:, k]) for k in range(q.shape[1])], 1)


def verify_path(name, q, t, speed_limit=0.14, step_limit=0.045):
    """Same guard as the physical script (rad, rad/s)."""
    if len(q) != len(t) or len(q) < 3 or abs(t[0]) > 1e-6:
        raise RuntimeError(f"{name}: invalid trajectory size/timing")
    if any(b <= a for a, b in zip(t, t[1:])):
        raise RuntimeError(f"{name}: nonmonotonic time")
    jumps = [maxdiff(a, b) for a, b in zip(q, q[1:])]
    rates = [v / (tb - ta) for v, ta, tb in zip(jumps, t, t[1:])]
    if max(jumps) > step_limit or max(rates) > speed_limit:
        raise RuntimeError(f"{name}: joint step/speed guard failed; max step={max(jumps):.4f} rad; "
                           f"max speed={max(rates):.4f} rad/s")
    return max(jumps), max(rates)
