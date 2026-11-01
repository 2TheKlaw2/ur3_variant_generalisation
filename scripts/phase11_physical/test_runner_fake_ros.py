#!/usr/bin/env python3
"""Offline harness (NO ROS, no robot): run run_full_cycle_physical.execute() against in-process fakes of rclpy,
the UR/RG2 controllers, MoveIt's /check_state_validity and the planning-scene client, to test the wrench
RECORDING end to end and that it changes nothing in what is commanded.

  python3 scripts/phase11_physical/test_runner_fake_ros.py [--variant 1]

Cases (each a full staged V<i> cycle HOME_START..HOME with every RUN_<STAGE> typed by the fake operator):
  A  wrench topic publishing          -> per-stage max |F| / |T|, wrench_log.csv, wrench_summary.json
  B  wrench topic absent              -> warns, records UNAVAILABLE (null, never 0), run still PASS
  C  --no-wrench                      -> no subscription at all
  D  wrench callback gets a bad msg   -> callback error counted, run unaffected
  E  operator types something else    -> stops before any motion; logs still written
Then: the goals sent to the arm/gripper (joint names, positions, times) are IDENTICAL in A, B, C and D.
This does NOT test the real UR driver, real QoS matching, real timing or real hardware.
"""
import argparse
import builtins
import json
import math
import sys
import tempfile
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))


# ------------------------------------------------------------------ fakes
class Fut:
    def __init__(self, result=None, ticks=0):
        self._r, self._ticks = result, ticks

    def done(self):
        return self._ticks <= 0

    def tick(self):
        self._ticks -= 1

    def result(self):
        return self._r


class W:   # simple attribute bag
    def __init__(self, **k):
        self.__dict__.update(k)


class World:
    def __init__(self, q0, width=0.040):
        self.q, self.width, self.t_ns = list(q0), width, 1_000_000_000_000
        self.sent, self.pending, self.wrench_mode, self.nodes = [], [], "ok", []


WORLD = None


def make_fake_ros():
    m = {}

    def mod(name, **attrs):
        x = types.ModuleType(name); x.__dict__.update(attrs); m[name] = x; return x

    class Duration:
        def __init__(self, seconds=0.0):
            self.s = float(seconds)

        def to_msg(self):
            return W(sec=int(self.s), nanosec=int(round((self.s - int(self.s)) * 1e9)))

    class Client:
        def __init__(self, srv):
            self.srv = srv

        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, req):
            n = self.srv
            if n == "hw":
                return Fut(W(component=[W(name="UR3e", class_type="ur_robot_driver/URPositionHardwareInterface"),
                                        W(name="OnRobotGripper", class_type="onrobot_driver/RGHardwareInterface")]))
            if n == "ctrl":
                return Fut(W(controller=[W(name="scaled_joint_trajectory_controller", state="active"),
                                         W(name="finger_width_trajectory_controller", state="active"),
                                         W(name="joint_trajectory_controller", state="inactive")]))
            return Fut(W(valid=True, contacts=[]))

    class Node:
        def __init__(self, name):
            self._subs = []; WORLD.nodes.append(self)

        def create_subscription(self, typ, topic, cb, qos):
            self._subs.append((topic, cb))

        def create_client(self, typ, name):
            return Client("hw" if "hardware" in name else "ctrl" if "controllers" in name else "valid")

        def get_publishers_info_by_topic(self, topic):
            if "wrench" in topic and WORLD.wrench_mode != "absent":
                return [W(node_namespace="/", node_name="force_torque_sensor_broadcaster",
                          topic_type="geometry_msgs/msg/WrenchStamped",
                          qos_profile=W(reliability="RELIABLE", durability="VOLATILE"))]
            return []

        def get_clock(self):
            return W(now=lambda: W(nanoseconds=WORLD.t_ns))

        def get_logger(self):
            return W(info=print, warn=print)

        def destroy_node(self):
            pass

    def spin_once(node, timeout_sec=None):
        WORLD.t_ns += 2_000_000
        for f in list(WORLD.pending):
            f.tick()
            if f.done():
                WORLD.pending.remove(f)
                pts = f.traj.points
                if f.traj.joint_names == ["finger_width"]:
                    WORLD.width = pts[-1].positions[0]
                else:
                    WORLD.q = list(pts[-1].positions)
        js = W(name=list(JOINTS) + ["finger_width"], position=list(WORLD.q) + [WORLD.width])
        for topic, cb in node._subs:
            if topic == "/joint_states":
                cb(js)
            elif "wrench" in topic and WORLD.wrench_mode in ("ok", "bad"):
                if WORLD.wrench_mode == "bad" and WORLD.t_ns % 10_000_000 == 0:
                    cb(W(header=None))                      # malformed: the callback must swallow it
                    continue
                s = W(sec=WORLD.t_ns // 1_000_000_000, nanosec=WORLD.t_ns % 1_000_000_000)
                busy = 1.0 if WORLD.pending else 0.0
                cb(W(header=W(stamp=s, frame_id="tool0"),
                     wrench=W(force=W(x=0.5, y=-0.2, z=-4.0 - 3.0 * busy), torque=W(x=0.01, y=0.02, z=0.0))))

    class ActionClient:
        def __init__(self, node, typ, name):
            self.name = name

        def wait_for_server(self, timeout_sec=None):
            return True

        def send_goal_async(self, goal):
            tr = goal.trajectory
            WORLD.sent.append((self.name, list(tr.joint_names),
                               [(round(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, 6),
                                 [round(x, 9) for x in p.positions]) for p in tr.points]))
            res = Fut(W(status=4, result=W(error_code=0, error_string="")), ticks=5)
            res.traj = tr; WORLD.pending.append(res)
            return Fut(W(accepted=True, get_result_async=lambda: res, cancel_goal_async=lambda: Fut(None)))

    class Msg:
        def __init__(self, **k):
            self.__dict__.update(k)

    class JointTrajectory(Msg):
        def __init__(self):
            self.joint_names, self.points = [], []

    class JointTrajectoryPoint(Msg):
        def __init__(self):
            self.positions, self.time_from_start = [], None

    class Goal(Msg):
        pass

    class Req(Msg):
        pass

    class RobotState:
        def __init__(self):
            self.joint_state = W(name=[], position=[])

    mod("rclpy", init=lambda **k: None, shutdown=lambda: None, ok=lambda: True, spin_once=spin_once)
    mod("rclpy.action", ActionClient=ActionClient)
    mod("rclpy.duration", Duration=Duration)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", qos_profile_sensor_data="sensor_data")
    mod("rclpy.signals", SignalHandlerOptions=W(NO="NO"))
    mod("action_msgs"); mod("action_msgs.msg", GoalStatus=W(STATUS_SUCCEEDED=4, STATUS_CANCELED=5))
    mod("control_msgs"); mod("control_msgs.action", FollowJointTrajectory=W(Goal=Goal))
    mod("controller_manager_msgs"); mod("controller_manager_msgs.srv", ListControllers=W(Request=Req),
                                        ListHardwareComponents=W(Request=Req))
    mod("moveit_msgs"); mod("moveit_msgs.msg", RobotState=RobotState); mod("moveit_msgs.srv", GetStateValidity=W(Request=Req))
    mod("sensor_msgs"); mod("sensor_msgs.msg", JointState=Msg)
    mod("trajectory_msgs"); mod("trajectory_msgs.msg", JointTrajectory=JointTrajectory, JointTrajectoryPoint=JointTrajectoryPoint)
    mod("geometry_msgs"); mod("geometry_msgs.msg", WrenchStamped=Msg)

    class SceneClient:
        def __init__(self, node):
            self._att = {}

        def world_poses(self):
            return {"pickup_plate": 1, "placement_plate": 1, **{f"fitting_V{i}": 1 for i in range(1, 6)}}, None

        def attached(self):
            return self._att

        def attach_part(self, obj_id, G, mesh):
            self._att[obj_id] = 1

        def set_allowed(self, a, b, allowed):
            pass

        def detach_v1_if_attached(self, remove=False):
            self._att.clear()
    mod("planning_scene_tools", SceneClient=SceneClient)
    return m


JOINTS = None


def run_case(r, plans, G, objs, variant, plans_dir, tmp, mode, answer_ok=True):
    global WORLD
    WORLD = World(r.HOME_Q); WORLD.wrench_mode = mode
    fake = make_fake_ros()
    saved = {k: sys.modules.get(k) for k in fake}
    sys.modules.update(fake)
    orig_input, orig_root = builtins.input, r.ROOT
    builtins.input = (lambda p="": p.split("Type ", 1)[1].split(" ", 1)[0]) if answer_ok else (lambda p="": "no")
    r.ROOT = tmp
    args = argparse.Namespace(plans=plans_dir, max_joint_speed=0.14, from_stage="HOME_START", until="HOME",
                              execute=True, wrench_topic="" if mode == "disabled" else r.WRENCH_TOPIC)
    try:
        rc = r.execute(variant, plans, G, objs, args)
    finally:
        builtins.input, r.ROOT = orig_input, orig_root
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    logdir = sorted((tmp / "results/physical/full_cycle").glob(f"V{variant}_*"))[-1]
    return rc, logdir, list(WORLD.sent)


def main():
    global JOINTS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", type=int, default=1, choices=[1, 2, 3])
    a = ap.parse_args()
    import os
    import shutil
    import time
    import run_full_cycle_physical as r
    JOINTS = r.JOINTS
    os.environ["ROS_DOMAIN_ID"] = "0"
    plans_src = r.default_plans()
    plans, G, objs = r.preflight(a.variant, plans_src, 0.14)
    fails = []

    def check(cond, msg):
        print(("PASS " if cond else "FAIL ") + msg); cond or fails.append(msg)

    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        (tmp / "config/scene").mkdir(parents=True)
        (tmp / "config/scene/scene_registration.yaml").write_text("layout: session\n")
        pdir = tmp / "plans"; pdir.mkdir()
        s10 = json.load(open(plans_src / "stage10_summary.json")); s10["layout"] = "session"
        json.dump(s10, open(pdir / "stage10_summary.json", "w"))
        res = {}
        for mode in ("ok", "absent", "disabled", "bad"):
            print(f"\n================ case {mode} ================")
            res[mode] = run_case(r, plans, G, objs, a.variant, pdir, tmp, mode)
            time.sleep(1.1)                                    # distinct log folder names (1 s resolution)
        print("\n================ case operator-stop ================")
        rc_stop, log_stop, sent_stop = run_case(r, plans, G, objs, a.variant, pdir, tmp, "ok", answer_ok=False)

        print("\n================ checks ================")
        for mode, (rc, logdir, sent) in res.items():
            st = json.load(open(logdir / "status.json"))
            check(rc == 0 and st["status"] == "PASS", f"[{mode}] full cycle PASS ({len(st['stages'])} stages logged)")
            check((logdir / "joint_log.csv").exists(), f"[{mode}] joint_log.csv written")
            wr = st.get("wrench_recording", {})
            if mode == "ok":
                stg = wr["stages"]
                check(all(s["available"] for s in stg) and len(stg) == len(sent), f"[ok] wrench available in all {len(stg)} commanded stages")
                check(all(abs(s["max_abs_force_N"] - math.sqrt(0.25 + 0.04 + 49)) < 1e-3 for s in stg), "[ok] max |F| = 7.02 N per stage")
                check(all(abs(s["max_abs_torque_Nm"] - math.sqrt(0.0005)) < 1e-4 for s in stg), "[ok] max |T| = 0.0224 Nm per stage")
                check(all(s["frame_ids"] == ["tool0"] for s in stg), "[ok] frame_id recorded")
                rows = (logdir / "wrench_log.csv").read_text().splitlines()
                check(len(rows) > 10 and rows[0].startswith("time,recv_monotonic_s,stage"), f"[ok] wrench_log.csv has {len(rows) - 1} samples")
                check(json.load(open(logdir / "wrench_summary.json"))["publishers_at_start"][0]["type"].endswith("WrenchStamped"),
                      "[ok] wrench_summary.json records publisher type and QoS")
            if mode == "absent":
                stg = wr["stages"]
                check(all((not s["available"]) and s["max_abs_force_N"] is None and s["max_abs_torque_Nm"] is None for s in stg),
                      "[absent] every stage UNAVAILABLE with null maxima (never 0)")
                check(wr.get("publishers_at_start") == [], "[absent] no publisher recorded")
            if mode == "disabled":
                check(wr.get("enabled") is False and wr.get("messages_received_total") == 0, "[disabled] no subscription, no data")
            if mode == "bad":
                check(wr.get("callback_errors", 0) > 0, f"[bad] {wr.get('callback_errors')} malformed messages counted, run unaffected")
        ref = res["disabled"][2]
        for mode in ("ok", "absent", "bad"):
            check(res[mode][2] == ref, f"[{mode}] commanded goals identical to --no-wrench ({len(ref)} goals)")
        st = json.load(open(log_stop / "status.json"))
        check(rc_stop == 1 and st["status"] == "STOPPED" and not sent_stop, "[operator-stop] nothing sent, STOPPED, logs written")
        shutil.rmtree(tmp / "results", ignore_errors=True)
    print(f"\n{'ALL PASS' if not fails else f'{len(fails)} FAIL'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
