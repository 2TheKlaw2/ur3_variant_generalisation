#!/usr/bin/env python3
"""Load the V1-V5 scene into MoveIt's planning scene (fake hardware, ROS_DOMAIN_ID=73).

  python3 scripts/phase6_pickup_sim/planning_scene_tools.py --load     # add/replace all 12 objects
  python3 scripts/phase6_pickup_sim/planning_scene_tools.py --reset    # detach any part, restore all to their pockets
  python3 scripts/phase6_pickup_sim/planning_scene_tools.py --clear    # remove everything this tool added

Objects come from scene_model.scene_objects() (same poses as the RViz URDF scene).
Also provides attach/ACM helpers used by run_v1_pickup_lift_sim.py.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scene"))
import scene_model as sm  # noqa: E402

OBJECT_IDS = (["pickup_plate", "placement_plate"] + [f"{k}_V{v}" for v in range(1, 6) for k in ("pickup", "fitting")]
              + ["fixture_plate", "trolley_top"])


def require_sim_domain():
    if os.environ.get("ROS_DOMAIN_ID") != "73":
        raise SystemExit("BLOCKED: fake-hardware tools require ROS_DOMAIN_ID=73")


def _pose_msg(T):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in sm.matrix_to_quat(T[:3, :3]))
    return p


def _identity_pose():
    from geometry_msgs.msg import Pose
    p = Pose(); p.orientation.w = 1.0; return p


_TF = {}


def lookup(node, target, source, timeout=6.0):
    """4x4 transform of `source` expressed in `target` from TF (latest)."""
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    if node not in _TF:
        buf = Buffer(); _TF[node] = (buf, TransformListener(buf, node))
    buf = _TF[node][0]
    end = time.monotonic() + timeout
    while not buf.can_transform(target, source, Time()):
        if time.monotonic() > end:
            raise RuntimeError(f"TF {target} <- {source} unavailable")
        rclpy.spin_once(node, timeout_sec=0.1)
    tr = buf.lookup_transform(target, source, Time()).transform
    return sm.pose_to_matrix((tr.translation.x, tr.translation.y, tr.translation.z),
                             (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))


_MESH_CACHE = {}


def mesh_msg(path):
    if path not in _MESH_CACHE:
        from geometry_msgs.msg import Point
        from shape_msgs.msg import Mesh, MeshTriangle
        v, f = sm.read_stl(path)
        m = Mesh()
        m.vertices = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in v]
        m.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in f]
        _MESH_CACHE[path] = m
    return _MESH_CACHE[path]


def collision_object(obj_id, T, mesh_path, frame="base", operation=None):
    from moveit_msgs.msg import CollisionObject
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = obj_id
    co.pose = _pose_msg(T)
    if mesh_path:
        co.meshes = [mesh_msg(mesh_path)]
        co.mesh_poses = [_identity_pose()]
    co.operation = CollisionObject.ADD if operation is None else operation
    return co


class SceneClient:
    """Thin wrapper around /apply_planning_scene and /get_planning_scene."""

    def __init__(self, node):
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        self.node = node
        self.apply_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_cli = node.create_client(GetPlanningScene, "/get_planning_scene")

    def _call(self, cli, req, label, timeout=30.0):
        import rclpy
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{label}: service unavailable (is move_group running in domain 73?)")
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"{label}: no response")
        return fut.result()

    def apply(self, scene, label):
        from moveit_msgs.srv import ApplyPlanningScene
        scene.is_diff = True
        scene.robot_state.is_diff = True
        r = self._call(self.apply_cli, ApplyPlanningScene.Request(scene=scene), label)
        if not r.success:
            raise RuntimeError(f"{label}: MoveIt rejected the planning-scene diff")

    def get(self, components):
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        req = GetPlanningScene.Request(); req.components = PlanningSceneComponents(components=components)
        return self._call(self.get_cli, req, "get_planning_scene").scene

    # ---------- world objects
    def load_all(self):
        from moveit_msgs.msg import PlanningScene
        objs = sm.scene_objects()
        self.detach_v1_if_attached()
        ps = PlanningScene()
        ps.world.collision_objects = [collision_object(k, objs[k]["T"], objs[k]["mesh"]) for k in OBJECT_IDS]
        self.apply(ps, "load scene")
        return objs

    def clear_all(self):
        from moveit_msgs.msg import CollisionObject, PlanningScene
        self.detach_v1_if_attached(remove=True)
        ps = PlanningScene()
        ps.world.collision_objects = [collision_object(k, sm.np.eye(4), None, operation=CollisionObject.REMOVE) for k in OBJECT_IDS]
        self.apply(ps, "clear scene")

    def world_poses(self):
        """World object poses expressed in `base` (MoveIt reports them in its planning frame)."""
        from moveit_msgs.msg import PlanningSceneComponents
        sc = self.get(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        out, frame = {}, ""
        for co in sc.world.collision_objects:
            p = co.pose; frame = co.header.frame_id
            out[co.id] = sm.pose_to_matrix((p.position.x, p.position.y, p.position.z),
                                           (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        if frame and frame != "base":
            T_fb = lookup(self.node, frame, "base")
            Tinv = sm.np.linalg.inv(T_fb)
            out = {k: Tinv @ v for k, v in out.items()}
        return out, frame

    # ---------- attach / detach
    def attached(self):
        from moveit_msgs.msg import PlanningSceneComponents
        sc = self.get(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        return {a.object.id: a for a in sc.robot_state.attached_collision_objects}

    def attach_part(self, obj_id, T_grasp, mesh_path, link="gripper_tcp"):
        """Attach a moving part (pickup_V<i>) to `link` at the fixed grasp transform.

        MoveIt removes the same-named world object itself when it is attached, so the
        diff contains only the attached object (an extra world REMOVE is rejected).
        """
        from moveit_msgs.msg import AttachedCollisionObject, PlanningScene
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object = collision_object(obj_id, T_grasp, mesh_path, frame=link)
        aco.touch_links = list(sm.RG2_TOUCH_LINKS)
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [aco]
        self.apply(ps, f"attach {obj_id}")
        if obj_id in self.world_poses()[0]:
            raise RuntimeError(f"attach {obj_id}: world copy still present")

    def attach_v1(self, T_grasp, mesh_path, link="gripper_tcp"):
        self.attach_part("pickup_V1", T_grasp, mesh_path, link)

    def detach_v1_if_attached(self, remove=False):
        """Detach every attached moving part (pickup_V1..V5); name kept for the V1 scripts."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
        ids = [k for k in self.attached() if k.startswith("pickup_V")]
        for obj_id in ids:
            aco = AttachedCollisionObject(); aco.object.id = obj_id; aco.object.operation = CollisionObject.REMOVE
            aco.link_name = "gripper_tcp"
            ps = PlanningScene(); ps.robot_state.attached_collision_objects = [aco]
            self.apply(ps, f"detach {obj_id}")
            if remove:  # MoveIt puts a detached object back into the world; remove it too
                ps = PlanningScene()
                ps.world.collision_objects = [collision_object(obj_id, sm.np.eye(4), None, operation=CollisionObject.REMOVE)]
                self.apply(ps, f"remove detached {obj_id}")
        return bool(ids)

    # ---------- allowed collision matrix
    def set_allowed(self, a, b, allowed):
        """Allow/disallow one object pair. Reads the full ACM, edits it, writes it back."""
        from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene, PlanningSceneComponents
        acm = self.get(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
        names = list(acm.entry_names)
        for n in (a, b):
            if n not in names:
                for row in acm.entry_values:
                    row.enabled.append(False)
                names.append(n)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
        acm.entry_names = names
        for row in acm.entry_values:
            while len(row.enabled) < len(names):
                row.enabled.append(False)
        i, j = names.index(a), names.index(b)
        acm.entry_values[i].enabled[j] = bool(allowed)
        acm.entry_values[j].enabled[i] = bool(allowed)
        ps = PlanningScene(); ps.allowed_collision_matrix = acm
        self.apply(ps, f"ACM {a}<->{b}={'allow' if allowed else 'check'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--load", action="store_true"); g.add_argument("--reset", action="store_true"); g.add_argument("--clear", action="store_true")
    ap.add_argument("--physical", action="store_true",
                    help="allow ROS_DOMAIN_ID=0 (real robot's MoveIt): loads/clears collision objects ONLY, never moves "
                         "anything; used by the physical full-cycle runner's state-validity checks")
    a = ap.parse_args()
    if a.physical:
        if os.environ.get("ROS_DOMAIN_ID", "0") != "0":
            raise SystemExit("--physical expects the real robot's ROS_DOMAIN_ID=0")
        print("PHYSICAL MoveIt: collision objects only, no robot motion", flush=True)
    else:
        require_sim_domain()
    import rclpy
    rclpy.init()
    node = rclpy.create_node("v1_scene_tools_SIM")
    try:
        sc = SceneClient(node)
        if a.clear:
            sc.clear_all(); print("Removed all scene objects.")
        else:
            t0 = time.monotonic(); sc.load_all()
            if a.reset:
                for i in range(1, 6):
                    sc.set_allowed(f"pickup_V{i}", "pickup_plate", False)
                    sc.set_allowed(f"pickup_V{i}", f"fitting_V{i}", False)
            poses, frame = sc.world_poses()
            missing = [k for k in OBJECT_IDS if k not in poses]
            if missing:
                raise RuntimeError(f"Objects missing after load: {missing}")
            objs = sm.scene_objects()
            err = max(sm.np.abs(poses[k] - objs[k]["T"]).max() for k in OBJECT_IDS)
            print(f"Loaded {len(OBJECT_IDS)} objects in '{frame}' ({time.monotonic()-t0:.1f}s); "
                  f"max pose readback error (in base) {err:.2e}")
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
