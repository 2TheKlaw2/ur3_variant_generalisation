#!/usr/bin/env python3
"""Live TOOL tilt meter for freedrive (READ-ONLY: no motion commands, no gripper commands).

  python3 scripts/lab/tilt_meter.py                 # ~5 Hz, base -> gripper_tcp, until Ctrl+C
  python3 scripts/lab/tilt_meter.py --rate 2 --warn 1.0
  python3 scripts/lab/tilt_meter.py --self-test     # offline check of the maths (no ROS)

It only listens to TF (robot_state_publisher on /tf) and prints the TOOL tilt of gripper_tcp:
the angle between the tool approach axis (gripper_tcp z, DOWN when upright) and base -z, plus the
signed leans about base x and y. This is the tilt of the TOOL, not a measured tilt of the part.
Run it in its own terminal while bash scripts/lab/freedrive.sh is running. Ctrl+C ends it.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tool_tilt as tt  # noqa: E402

STALE_S = 0.5


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-frame", default="base")
    ap.add_argument("--tool-frame", default="gripper_tcp")
    ap.add_argument("--rate", type=float, default=5.0, help="print rate [Hz] (default 5)")
    ap.add_argument("--warn", type=float, default=tt.WARN_DEG, help="mark TOOL tilt above this [deg] (default 1.0)")
    ap.add_argument("--self-test", action="store_true", help="run the offline maths tests and exit (no ROS)")
    a = ap.parse_args()
    if a.self_test:
        import test_lab_tools
        return test_lab_tools.run_all()
    if not 0.2 <= a.rate <= 20:
        raise SystemExit("--rate must be in [0.2, 20] Hz")

    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node("tool_tilt_meter_readonly")
    buf = Buffer()
    TransformListener(buf, node)
    print(f"TOOL tilt meter (READ-ONLY, sends nothing). ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
          f"TF {a.base_frame} -> {a.tool_frame}.\n"
          f"TOOL tilt = angle(tool z, base -z): 0 = upright. NOT the part's tilt.\nSign: {tt.SIGN_CONVENTION}\n"
          f"Ctrl+C to stop.\n", flush=True)
    period = 1.0 / a.rate
    nxt = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() < nxt:
                continue
            nxt += period
            try:
                tf = buf.lookup_transform(a.base_frame, a.tool_frame, Time(), timeout=Duration(seconds=0.0))
            except Exception as e:  # noqa: BLE001 - any TF error is shown, never hidden
                print(f"  no TF {a.base_frame} -> {a.tool_frame}: {type(e).__name__}", flush=True)
                continue
            r = tf.transform.rotation
            t = tt.tool_tilt_from_quat([r.x, r.y, r.z, r.w])
            age = (node.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            p = tf.transform.translation
            flag = "  TILTED > %.1f deg" % a.warn if t["tool_tilt_deg"] > a.warn else ""
            stale = f"  STALE TF ({age:.1f} s old)" if age > STALE_S else ""
            print(f"  {tt.fmt(t)}  TCP [{p.x * 1000:7.1f} {p.y * 1000:7.1f} {p.z * 1000:7.1f}] mm{flag}{stale}",
                  flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
