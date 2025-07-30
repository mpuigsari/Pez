#!/usr/bin/env python3
"""
CommandPlayer

* Publishes /<ns>/cmd_vel continuously at `rate_hz`.
* Loads a YAML script that may reference arbitrary service types
  (e.g. std_srvs/srv/Trigger, pez_interfaces/srv/SnapSensors).
* Calls each service once at the scheduled time and logs the reply.

YAML example
------------
cmd_vel:
  - {t: 0.0,  linear: {x:  0.3}, angular: {z: 0.0}}
  - {t: 5.0,  linear: {x:  0.0}, angular: {z: 0.4}}
  - {t: 8.0,  linear: {x:  0.0}, angular: {z: 0.0}}

services:
  - {t: 3.0,  service: teleoperation/swim_start, type: std_srvs/srv/Trigger}
  - {t:12.0,  service: get_sensor_snapshot,      type: pez_interfaces/srv/SnapSensors}
"""
import sys
import argparse
import importlib
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger   # still useful for default


def resolve_service_type(type_str: str):
    """
    Convert "std_srvs/srv/Trigger" to the Python service class.
    Caches results so we only import each module once.
    """
    if type_str in resolve_service_type._cache:
        return resolve_service_type._cache[type_str]

    try:
        pkg, _, tail = type_str.partition('/')
        _, _, srv_name = tail.partition('/')      # tail == "srv/Trigger"
        module = importlib.import_module(f"{pkg}.srv")
        srv_cls = getattr(module, srv_name)
    except (ModuleNotFoundError, AttributeError, ValueError) as e:
        raise RuntimeError(f"Cannot resolve service type '{type_str}': {e}") from e

    resolve_service_type._cache[type_str] = srv_cls
    return srv_cls


resolve_service_type._cache = {}


class CommandPlayer(Node):
    def __init__(self, yaml_file: str, rate_hz: float = 20.0):
        super().__init__("command_player")

        # ─── Load YAML ────────────────────────────────────────────────────
        with open(yaml_file, "r") as f:
            cfg = yaml.safe_load(f) or {}

        raw_cmd = cfg.get("cmd_vel", [])
        raw_srv = cfg.get("services", [])

        self.cmd_seq = [d for d in raw_cmd if isinstance(d, dict) and "t" in d]
        self.srv_seq = [d for d in raw_srv if isinstance(d, dict) and "t" in d]

        self.cmd_seq.sort(key=lambda x: x["t"])
        self.srv_seq.sort(key=lambda x: x["t"])

        if not self.cmd_seq:
            self.get_logger().warn("No cmd_vel entries found; publishing zeros.")

        # ─── Publisher & timer ────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.active_twist = Twist()
        self._update_active_twist(initial=True)

        self.t0 = self.get_clock().now()
        self.timer_period = 1.0 / max(rate_hz, 1.0)
        self.create_timer(self.timer_period, self.tick)

        self.get_logger().info(f"CommandPlayer running at {rate_hz:.1f} Hz")

    # ────────────────── internal helpers ────────────────────────────────
    def _update_active_twist(self, initial=False):
        while self.cmd_seq and (initial and self.cmd_seq[0]["t"] == 0.0):
            self._set_twist_from_dict(self.cmd_seq.pop(0))

    def _set_twist_from_dict(self, d):
        self.active_twist.linear.x  = d["linear"]["x"]
        self.active_twist.linear.y  = d["linear"].get("y", 0.0)
        self.active_twist.linear.z  = d["linear"].get("z", 0.0)
        self.active_twist.angular.z = d["angular"].get("z", 0.0)

    # ───────────────────────── main loop ────────────────────────────────
    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9  # seconds

        # switch to next cmd_vel if its time has come
        while self.cmd_seq and t >= self.cmd_seq[0]["t"]:
            self._set_twist_from_dict(self.cmd_seq.pop(0))
            self.get_logger().info(f"Loaded new cmd_vel at {t:.2f}s")

        # publish current cmd_vel continuously
        self.cmd_pub.publish(self.active_twist)

        # service calls (one-shot each)
        while self.srv_seq and t >= self.srv_seq[0]["t"]:
            cfg = self.srv_seq.pop(0)
            svc_name = cfg["service"]
            type_str = cfg.get("type", "std_srvs/srv/Trigger")
            srv_cls  = resolve_service_type(type_str)

            cli = self.create_client(srv_cls, svc_name)
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"{svc_name} unavailable")
                continue

            future = cli.call_async(srv_cls.Request())
            future.add_done_callback(
                lambda fut, name=svc_name, ts=t: self._log_service_response(fut, name, ts)
            )
            self.get_logger().info(f"Called {svc_name} ({type_str}) at {t:.2f}s")

        # shutdown when nothing left
        if not self.cmd_seq and not self.srv_seq:
            self.get_logger().info("Sequence complete – shutting down")
            rclpy.shutdown()

    # ────────────────── response logging ────────────────────────────────
    def _log_service_response(self, future, svc_name: str, call_time: float):
        try:
            resp = future.result()
            if hasattr(resp, "success") and hasattr(resp, "message"):
                self.get_logger().info(
                    f"{svc_name} response (sent @{call_time:.2f}s) → "
                    f"success={resp.success} msg='{resp.message}'"
                )
            else:
                self.get_logger().info(
                    f"{svc_name} response (sent @{call_time:.2f}s) → {resp}"
                )
        except Exception as e:
            self.get_logger().error(f"{svc_name} response failed: {e}")


# ─────────────────────────── CLI entry ──────────────────────────────────
def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-f", "--file", required=True, help="YAML script file")
    parser.add_argument("-r", "--rate_hz", type=float, default=20.0,
                        help="Publish frequency (default 20 Hz)")

    clean = rclpy.utilities.remove_ros_args(sys.argv[1:])
    args = parser.parse_args(clean)

    rclpy.init(args=sys.argv)
    node = CommandPlayer(args.file, args.rate_hz)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
