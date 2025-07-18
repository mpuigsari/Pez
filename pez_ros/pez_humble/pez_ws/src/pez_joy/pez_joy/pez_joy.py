#!/usr/bin/env python3
import os
import csv
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


from pez_interfaces.srv import SnapSensors


class JoystickController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # 1) Declare each sub-key as its own scalar parameter:
        self.declare_parameter('axes_forward', 4)
        self.declare_parameter('axes_turn',    0)
        self.declare_parameter('axes_dive',    1)
        self.declare_parameter('axes_camera',  6)

        self.declare_parameter('scale_forward',  1.0)
        self.declare_parameter('scale_turn',     1.0)
        self.declare_parameter('scale_dive',    -1.0)
        self.declare_parameter('scale_camera',   1.0)

        self.declare_parameter('button_start',   7)
        self.declare_parameter('button_stop',    6)
        self.declare_parameter('button_magnet',  2)
        self.declare_parameter('button_mode_0',  3)
        self.declare_parameter('button_neutral', 4)

        self.declare_parameter('start_service',   'teleoperation/swim_start')
        self.declare_parameter('stop_service',    'teleoperation/swim_stop')
        self.declare_parameter('magnet_service',  'teleoperation/toggle_magnet')
        self.declare_parameter('neutral_service', 'teleoperation/toggle_neutral')

        self.declare_parameter('joy_topic',       'joy')
        self.declare_parameter('cmd_vel_topic',   'cmd_vel')
        self.declare_parameter('camera_topic',    'camera_control')

        # 2) Read them back into your dicts:
        self.axes = {
            'forward': self.get_parameter('axes_forward').value,
            'turn':    self.get_parameter('axes_turn').value,
            'dive':    self.get_parameter('axes_dive').value,
            'camera':  self.get_parameter('axes_camera').value,
        }

        self.scales = {
            'forward': self.get_parameter('scale_forward').value,
            'turn':    self.get_parameter('scale_turn').value,
            'dive':    self.get_parameter('scale_dive').value,
            'camera':  self.get_parameter('scale_camera').value,
        }

        self.buttons = {
            'start':  self.get_parameter('button_start').value,
            'stop':   self.get_parameter('button_stop').value,
            'magnet': self.get_parameter('button_magnet').value,
            'mode_0': self.get_parameter('button_mode_0').value,
            'neutral': self.get_parameter('button_neutral').value,
        }

        # 2) Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )
        self.camera_pub = self.create_publisher(
            Float64,
            self.get_parameter('camera_topic').value,
            10
        )

        # 3) Service clients for teleop calls
        ns = self.get_namespace().strip('/')
        if ns:
            ns = '/' + ns

        self.start_cli   = self.create_client(Trigger, f"{ns}/{self.get_parameter('start_service').value}")
        self.stop_cli    = self.create_client(Trigger, f"{ns}/{self.get_parameter('stop_service').value}")
        self.magnet_cli  = self.create_client(Trigger, f"{ns}/{self.get_parameter('magnet_service').value}")
        self.neutral_cli = self.create_client(Trigger, f"{ns}/{self.get_parameter('neutral_service').value}")

        # 3a) Snapshot service client
        self.snapshot_cli = self.create_client(SnapSensors, 'get_sensor_snapshot')

        # 4) Internal state
        self.mode                = -1
        self.magnet_on           = False
        self.prev_magnet_btn     = False
        self.neutral_on          = False
        self.prev_neutral_btn    = False
        self.lock                = threading.Lock()
        self.pez_body_cmd        = Twist()
        self.pez_camera_float    = Float64()

        # ─── Flags to track when both calls succeed ───
        self._pending_magnet_success   = False
        self._pending_snapshot_success = False
        self._last_snapshot_response   = None

        # 5) Subscriber + timer
        self.create_subscription(
            Joy,
            self.get_parameter('joy_topic').value,
            self.joy_callback,
            10
        )
        self.create_timer(0.05, self.iterate)



        # 6) snapshots directory (inside the pez_joy package)
        pkg_root = Path(get_package_share_directory("pez_joy")).resolve()
        snap_dir = pkg_root / "snapshots"          # → src/pez_joy/snapshots  (symlink-install)
        snap_dir.mkdir(parents=True, exist_ok=True)

        self._snapshot_dir = str(snap_dir)
        self.get_logger().info(f"Snapshots will be stored in {self._snapshot_dir}")

        self.get_logger().info(f"[JoystickController] Initialized in mode {self.mode}")

    def joy_callback(self, msg: Joy):
        self.joy_msg = msg
        self.process_buttons()
        self.process_axes()

    def process_buttons(self):
        """
        Handle button edges.

        • Magnet button logic:
            – On every edge we flip self.magnet_on and call the toggle-magnet service.
            – No snapshot is taken here; it will be triggered from _on_response()
            *after* a successful magnet-OFF response.
        """
        b = self.joy_msg.buttons

        # ── Start / Stop swim ────────────────────────────────────────────────────
        if b[self.buttons['start']] and self.mode != 0:
            self.mode = 0
            self.call_service(self.start_cli, "start")

        if b[self.buttons['stop']] and self.mode != -1:
            self.mode = -1
            self.call_service(self.stop_cli, "stop")

        # ── Magnet toggle ───────────────────────────────────────────────────────
        mag_btn = bool(b[self.buttons['magnet']])
        if mag_btn and not self.prev_magnet_btn:
            self.magnet_on = not self.magnet_on
            self.call_service(self.magnet_cli, "magnet")
        self.prev_magnet_btn = mag_btn

        # ── Neutral ballast toggle ──────────────────────────────────────────────
        n_btn = bool(b[self.buttons['neutral']])
        if n_btn and not self.prev_neutral_btn:
            self.neutral_on = not self.neutral_on
            self.call_service(self.neutral_cli, "neutral")
        self.prev_neutral_btn = n_btn

    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{name} service not available")
            return

        req = Trigger.Request()
        fut = client.call_async(req)
        fut.add_done_callback(lambda future, n=name: self._on_response(future, n))

    def _on_response(self, future, name):
        """
        Handle service responses.

        • When a **magnet-OFF** response succeeds:
            – Trigger a sensor snapshot.
        """
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"{name} service call exception: {e}")
            return

        if not resp.success:
            self.get_logger().warn(f"{name} service returned failure")
            return

        # ────────────────────────────────────────────────────────────────────────
        if name == 'neutral':
            state = 'ON' if self.neutral_on else 'OFF'
            self.get_logger().info(f"{name} service succeeded → neutral mode is now {state}")
            return

        if name != 'magnet':
            self.get_logger().info(f"{name} service succeeded")
            return

        # ── Magnet success ──────────────────────────────────────────────────────
        state = 'ON' if self.magnet_on else 'OFF'
        self.get_logger().info(f"{name} service succeeded → magnet is now {state}")

        # If the magnet was JUST TURNED OFF, capture a snapshot now
        if not self.magnet_on:
            self._call_snapshot()

    def _call_snapshot(self):
        if not self.snapshot_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Snapshot service not available")
            return

        req = SnapSensors.Request()
        fut = self.snapshot_cli.call_async(req)
        fut.add_done_callback(self._on_snapshot_response)


    def _on_snapshot_response(self, future):
        """
        Write the CSV as soon as the snapshot succeeds.
        (This function is only called after a successful magnet-OFF.)
        """
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Snapshot service call failed: {e}")
            return

        if not resp.success:
            self.get_logger().warn(f"Snapshot service returned failure: {resp.message}")
            return

        self._write_snapshot_csv(resp)

    def _write_snapshot_csv(self, resp):
        """
        Write a CSV using resp.header.stamp and resp.<sensor> values.
        Called only when both magnet and snapshot services have succeeded.
        """
        # Convert header.stamp into human-readable datetime
        sec  = resp.header.stamp.sec
        nsec = resp.header.stamp.nanosec
        ts_ros = sec + nsec * 1e-9

        dt_utc   = datetime.fromtimestamp(ts_ros, tz=timezone.utc)
        dt_local = dt_utc.astimezone()
        human_readable = dt_local.strftime('%Y-%m-%d %H:%M:%S.%f')

        # Create a safe filename
        safe_ts = dt_local.strftime('%Y%m%d_%H%M%S_%f')  # e.g. "20250607_142305_123456"
        fname = os.path.join(self._snapshot_dir, f"magnet_snapshot_{safe_ts}.csv")

        try:
            with open(fname, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'datetime',
                    'tsys01_temp',
                    'ms5837_temp',
                    'ms5837_pressure',
                    'ms5837_depth'
                ])
                writer.writerow([
                    human_readable,
                    resp.tsys01_temp,
                    resp.ms5837_temp,
                    resp.ms5837_pressure,
                    resp.ms5837_depth,
                ])
            self.get_logger().info(f"Snapshot written to {fname}")
        except Exception as e:
            self.get_logger().error(f"Failed to write snapshot CSV: {e}")

        # Clear flags so a future ON-toggle can repeat the process
        self._pending_magnet_success   = False
        self._pending_snapshot_success = False
        self._last_snapshot_response   = None

    def process_axes(self):
        a = self.joy_msg.axes
        with self.lock:
            self.pez_body_cmd.linear.x  = a[self.axes['forward']] * self.scales['forward']
            self.pez_body_cmd.linear.y  = a[self.axes['turn']]    * self.scales['turn']
            self.pez_body_cmd.linear.z  = a[self.axes['dive']]    * self.scales['dive']
            self.pez_camera_float.data  = a[self.axes['camera']]  * self.scales['camera']

    def iterate(self):
        with self.lock:
            if self.mode == 0:
                self.cmd_vel_pub.publish(self.pez_body_cmd)
                self.camera_pub.publish(self.pez_camera_float)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt—shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
