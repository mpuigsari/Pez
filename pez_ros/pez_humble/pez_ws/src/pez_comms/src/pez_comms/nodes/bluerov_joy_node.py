#!/usr/bin/env python3
"""
joy_teleop.py – translate /joy into

    • <ns>/cmd_vel      geometry_msgs/Twist      (published at ≤ 1 Hz)
    • <ns>/start        std_srvs/Trigger         ← “start / options” button
    • <ns>/stop         std_srvs/Trigger         ← “back / select”   button
    • <ns>/lights_on    std_srvs/Trigger         ← RB  (right-bumper)
    • <ns>/lights_off   std_srvs/Trigger         ← LB  (left-bumper)

All names are **relative**, so a single PushRosNamespace in the launch file
keeps everything under your chosen <ns>.

ROS 2 Humble.
"""

import argparse, yaml, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class JoyTeleop(Node):
    def __init__(self, cfg: dict):
        super().__init__('joy_teleop')

        prm              = cfg['joy_teleop']['ros__parameters']
        self.axis_idx    = prm['axis_map']
        self.scale       = prm.get('scale_map',
                                   {'vx': 1.0, 'vy': 1.0, 'vz': 1.0, 'wz': 1.0})
        self.buttons     = prm['button_map']
        self.deadzone    = prm.get('deadzone', 0.1)

        # ── publish rate bookkeeping ───────────────────────────
        self.rate_hz      = prm.get('publish_hz', 1.0)
        self._period_ns   = int(1e9 / self.rate_hz)
        self._last_pub_ns = 0
        self.last_twist   = Twist()

        # ── publishers & service-clients (relative names) ─────
        self.pub_twist     = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cli_start     = self.create_client(Trigger, 'start')
        self.cli_stop      = self.create_client(Trigger, 'stop')
        self.cli_light_on  = self.create_client(Trigger, 'lights_on')
        self.cli_light_off = self.create_client(Trigger, 'lights_off')

        # rising-edge detector
        self.prev_buttons = []

        # subscriptions & timers
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.create_timer(1.0 / self.rate_hz, self._timer_republish)

        self.get_logger().info(
            f'joy_teleop up in ns [{self.get_namespace()}] – '
            f'cmd_vel capped at {self.rate_hz:.1f} Hz'
        )

    # ──────────────────────────────────────────────────────────
    def joy_cb(self, joy: Joy):
        # -------- AXES → Twist ---------------------------------
        idx, sc = self.axis_idx, self.scale

        def take(i, gain):
            v = joy.axes[i] if abs(joy.axes[i]) >= self.deadzone else 0.0
            return v * gain

        t = Twist()
        t.linear.x  = take(idx['vx'], sc['vx'])
        t.linear.y  = take(idx['vy'], sc['vy'])
        t.linear.z  = take(idx['vz'], sc['vz'])
        t.angular.z = take(idx['wz'], sc['wz'])
        self.last_twist = t

        # immediate publish – throttled to 1 Hz
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_pub_ns >= self._period_ns:
            self._publish_twist(t, now_ns)

        # -------- BUTTONS → services ---------------------------
        if not self.prev_buttons:          # first callback
            self.prev_buttons = joy.buttons
            return
        pb, b = self.prev_buttons, joy.buttons

        def rise(key):
            i = self.buttons[key]
            return b[i] == 1 and pb[i] == 0

        if rise('start'):
            self._call_trigger(self.cli_start, 'start')
        if rise('stop'):
            self._call_trigger(self.cli_stop,  'stop')
        if rise('lights_on'):
            self._call_trigger(self.cli_light_on,  'lights_on')
        if rise('lights_off'):
            self._call_trigger(self.cli_light_off, 'lights_off')

        self.prev_buttons = b

    # ──────────────────────────────────────────────────────────
    def _timer_republish(self):
        """Always re-publish the latest Twist once per period."""
        self._publish_twist(self.last_twist,
                            self.get_clock().now().nanoseconds)

    def _publish_twist(self, twist: Twist, now_ns: int):
        self.pub_twist.publish(twist)
        self._last_pub_ns = now_ns

    # ──────────────────────────────────────────────────────────
    def _call_trigger(self, client, name):
        if client.wait_for_service(timeout_sec=0.5):
            client.call_async(Trigger.Request())
            self.get_logger().info(f'Called {name}()')
        else:
            self.get_logger().warn(f'{name} service unavailable')


# ─────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-c', '--config', required=True,
                    help='YAML file with axis / button mapping')
    args, _ = ap.parse_known_args()

    with open(args.config, 'r') as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    node = JoyTeleop(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
