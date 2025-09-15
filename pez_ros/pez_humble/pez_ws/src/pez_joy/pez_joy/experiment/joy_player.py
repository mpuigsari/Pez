#!/usr/bin/env python3
"""
ROS 2 Humble node that publishes a `sensor_msgs/Joy` message at a user-defined **timer period**.

### Key points
* Period is read from the **ROS parameter** `timer_sec` (default **4.0 s**).
  ```bash
  # One press every 4 s (default)
  ros2 run pez_joy joy_player

  # One press every 0.5 s (timer = 0.5 s)
  ros2 run pez_joy joy_player --ros-args -p timer_sec:=0.5
  ```
* Works transparently inside launch files:
  ```python
  Node(
      package="pez_joy", executable="joy_player",
      parameters=[{"timer_sec": 0.25}],
  )
  ```
* **Namespace-agnostic**: topic name is relative (`joy`), so any `PushRosNamespace` in launch will prefix it automatically.
* Other configurable parameters:
  * `sequence` (list[int]) – default `[7,2,4,4,2,6]`
  * `max_presses` (int)    – default 500
  * `num_buttons`, `axis_count`, `topic`

The node keeps **default QoS** (depth 10, RELIABLE, VOLATILE).
"""
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyScheduler(Node):
    """Publishes a Joy message following a button sequence at regular intervals."""

    def __init__(self):
        super().__init__('joy_scheduler')

        # -------------------- Parameters --------------------
        self.declare_parameter('timer_sec', 4.0)   # ⏲️ NEW
        self.declare_parameter('topic', 'pez/joy')
        self.declare_parameter('num_buttons', 12)
        self.declare_parameter('axis_count', 8)
        self.declare_parameter('sequence', [7, 2, 4, 4, 2, 6])
        self.declare_parameter('max_presses', 500)

        period_sec  = float(self.get_parameter('timer_sec').value)
        topic       = self.get_parameter('topic').value
        self.num_btn  = int(self.get_parameter('num_buttons').value)
        self.axis_cnt = int(self.get_parameter('axis_count').value)
        self.sequence: List[int] = [int(x) for x in self.get_parameter('sequence').value]
        self.max_presses = int(self.get_parameter('max_presses').value)

        # -------------------- Publisher --------------------
        self.publisher_ = self.create_publisher(Joy, topic, 10)  # default QoS
        full_topic = self.publisher_.topic  # fully-qualified with namespace

        self.get_logger().info(
            f"Publishing on '{full_topic}' every {period_sec}s | Sequence={self.sequence} | "
            f"max_presses={self.max_presses} | buttons={self.num_btn}, axes={self.axis_cnt}"
        )

        # -------------------- State & Timer --------------------
        self._seq_idx = 0
        self._press_count = 0
        self.timer = self.create_timer(period_sec, self._timer_cb)

    # -------------------- Timer Callback --------------------
    def _timer_cb(self):
        if self._press_count >= self.max_presses:
            self.get_logger().info('Reached max_presses → shutting down.')
            self.timer.cancel()
            self.create_timer(0.5, lambda: rclpy.shutdown())
            return

        button = self.sequence[self._seq_idx]
        if button >= self.num_btn:
            self.get_logger().error(
                f"Sequence value {button} exceeds num_buttons ({self.num_btn}). Skipping.")
            self._advance()
            return

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes    = [0.0] * self.axis_cnt
        msg.buttons = [0]   * self.num_btn
        msg.buttons[button] = 1

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"[{self._press_count + 1}/{self.max_presses}] Button {button} → published")

        self._advance()

    # -------------------- Helpers --------------------
    def _advance(self):
        self._seq_idx = (self._seq_idx + 1) % len(self.sequence)
        self._press_count += 1


# ==================== main ====================

def main(args=None):
    rclpy.init(args=args)
    node = JoyScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt – shutting down…')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()