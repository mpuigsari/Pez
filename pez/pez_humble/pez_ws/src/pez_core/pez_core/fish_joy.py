#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

class JoystickController(Node):
    def __init__(self):
        super().__init__('joy_controller')

                # 1) Declare each sub‐key as its own scalar parameter:
        self.declare_parameter('axes_forward', 4)
        self.declare_parameter('axes_turn',    0)
        self.declare_parameter('axes_dive',    1)
        self.declare_parameter('axes_camera',  6)

        self.declare_parameter('scale_forward',  1.0)
        self.declare_parameter('scale_turn',    -1.0)
        self.declare_parameter('scale_dive',     1.0)
        self.declare_parameter('scale_camera',  -1.0)

        self.declare_parameter('button_start',   7)
        self.declare_parameter('button_stop',    6)
        self.declare_parameter('button_magnet',  2)
        self.declare_parameter('button_mode_0',  3)
        self.declare_parameter('button_neutral',  4)

        self.declare_parameter('start_service', 'teleoperation/start_swim')
        self.declare_parameter('stop_service', 'teleoperation/stop_swim')
        self.declare_parameter('magnet_service', 'teleoperation/toggle_magnet')
        self.declare_parameter('neutral_service', 'teleoperation/toggle_neutral')

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('camera_topic', 'camera_control')

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
        self.cmd_vel_pub = self.create_publisher(Twist,   self.get_parameter('cmd_vel_topic').value, 10)
        self.camera_pub  = self.create_publisher(Float64, self.get_parameter('camera_topic').value,  10)

        # 3) Service clients
        
        self.start_cli  = self.create_client(Trigger, self.get_parameter('start_service').value)
        self.stop_cli   = self.create_client(Trigger, self.get_parameter('stop_service').value)
        self.magnet_cli = self.create_client(Trigger, self.get_parameter('magnet_service').value)
        self.neutral_cli = self.create_client(Trigger, self.get_parameter('neutral_service').value)

        # 4) Internal state
        self.mode = -1
        self.magnet_on = False
        self.prev_magnet_btn = False
        self.neutral_on = False
        self.prev_neutral_btn = False
        self.lock = threading.Lock()
        self.pez_body_cmd     = Twist()
        self.pez_camera_float = Float64()

        # 5) Subscriber + timer
        self.create_subscription(Joy, self.get_parameter('joy_topic').value, self.joy_callback, 10)
        self.create_timer(0.05, self.iterate)

        self.get_logger().info(f"[JoystickController] Initialized in mode {self.mode}")

    def joy_callback(self, msg: Joy):
        self.joy_msg = msg
        self.process_buttons()
        self.process_axes()

    def process_buttons(self):
        b = self.joy_msg.buttons
        # Start
        if b[self.buttons['start']] and self.mode != 0:
            self.mode = 0
            self.call_service(self.start_cli, "start")
        # Stop
        if b[self.buttons['stop']] and self.mode != -1:
            self.mode = -1
            self.call_service(self.stop_cli, "stop")
        # Toggle magnet
        mag_btn = bool(b[self.buttons['magnet']])
        if mag_btn and not self.prev_magnet_btn:
            self.magnet_on = not self.magnet_on
            self.call_service(self.magnet_cli, "magnet")
        self.prev_magnet_btn = mag_btn

        # Toggle neutral
        n_btn = bool(b[self.buttons['neutral']])
        if n_btn and not self.prev_neutral_btn:
            self.neutral_on = not self.neutral_on
            self.call_service(self.neutral_cli, "neutral")
        self.prev_neutral_btn = n_btn

    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"{name} service not available")
            return

        req = Trigger.Request()
        fut = client.call_async(req)
        fut.add_done_callback(lambda future, n=name: self._on_response(future, n))

    def _on_response(self, future, name):
        try:
            resp = future.result()
            if resp.success:
                # special-case magnet and neutral so we log their new state
                if name == 'magnet':
                    state = 'ON' if self.magnet_on else 'OFF'
                    self.get_logger().info(f"{name} service succeeded → magnet is now {state}")
                elif name == 'neutral':
                    state = 'ON' if self.neutral_on else 'OFF'
                    self.get_logger().info(f"{name} service succeeded → neutral mode is now {state}")
                else:
                    self.get_logger().info(f"{name} service succeeded")
            else:
                self.get_logger().warn(f"{name} service returned failure")
        except Exception as e:
            self.get_logger().error(f"{name} service call exception: {e}")



    def process_axes(self):
        a = self.joy_msg.axes
        with self.lock:
            self.pez_body_cmd.linear.x = a[self.axes['forward']]*self.scales['forward']
            self.pez_body_cmd.linear.y = a[self.axes['turn']]   *self.scales['turn']
            self.pez_body_cmd.linear.z = a[self.axes['dive']]   *self.scales['dive']
            self.pez_camera_float.data = a[self.axes['camera']]*self.scales['camera']

    def iterate(self):
        with self.lock:
            if self.mode == 0:
                self.cmd_vel_pub.publish(self.pez_body_cmd)
                self.camera_pub.publish(self.pez_camera_float)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
