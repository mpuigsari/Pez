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

        # 1) Declare & get parameters
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('camera_topic', 'camera_control')
        
        self.declare_parameter('axes',   {'forward': 4, 'turn': 0, 'dive': 1, 'camera': 6})
        self.declare_parameter('scales', {'forward': 1.0, 'turn': -1.0, 'dive': 1.0, 'camera': -1.0})
        self.declare_parameter('buttons',{'start': 7, 'stop': 6, 'magnet': 2, 'mode_0': 3})

        # 2) grab it back in one line each
        self.axes   = self.get_parameter('axes').value
        self.scales = self.get_parameter('scales').value
        self.buttons= self.get_parameter('buttons').value

        self.joy_topic     = self.get_parameter('joy_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.camera_topic  = self.get_parameter('camera_topic').value

        # 2) Publishers
        self.cmd_vel_pub = self.create_publisher(Twist,   self.cmd_vel_topic, 10)
        self.camera_pub  = self.create_publisher(Float64, self.camera_topic,  10)

        # 3) Service clients
        self.start_cli  = self.create_client(Trigger, 'start_service_name')
        self.stop_cli   = self.create_client(Trigger, 'stop_service_name')
        self.magnet_cli = self.create_client(Trigger, 'magnet_service_name')

        # 4) Internal state
        self.mode = -1
        self.magnet_on = False
        self.prev_magnet_btn = False
        self.lock = threading.Lock()
        self.pez_body_cmd     = Twist()
        self.pez_camera_float = Float64()

        # 5) Subscriber + timer
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.create_timer(0.1, self.iterate)

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

    def call_service(self, client, name):
        if client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result().success:
                self.get_logger().info(f"{name} service succeeded")
            else:
                self.get_logger().warn(f"{name} service failed")
        else:
            self.get_logger().error(f"{name} service not available")

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
