#!/usr/bin/env python3
import threading
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

class JoystickController:
    def __init__(self):
        rospy.init_node('joy_controller', anonymous=True)
        self.load_params()
        self.init_publishers()
        self.init_services()

        # Internal state
        self.mode = -1                  # -1=idle, 0=manual
        self.magnet_on = False
        self.joy_msg = None
        self.lock = threading.Lock()

        # Internal command buffers
        self.pez_body_cmd    = Twist()
        self.pez_camera_float= Float64()

        # Subscribe and start timer
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)
        # 10 Hz timer for publishing commands
        rospy.Timer(rospy.Duration(0.1), self.iterate)

        rospy.loginfo("[JoystickController] Initialized in mode %d", self.mode)
        rospy.spin()

    def load_params(self):
        # Topics
        self.joy_topic     = rospy.get_param('~joy_topic', 'joy')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
        self.camera_topic  = rospy.get_param('~camera_topic', 'camera_control')

        # Axes mapping & scales
        self.axes   = rospy.get_param('~axes',   {'forward':4,'turn':0,'dive':1,'camera':6})
        self.scales = rospy.get_param('~scales', {'forward':1.0,'turn':-1.0,'dive':1.0,'camera':-1.0})

        # Buttons mapping
        self.buttons= rospy.get_param('~buttons', {
            'start':    7,  # e.g. OPTIONS
            'stop':     6,  # e.g. SHARE
            'magnet':   2,  # e.g. X
            'mode_0':   3,  # e.g. Y
        })

        # Service names (Trigger)
        self.start_srv_name  = rospy.get_param('~start_service',  '')
        self.stop_srv_name   = rospy.get_param('~stop_service',   '')
        self.magnet_srv_name = rospy.get_param('~magnet_service', '')

    def init_publishers(self):
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist,   queue_size=1)
        self.camera_pub  = rospy.Publisher(self.camera_topic,  Float64, queue_size=1)

    def init_services(self):
        def make_proxy(name):
            if name:
                rospy.wait_for_service(name, timeout=5)
                
                return rospy.ServiceProxy(name, Trigger)
            return None

        self.start_srv  = make_proxy(self.start_srv_name)
        self.stop_srv   = make_proxy(self.stop_srv_name)
        self.magnet_srv = make_proxy(self.magnet_srv_name)

    def joy_callback(self, joy: Joy):
        # Called whenever a new /joy arrives
        self.joy_msg = joy

        # 1) Button-driven actions + mode changes
        self.process_buttons()

        # 2) Axis-driven command updates
        self.process_axes()

    def process_buttons(self):
        b = self.joy_msg.buttons

        # Start → enter manual mode (0)
        if self.start_srv and b[self.buttons['start']]:
            try:
                self.start_srv()
                self.mode = 0
                rospy.loginfo("[JoystickController] Mode → %d (manual)", self.mode)
            except rospy.ServiceException as e:
                rospy.logwarn("[JoystickController] start_service failed: %s", e)

        # Stop → idle (-1)
        if self.stop_srv and b[self.buttons['stop']]:
            try:
                self.stop_srv()
                self.mode = -1
                rospy.loginfo("[JoystickController] Mode → %d (idle)", self.mode)
            except rospy.ServiceException as e:
                rospy.logwarn("[JoystickController] stop_service failed: %s", e)

        # Toggle electromagnet
        if self.magnet_srv and b[self.buttons['magnet']]:
            self.magnet_on = not self.magnet_on
            try:
                self.magnet_srv()
                rospy.loginfo("Magnet → %s", self.magnet_on)
            except rospy.ServiceException as e:
                rospy.logwarn("[JoystickController] magnet_service failed: %s", e)

        # Direct set to manual mode 0
        if b[self.buttons['mode_0']]:
            if self.mode != 0:
                self.mode = 0
                rospy.loginfo("[JoystickController] Mode → %d (manual)", self.mode)

    def process_axes(self):
        # Build the Twist and Float64 commands
        a = self.joy_msg.axes

        # Acquire lock so iterate() won't publish half-updated commands
        with self.lock:
            # Always update these buffers, even if mode==-1
            self.pez_body_cmd.linear.x = a[self.axes['forward']] * self.scales['forward']
            self.pez_body_cmd.linear.y = a[self.axes['turn']]    * self.scales['turn']
            self.pez_body_cmd.linear.z = a[self.axes['dive']]    * self.scales['dive']

            self.pez_camera_float.data = a[self.axes['camera']] * self.scales['camera']

    def iterate(self, event):
        """10 Hz timer: publish buffered commands based on current mode."""
        with self.lock:
            if self.mode == 0:  # manual teleop
                self.cmd_vel_pub.publish(self.pez_body_cmd)
                self.camera_pub.publish(self.pez_camera_float)
                rospy.logdebug("[JoystickController] Iterate pub  cmd_vel x=%.2f y=%.2f z=%.2f | cam=%.2f",
                    self.pez_body_cmd.linear.x,
                    self.pez_body_cmd.linear.y,
                    self.pez_body_cmd.linear.z,
                    self.pez_camera_float.data)
            # elif self.mode == 1:  # future autonomous
            #     ... publish different topics ...
            # mode -1 → do nothing

if __name__ == '__main__':
    try:
        JoystickController()
    except rospy.ROSInterruptException:
        pass
