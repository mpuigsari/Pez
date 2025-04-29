#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop', anonymous=True)

        # Publisher for cmd_vel
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Parameters for axis mapping and scaling
        self.axis_forward = rospy.get_param('~axis_forward', 1)   # typically left stick vertical
        self.axis_turn    = rospy.get_param('~axis_turn',    0)   # typically left stick horizontal
        self.axis_dive    = rospy.get_param('~axis_dive',    4)   # e.g. right stick vertical

        self.scale_forward = rospy.get_param('~scale_forward', 1.0)
        self.scale_turn    = rospy.get_param('~scale_turn',    1.0)
        self.scale_dive    = rospy.get_param('~scale_dive',    1.0)

        # Subscribe to joystick messages
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.loginfo("[JoyTeleop] Initialized. Publishing /cmd_vel from /joy inputs.")
        rospy.spin()

    def joy_callback(self, joy_msg):
        # Create Twist message
        twist = Twist()

        # Map joystick axes to Twist fields
        # Forward/backward
        try:
            forward = joy_msg.axes[self.axis_forward]
        except IndexError:
            forward = 0.0
        twist.linear.x = forward * self.scale_forward

        # Left/right yaw turn
        try:
            turn = joy_msg.axes[self.axis_turn]
        except IndexError:
            turn = 0.0
        twist.linear.y = turn * self.scale_turn

        # Up/down pitch (dive/climb)
        try:
            dive = joy_msg.axes[self.axis_dive]
        except IndexError:
            dive = 0.0
        twist.linear.z = dive * self.scale_dive

        # No angular.z usage (we handle yaw in linear.y for fish)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # Publish the command
        self.pub.publish(twist)

        rospy.logdebug(f"[JoyTeleop] published Twist: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, z={twist.linear.z:.2f}")

if __name__ == '__main__':
    try:
        JoyTeleop()
    except rospy.ROSInterruptException:
        pass
