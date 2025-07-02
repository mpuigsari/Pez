#!/usr/bin/env python3
"""
server.py – BlueROV motor / servo driver (ROS Noetic)

Interfaces
----------
/start       std_srvs/Trigger   → arm   vehicle
/stop        std_srvs/Trigger   → disarm vehicle
/lights_on   std_srvs/Trigger   → lights 1900 µs  (ON)
/lights_off  std_srvs/Trigger   → lights 1100 µs  (OFF)

/cmd_vel       geometry_msgs/Twist  thruster control
/camara_servo  std_msgs/Int32       camera-tilt PWM 1100-1900
"""

# --------------------------------------------------------------------------- #
# 1)  Imports
# --------------------------------------------------------------------------- #
from pymavlink import mavutil
import rospy, time, numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg     import Int32
from std_srvs.srv     import Trigger, TriggerResponse

# --------------------------------------------------------------------------- #
# 2)  Polynomial LUT for thrusters (unchanged)
# --------------------------------------------------------------------------- #
coefficients = np.array([
    1.3871668161975536e-25, -2.080750463000313e-21,  1.3983334487807407e-17,
   -5.543948233018681e-14,  1.4359451930265564e-10, -2.5387836010458446e-07,
    0.0003102899349247361, -0.25886076958834003,   141.07497402237286,
  -45353.82880615414,      6531725.54047377 ])
x           = np.linspace(1100, 1900, 801)
y           = np.polyval(coefficients, x)
velocidades = np.where(np.arange(len(x)) < 400, -y, y)

servvelant = 1500                                              # camera servo cache

# --------------------------------------------------------------------------- #
# 3)  MAVLink connection
# --------------------------------------------------------------------------- #
master = mavutil.mavlink_connection('udpin:192.168.2.2:14565')
master.wait_heartbeat()
rospy.loginfo("Connected to MAVLink UDP endpoint")


def set_rc_channel_pwm(ch: int, pwm: int):
    """Send a single RC override (1100-1900 µs)."""
    if not 1 <= ch <= 18:
        rospy.logwarn("RC channel %s out of range", ch)
        return
    values = [65535] * 18
    values[ch - 1] = pwm
    master.mav.rc_channels_override_send(master.target_system,
                                         master.target_component, *values)

# --------------------------------------------------------------------------- #
# 4)  Arm / disarm helpers
# --------------------------------------------------------------------------- #
def armar_rov():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    rospy.loginfo("ROV armed")

def desarmar_rov():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    rospy.loginfo("ROV disarmed")

# --------------------------------------------------------------------------- #
# 5)  Service callbacks  (all Trigger)
# --------------------------------------------------------------------------- #
def handle_start(_req):
    armar_rov()
    return TriggerResponse(success=True, message="ROV armed")

def handle_stop(_req):
    desarmar_rov()
    return TriggerResponse(success=True, message="ROV disarmed")

def handle_lights_on(_req):
    set_rc_channel_pwm(9, 1900)
    rospy.loginfo("Lights ON (PWM 1900)")
    return TriggerResponse(success=True, message="Lights ON")

def handle_lights_off(_req):
    set_rc_channel_pwm(9, 1100)
    rospy.loginfo("Lights OFF (PWM 1100)")
    return TriggerResponse(success=True, message="Lights OFF")

# --------------------------------------------------------------------------- #
# 6)  Topic callbacks
# --------------------------------------------------------------------------- #
def cmd_vel_cb(msg: Twist):
    # X
    xpwm = 1500
    if -1.0 <= msg.linear.x <= 1.0 and msg.linear.x != 0.0:
        xpwm = np.argmin(np.abs(velocidades - msg.linear.x)) + 1100
    set_rc_channel_pwm(5, xpwm)

    # Y
    ypwm = 1500
    if -1.0 <= msg.linear.y <= 1.0 and msg.linear.y != 0.0:
        ypwm = np.argmin(np.abs(velocidades - msg.linear.y)) + 1100
    set_rc_channel_pwm(6, ypwm)

    # Z
    zpwm = 1500
    if -1.0 <= msg.linear.z <= 1.0 and msg.linear.z != 0.0:
        zpwm = np.argmin(np.abs(velocidades - msg.linear.z)) + 1100
    set_rc_channel_pwm(3, zpwm)

    # Yaw
    yawpwm = 1500
    if -1.0 <= msg.angular.z <= 1.0 and msg.angular.z != 0.0:
        yawpwm = np.argmin(np.abs(velocidades - msg.angular.z)) + 1100
    set_rc_channel_pwm(4, yawpwm)

def camera_servo_cb(msg: Int32):
    global servvelant
    if 1100 <= msg.data <= 1900 and msg.data != servvelant:
        set_rc_channel_pwm(8, msg.data)
        time.sleep(3)
        servvelant = msg.data
        set_rc_channel_pwm(8, 1500)

# --------------------------------------------------------------------------- #
# 7)  ROS node setup
# --------------------------------------------------------------------------- #
def main():
    rospy.init_node('bluerov_server', anonymous=False)
    rospy.loginfo("ROS node up")

    # Subscribers
    rospy.Subscriber('cmd_vel', Twist,  cmd_vel_cb)
    rospy.Subscriber('camara_servo', Int32, camera_servo_cb)

    # Services
    rospy.Service('/start',       Trigger, handle_start)
    rospy.Service('/stop',        Trigger, handle_stop)
    rospy.Service('/lights_on',   Trigger, handle_lights_on)
    rospy.Service('/lights_off',  Trigger, handle_lights_off)
    rospy.loginfo("Services [/start] [/stop] [/lights_on] [/lights_off] advertised")

    rospy.spin()

if __name__ == '__main__':
    main()
