#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import time
import threading



class PezController:
    def __init__(self):
        rospy.init_node('pez_move_controller', anonymous=True)

        # Initialize Navigator
        navigator.init()
        navigator.set_pwm_enable(True)
        navigator.set_pwm_freq_hz(50)

        # Subscribe to /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.controller_callback)

        # Initial control values
        self.tail_pwm = PWMValue(303)      # [176.1, 303, 434]PWM [30,90,150]º      Ch16
        self.lat0_pwm = PWMValue(231)      # [153.5, 231, 307]PWM [45,113,180]º     Ch14
        self.lat1_pwm = PWMValue(231)      # [153.5, 231, 307]PWM [45,113,180]º     Ch15
        self.camera_angle = PWMValue(315)  # [XX, 315, XX]PWM [100,180,260]º        Ch11
        self.timer_movement = PWMValue(0.30)

        self.setup_pwm()

        # Start Movement Control Loop
        self.thread = threading.Thread(target=self.send_movement)
        self.thread.start()

        rospy.loginfo("[PezController] Node Initialized.")

    def setup_pwm(self):
        navigator.set_pwm_channels_values([PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15, PwmChannel.Ch11 ], 
                                           [self.tail_pwm.default, 
                                            self.lat0_pwm.default, 
                                            self.lat1_pwm.default, 
                                            self.camera_angle.default])
        rospy.loginfo("[PezController] PWM Values Initialized.")

    def send_movement(self):
        while not rospy.is_shutdown():
            navigator.set_pwm_channels_values([PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15, PwmChannel.Ch11 ], 
                                           [self.tail_pwm.current, 
                                            self.lat0_pwm.current, 
                                            self.lat1_pwm.current, 
                                            self.camera_angle.current])
            time.sleep(self.timer_movement)
        rospy.loginfo("[PezController] Movement thread stopped.")

    def controller_callback(self, msg):
        # Map Twist linear velocities to PWM signals
        x = msg.linear.x  # Forward
        y = msg.linear.y  # Left/right
        z = msg.linear.z  # Up/down


    def up_down(self, z):
        if z == 0.0:
            return
        rospy.loginfo("[PezController] Upping/Downing.")

    def left_right(self, y):
        if y == 0:
            
            rospy.loginfo("[PezController] Lefting/Righting.")
            return
        rospy.loginfo("[PezController] Lefting/Righting.")

    def set_vel(self, x):
        rospy.loginfo("[PezController] Velocitying.")

    def shutdown(self):
        self.thread.join()

    def run(self):
        rospy.spin()


class PWMValue:
    def __init__(self, default):
        self.default = default
        self.current = default


if __name__ == '__main__':
    try:
        controller = PezController()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()
        pass
