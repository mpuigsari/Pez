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

        # Parameters declared [default, min, max]
        tail_pwm = [rospy.get_param('~tail_default', 303), rospy.get_param('~tail_min', 303), rospy.get_param('~tail_max', 303), rospy.get_param('~tail_sens', 25)]
        fin_pwm = [rospy.get_param('~fin_default', 231), rospy.get_param('~fin_min', 153.5), rospy.get_param('~fin_max', 307), rospy.get_param('~tail_sens', 25)]
        # fin_pwm = [rospy.get_param('~fin_default', 231), rospy.get_param('~fin_min', 153.5), rospy.get_param('~fin_max', 307), rospy.get_param('~tail_sens', 25)]
        cam_pwm = [rospy.get_param('~cam_default', 315), rospy.get_param('~cam_min', None), rospy.get_param('~cam_max', None), rospy.get_param('~cam_sens', 25)]
        time_pwm = [rospy.get_param('~time_default', 0.3), rospy.get_param('~time_min', 0.1), rospy.get_param('~time_max', 0.5)]


        # Initial control values
        self.tail_pwm = PWMValue(tail_pwm[0], tail_pwm[1], tail_pwm[2], tail_pwm[3])       # [176.1, 303, 434]PWM [30,90,150]º      Ch16
        self.lat0_pwm = PWMValue(fin_pwm[0], fin_pwm[1], fin_pwm[2], fin_pwm[3])          # [153.5, 231, 307]PWM [45,113,180]º     Ch14
        self.lat1_pwm = PWMValue(fin_pwm[0], fin_pwm[1], fin_pwm[2], fin_pwm[3])          # [153.5, 231, 307]PWM [45,113,180]º     Ch15
        self.camera_angle = PWMValue(cam_pwm[0], cam_pwm[1], cam_pwm[2], cam_pwm[3])      # [XX, 315, XX]PWM [100,180,260]º        Ch11
        self.timer_movement = PWMValue(time_pwm[0], time_pwm[1], time_pwm[2])

        self.setup_pwm()

        # Start Movement Control Loop ¿Service?
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
        
    def minmax(self, value, min_value, max_value):
        """
        Return value clamped. The minimum between top/current value and the maximum between bottom/current value
        """
        return max(min_value, min(max_value, value))
    
    def pwm_to_degrees(self, pwm_val, pwm_min, pwm_max, deg_min, deg_max):
        return deg_min + (pwm_val - pwm_min) * (deg_max - deg_min) / (pwm_max - pwm_min)


    def up_down(self, z):
        if z == 0.0:
            return # No offset
        
        # Apply sensitivity scale (sense) to input (z) and added to current value
        offset_0 = self.lat0_pwm.sense * z + self.lat0_pwm.current
        offset_1 = self.lat1_pwm.sense * z + self.lat1_pwm.current


        #Clamp values and set to current
        self.lat0_pwm.current = self.minmax(offset_0, self.lat0_pwm.min, self.lat0_pwm.max)
        self.lat1_pwm.current = self.minmax(offset_1, self.lat1_pwm.min, self.lat1_pwm.max)
        
        # Display Fin position
        deg = self.pwm_to_degrees(self.lat0_pwm.current, self.lat0_pwm.min, self.lat0_pwm.max, 45, 180)
        rospy.loginfo(f"[PezController] Upping/Downing: PWM={self.lat0_pwm.current:.1f} → {deg:.1f}º")


    def set_vel(self, x):
        rospy.loginfo("[PezController] Velocitying.")


    def left_right(self, y):
        if y == 0:
            
            rospy.loginfo("[PezController] Lefting/Righting.")
            return
        rospy.loginfo("[PezController] Lefting/Righting.")


    def shutdown(self):
        self.thread.join()

    def run(self):
        rospy.spin()


class PWMValue:
    def __init__(self, default, min, max, sense):
        self.default = default
        self.current = default
        self.min = min
        self.max = max
        self.sense = sense


if __name__ == '__main__':
    try:
        controller = PezController()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()
        pass
