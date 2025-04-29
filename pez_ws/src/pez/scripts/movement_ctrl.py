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
        self.blend = rospy.get_param('~vel_blend', 0.2)

        # Initial control values
        self.tail_pwm = PWMValue(tail_pwm[0], tail_pwm[1], tail_pwm[2], tail_pwm[3])       # [176.1, 303, 434]PWM [30,90,150]º      Ch16
        self.tail_pwm_offset = 0
        self.lat0_pwm = PWMValue(fin_pwm[0], fin_pwm[1], fin_pwm[2], fin_pwm[3])          # [153.5, 231, 307]PWM [45,113,180]º     Ch14
        self.lat1_pwm = PWMValue(fin_pwm[0], fin_pwm[1], fin_pwm[2], fin_pwm[3])          # [153.5, 231, 307]PWM [45,113,180]º     Ch15
        self.camera_angle = PWMValue(cam_pwm[0], cam_pwm[1], cam_pwm[2], cam_pwm[3])      # [XX, 315, XX]PWM [100,180,260]º        Ch11
        self.timer_movement = PWMValue(time_pwm[0], time_pwm[1], time_pwm[2])

        # Note that during execution tail_pwm.current is an offset for tail_pwm.default
        # In lat and camera cases default and current are absolute pwm values

        self.setup_pwm()
        self.sync_flag = True # Flag sync. While computing input message wait for sending output movement

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
            """
            Option 1: While computing values do not send output
            if self.sync_flag == False: 
                continue
            tail, fin0, fin1, cam, timer = self.tail_pwm.current, self.lat0_pwm.current,  self.lat1_pwm.current, self.camera_angle.current, self.timer_movement.current
            """
            #Option 2: While computing values do not modify output values but maintain previous send
            if self.sync_flag == True:
                offset_tail, fin0, fin1, cam, timer = self.tail_pwm.current, self.lat0_pwm.current,  self.lat1_pwm.current, self.camera_angle.current, self.timer_movement.current

            navigator.set_pwm_channels_values([PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15, PwmChannel.Ch11 ], 
                                           [self.tail_pwm.default + offset_tail + self.tail_pwm_offset, fin0, fin1, cam])
            time.sleep(timer)
            navigator.set_pwm_channels_values([PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15, PwmChannel.Ch11 ], 
                                           [self.tail_pwm.default - offset_tail + self.tail_pwm_offset, fin0, fin1, cam])
            time.sleep(timer)
        rospy.loginfo("[PezController] Movement thread stopped.")

    def controller_callback(self, msg):
        
        self.sync_flag = False
        # Map Twist linear velocities to PWM signals
        x = msg.linear.x  # Forward
        y = msg.linear.y  # Left/right
        z = msg.linear.z  # Up/down

        self.tail_pwm.current, timer_offset = self.set_vel(x)
        self.lat0_pwm.current, self.lat1_pwm.current = self.up_down(z)
        self.tail_pwm_offset, timer_offset, self.lat0_pwm.current, self.lat1_pwm.current = self.left_right(y)
        

        # clamp
        self.timer_movement.current = self.minmax(self.timer_movement.current + timer_offset, self.timer_movement.min, self.timer_movement.max)

        if self.minmax(self.tail_pwm.default + self.tail_pwm.current + self.tail_pwm_offset, self.tail_pwm.min, self.tail_pwm.max) >= self.tail_pwm.max:
            self.tail_pwm.current = self.tail_pwm.max - self.tail_pwm.default - self.tail_pwm_offset
        elif self.minmax(self.tail_pwm.default - self.tail_pwm.current + self.tail_pwm_offset, self.tail_pwm.min, self.tail_pwm.max) <= self.tail_pwm.min:
            self.tail_pwm.current = -self.tail_pwm.min + self.tail_pwm.default + self.tail_pwm_offset
        
        self.sync_flag = True
        

    def minmax(self, value, min_value, max_value):
        """
        Return value clamped. The minimum between top/current value and the maximum between bottom/current value
        """
        return max(min_value, min(max_value, value))
    
    def pwm_to_degrees(self, pwm_val, pwm_min, pwm_max, deg_min, deg_max):
        return deg_min + (pwm_val - pwm_min) * (deg_max - deg_min) / (pwm_max - pwm_min)


    def up_down(self, z):
        if z == 0.0:
            return self.lat0_pwm.current, self.lat1_pwm.current# Holding current position
        
        # Apply sensitivity scale (sense) to input (z) and added to current value
        lat0_pwm = self.lat0_pwm.sense * z + self.lat0_pwm.current
        lat1_pwm = self.lat1_pwm.sense * z + self.lat1_pwm.current


        #Clamp values
        lat0_pwm = self.minmax(lat0_pwm, self.lat0_pwm.min, self.lat0_pwm.max)
        lat1_pwm = self.minmax(lat1_pwm, self.lat1_pwm.min, self.lat1_pwm.max)
        
        # Display Fin position
        deg = self.pwm_to_degrees(self.lat0_pwm.current, self.lat0_pwm.min, self.lat0_pwm.max, 45, 180)
        rospy.loginfo(f"[PezController] Upping/Downing: PWM={self.lat0_pwm.current:.1f} → {deg:.1f}º")

        return lat0_pwm, lat1_pwm


    def set_vel(self, x):
        tail_pwm_amplitude = 0
        timer_offset = 0

        if x == 0.0:
            return tail_pwm_amplitude, timer_offset

        # Compute Target PWM and Timer by mapping input x

        s = (x + 1.0) / 2.0                                     # Normalize x [0,1]
        dt = self.timer_movement.max - self.timer_movement.min  # Range of T (deltaT)
        target_T = self.timer_movement.max - s * dt             # Target T = max - range * x_norm | x=1 T=min | x=-1 T=max

        Amax = (self.tail_pwm.max - self.tail_pwm.min) / 2.0    # Amplitud range | Maximum amplitud from center
        k = 1.0 - abs(x)                                        # x=+-1 k=0 A=0 (will be clamped before that case)
        target_pwm = Amax * k                                   # Amplitud inversed proportional to velocity desired

        # move a fraction toward it each call
        blend = self.blend  # % of the gap per update
        timer_offset = (target_T - self.timer_movement.current) * blend
        tail_pwm_amplitude = self.tail_pwm.current + (target_pwm - self.tail_pwm.current) * blend

        rospy.loginfo("[PezController] Velocitying.")
        return tail_pwm_amplitude, timer_offset

    def left_right(self, y):
        tail_pwm_offset = 0
        timer_offset = 0
        lat0_pwm = self.lat0_pwm.current
        lat1_pwm = self.lat1_pwm.current

        if y == 0:
            
            rospy.loginfo("[PezController] Lefting/Righting.")
            return tail_pwm_offset, timer_offset, lat0_pwm, lat1_pwm
            # 1) Smoothly move each fin to default ± max_deflection*y
        max_deflect = 80  # degrees or PWM units from center
        blend_fast  = 0.5  # how fast fins move toward target (0<blend≤1)

        target0 = self.lat0_pwm.default +  y * max_deflect
        target1 = self.lat1_pwm.default + -y * max_deflect

        # blend current → target
        self.lat0_pwm.current += (target0 - self.lat0_pwm.current) * blend_fast
        self.lat1_pwm.current += (target1 - self.lat1_pwm.current) * blend_fast

        # clamp
        self.lat0_pwm.current = self.minmax(self.lat0_pwm.current,
                                            self.lat0_pwm.min,
                                            self.lat0_pwm.max)
        self.lat1_pwm.current = self.minmax(self.lat1_pwm.current,
                                            self.lat1_pwm.min,
                                            self.lat1_pwm.max)


        max_tail_off = 75  # max extra swing from center
        tail_pwm_offset = y * max_tail_off

        rospy.loginfo("[PezController] Lefting/Righting.")
        return tail_pwm_offset, timer_offset, lat0_pwm, lat1_pwm


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
