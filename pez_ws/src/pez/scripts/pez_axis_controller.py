#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import time
import threading


class PWMValue:
    def __init__(self, default, min_, max_):
        self.default = default
        self.current = default
        self.min = min_
        self.max = max_

    def clamp(self, value):
        return min(max(value, self.min), self.max)


class AxisController:
    def __init__(self, pwm, max_deflect, blend):
        self.pwm = pwm
        self.max_deflect = max_deflect
        self.blend = blend

    def update(self, input_val):
        target = self.pwm.default + input_val * self.max_deflect
        self.pwm.current += (target - self.pwm.current) * self.blend
        self.pwm.current = self.pwm.clamp(self.pwm.current)


class PezController:
    def __init__(self):
        rospy.init_node('move_controller', anonymous=True)

        navigator.init()
        navigator.set_pwm_enable(True)
        navigator.set_pwm_freq_hz(50)

        namespace = rospy.get_namespace()
        rospy.Subscriber(namespace + "/cmd_vel", Twist, self.controller_callback)

        # PWM setup via ROS parameters
        tail_params = rospy.get_param('~tail_pwm', {'default': 303, 'min': 176, 'max': 434})
        self.tail_pwm = PWMValue(tail_params['default'], tail_params['min'], tail_params['max'])

        fin_params = rospy.get_param('~fins_pwm', {'default': 231, 'min': 153.5, 'max': 307})
        self.left_fin_pwm = PWMValue(fin_params['default'], fin_params['min'], fin_params['max'])
        self.right_fin_pwm = PWMValue(fin_params['default'], fin_params['min'], fin_params['max'])

        timer_params = rospy.get_param('~timer_movement', {'default': 0.3, 'min': 0.1, 'max': 0.5})
        self.timer_movement = PWMValue(timer_params['default'], timer_params['min'], timer_params['max'])

        # Axis Controllers via simplified ROS parameters
        tail_deflect = rospy.get_param('~tail_max_deflect', 100)
        tail_blend   = rospy.get_param('~tail_blend', 0.2)
        fin_deflect  = rospy.get_param('~fin_max_deflect', 60)
        fin_blend    = rospy.get_param('~fin_blend', 0.5)

        self.tail_axis      = AxisController(self.tail_pwm,    max_deflect=tail_deflect, blend=tail_blend)
        self.left_fin_axis  = AxisController(self.left_fin_pwm, max_deflect=fin_deflect,  blend=fin_blend)
        self.right_fin_axis = AxisController(self.right_fin_pwm,max_deflect=fin_deflect,  blend=fin_blend)
        
        # Velocity blend (used in update_velocity)
        self.vel_blend = rospy.get_param('~vel_blend', 0.2)

        # Thread sync flag
        self.sync_flag = True

        # Start the thread
        self.thread = threading.Thread(target=self.send_movement)
        self.thread.start()

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("[PezController] Initialized.")

    def send_movement(self):
        while not rospy.is_shutdown():
            if self.sync_flag:
                tail = self.tail_pwm.current
                left_fin = self.left_fin_pwm.current
                right_fin = self.right_fin_pwm.current
                timer = self.timer_movement.current

                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                    [self.tail_pwm.default + tail, left_fin, right_fin])
                time.sleep(timer)

                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                    [self.tail_pwm.default - tail, left_fin, right_fin])
                time.sleep(timer)

    def controller_callback(self, msg):
        self.sync_flag = False

        self.update_velocity(msg.linear.x)
        self.update_diving(msg.linear.z)
        self.update_turning(msg.linear.y)

        self.sync_flag = True

    def update_velocity(self, x):
        if x == 0.0:
            return

        s = (x + 1.0) / 2.0
        dt = self.timer_movement.max - self.timer_movement.min
        target_timer = self.timer_movement.max - s * dt

        Amax = (self.tail_pwm.max - self.tail_pwm.min) / 2.0
        target_amplitude = Amax * (1.0 - abs(x))

        blend = self.vel_blend

        self.timer_movement.current += (target_timer - self.timer_movement.current) * blend
        self.timer_movement.current = self.timer_movement.clamp(self.timer_movement.current)

        self.tail_pwm.current += (target_amplitude - self.tail_pwm.current) * blend
        self.tail_pwm.current = self.tail_pwm.clamp(self.tail_pwm.current)

    def update_diving(self, z):
        if z == 0.0:
            return

        self.left_fin_axis.update(z)
        self.right_fin_axis.update(z)

    def update_turning(self, y):
        self.left_fin_axis.update(y)
        self.right_fin_axis.update(-y)
        self.tail_axis.update(y)

    def shutdown(self):
        rospy.loginfo("[PezController] Shutting down.")
        self.thread.join()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    controller = PezController()
    controller.run()
