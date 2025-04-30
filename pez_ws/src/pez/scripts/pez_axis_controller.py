#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import time
import threading
from std_srvs.srv import Trigger, TriggerResponse



class PWMValue:
    def __init__(self, default, min_, max_):
        self.default = default
        self.current = default
        self.min = min_
        self.max = max_
        self.deflect = 0
        self.blend = 0

    def clamp(self, value):
        return min(max(value, self.min), self.max)


class PezController:
    def __init__(self):
        rospy.init_node('move_controller', anonymous=True)

        navigator.init()
        navigator.set_pwm_enable(True)
        navigator.set_pwm_freq_hz(50)

        namespace = rospy.get_namespace()
        rospy.Subscriber(namespace + "cmd_vel", Twist, self.controller_callback)

        # PWM setup via ROS parameters
        
        #Tail
        tail_params = rospy.get_param('~tail_pwm', {'default': 303, 'min': 176, 'max': 434})
        self.tail_pwm = PWMValue(tail_params['default'], tail_params['min'], tail_params['max'])
        # limits tail (absolute PWM)
        tail_offset_min = self.tail_pwm.min - self.tail_pwm.default
        tail_offset_max = self.tail_pwm.max - self.tail_pwm.default
        # compute how large the half-swing (amplitude) can get
        Amax = min(abs(tail_offset_min), abs(tail_offset_max))
        # Tail offset and amplitude values
        self.tail_offset = PWMValue(0, tail_offset_min, tail_offset_max)  # from turning (y input)
        self.tail_amplitude = PWMValue(0, int(Amax*0.1), Amax) # [10%,100%] Amplitude
        
        #Fins
        fin_params = rospy.get_param('~fins_pwm', {'default': 231, 'min': 153.5, 'max': 307})
        self.left_fin_pwm = PWMValue(fin_params['default'], fin_params['min'], fin_params['max'])
        self.right_fin_pwm = PWMValue(fin_params['default'], fin_params['min'], fin_params['max'])

        timer_params = rospy.get_param('~timer_movement', {'default': 0.6, 'min': 0.3, 'max': 1.5})
        self.timer_movement = PWMValue(timer_params['default'], timer_params['min'], timer_params['max'])

        self.tail_pwm.deflect = rospy.get_param('~tail_max_deflect', 125)
        self.tail_pwm.blend   = rospy.get_param('~tail_blend', 0.2)
        self.left_fin_pwm.deflect = self.right_fin_pwm.deflect = rospy.get_param('~fin_max_deflect', 75)
        self.left_fin_pwm.blend = self.right_fin_pwm.blend = rospy.get_param('~fin_blend', 0.5)
        self.vel_blend = rospy.get_param('~vel_blend', 0.2)



        # Thread sync flag
        self.sync_flag = True

        # start/stop gating
        self.running = False
        self._srv_start = rospy.Service('teleoperation/start_swim', Trigger, self.handle_start)
        self._srv_stop  = rospy.Service('teleoperation/stop_swim',  Trigger, self.handle_stop)
        self.magnet_on = False
        self._srv_toggle_magnet = rospy.Service("teleoperation/toggle_magnet", Trigger, self.handle_toggle_magnet)

        # Start the thread
        self.exit_event = threading.Event()
        self.thread = threading.Thread(target=self.send_movement)
        self.thread.start()

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("[PezController] Initialized.")
        

    def handle_start(self, req):
        self.tail_amplitude.current = self.tail_amplitude.default
        self.tail_offset.current = self.tail_offset.default
        self.right_fin_pwm.current = self.right_fin_pwm.default
        self.left_fin_pwm.current = self.right_fin_pwm.default
        
        self.running = True
        rospy.loginfo("[PezController] Received start_swim → running")
        return TriggerResponse(success=True, message="started")

    def handle_stop(self, req):
        self.tail_amplitude.current = self.tail_amplitude.default
        self.tail_offset.current = self.tail_offset.default
        self.right_fin_pwm.current = self.right_fin_pwm.default
        self.left_fin_pwm.current = self.right_fin_pwm.default

        self.running = False
        rospy.loginfo("[PezController] Received stop_swim → stopped")
        # send a single center command so we don’t leave it oscillating
        navigator.set_pwm_channels_values(
            [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
            [self.tail_pwm.default,
             self.left_fin_pwm.default,
             self.right_fin_pwm.default])
        return TriggerResponse(success=True, message="stopped")
    
    def handle_toggle_magnet(self, req):
        # Flip the state
        self.magnet_on = not self.magnet_on
        state_str = "ON" if self.magnet_on else "OFF"

        # Log the new state
        rospy.loginfo(f"[PezController] Magnet toggled → {state_str}")

        # TODO: actually drive the PWM channel for the electromagnet.
        return TriggerResponse(success=True, message=f"magnet turned {state_str.lower()}"
        )

    def send_movement(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and not self.exit_event.is_set():
            if not self.running:
                rate.sleep()
                continue

            if self.sync_flag:
                left_fin = self.left_fin_pwm.current
                right_fin = self.right_fin_pwm.current
                timer = self.timer_movement.current

                # center swings around default + offset
                tail_center = self.tail_pwm.default + self.tail_offset.current
                tail_amp    = self.tail_amplitude.current

                tail_high = self.tail_pwm.clamp(tail_center + tail_amp)
                tail_low  = self.tail_pwm.clamp(tail_center - tail_amp)

                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                    [tail_high, left_fin, right_fin])
                time.sleep(timer)
                if not self.exit_event.is_set():
                    navigator.set_pwm_channels_values(
                        [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                        [tail_low, left_fin, right_fin])
                    time.sleep(timer)

    def controller_callback(self, msg):
        self.sync_flag = False
        x,y,z = msg.linear.x, msg.linear.y, msg.linear.z

        # Tail
        if x != 0.0:
            self.update_x(x)
        # 2) turning → offset = y
        target = self.tail_offset.default + y * self.tail_pwm.deflect
        self.tail_offset.current += (target - self.tail_offset.current) * self.tail_pwm.blend
        self.tail_offset.current  = self.tail_offset.clamp(self.tail_offset.current)
        
        # Fins:
        #   - dive: hold last position if z==0
        #   - turn: auto-center when y==0
        self.update_fins(y,z)


        self.sync_flag = True
    
    def update_x(self,x):
        # normalize & compute both targets
            norm   = (x + 1.0) / 2.0                     # → [0..1]
            amp_t  = (1.0 - abs(x)) * self.tail_amplitude.max
            tim_t  = self.timer_movement.max \
                     - norm * (self.timer_movement.max - self.timer_movement.min)
            # blend BOTH amplitude & timer by the same vel_blend factor
            for pwm, tgt in ((self.tail_amplitude, amp_t),(self.timer_movement, tim_t)):
                pwm.current = pwm.clamp(
                    pwm.current + (tgt - pwm.current) * self.vel_blend)
                
    def update_fins(self, y, z):
        for pwm, sign_y in [(self.left_fin_pwm, +1), (self.right_fin_pwm, -1)]:
            dive_component = 0.0
            if z != 0.0:
                # Apply new dive input
                dive_component = z * pwm.deflect
            else:
                # Hold current dive deflection (remove last turn contribution)
                dive_component = pwm.current - pwm.default - sign_y * y * pwm.deflect

            # Turn contribution (always computed)
            turn_component = sign_y * y * pwm.deflect

            # Final target
            target = pwm.default + dive_component + turn_component

            # Blend and clamp
            pwm.current += (target - pwm.current) * pwm.blend
            pwm.current = pwm.clamp(pwm.current)


    def shutdown(self):
        rospy.loginfo("[PezController] Shutting down.")
        # signal the thread to exit immediately
        self.exit_event.set()
        # give it a moment to wake up
        self.thread.join(timeout=1.0)
        rospy.loginfo("[PezController] Shutdown complete.")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    controller = PezController()
    controller.run()
