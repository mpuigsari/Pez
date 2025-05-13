#!/usr/bin/env python3

import time
import threading
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import bluerobotics_navigator.bluerobotics_navigator as navigator


class PWMValue:
    def __init__(self, default, min_, max_, deflect_=0, blend_=0):
        self.default = default
        self.current = default
        self.min     = min_
        self.max     = max_
        self.deflect = deflect_
        self.blend   = blend_

    def clamp(self, value):
        return min(max(value, self.min), self.max)

    def clamp_deflect(self, v):
        return min(max(v, -self.deflect), self.deflect)

class PwmChannel:
    Ch9 = 9-1
    Ch11 = 11-1
    Ch14 = 14-1
    Ch15 = 15-1
    Ch16 = 16-1

class PezController(Node):
    def __init__(self):
        super().__init__('pez_controller')

        # 1) Initialize hardware
        navigator.init()
        navigator.set_pwm_enable(True)
        navigator.set_pwm_freq_hz(50)

        # 2) Declare & read scalar parameters

        #Cam PWM limits
        self.declare_parameter('cam_pwm_default', 315)
        self.declare_parameter('cam_pwm_min',     215)
        self.declare_parameter('cam_pwm_max',     415)
        cam_def = self.get_parameter('cam_pwm_default').value
        cam_min = self.get_parameter('cam_pwm_min').value
        cam_max = self.get_parameter('cam_pwm_max').value
        self.cam_pwm = PWMValue(default=cam_def, min_=cam_min, max_=cam_max)

        # Tail PWM limits
        self.declare_parameter('tail_pwm_default', 303)
        self.declare_parameter('tail_pwm_min',     176)
        self.declare_parameter('tail_pwm_max',     434)
        tail_def = self.get_parameter('tail_pwm_default').value
        tail_min = self.get_parameter('tail_pwm_min').value
        tail_max = self.get_parameter('tail_pwm_max').value
        self.tail_pwm = PWMValue(tail_def, tail_min, tail_max)

        # Compute offset and amplitude limits
        tail_off_min = tail_min - tail_def
        tail_off_max = tail_max - tail_def
        Amax = min(abs(tail_off_min), abs(tail_off_max))
        self.tail_offset    = PWMValue(0, tail_off_min, tail_off_max)
        self.tail_amplitude = PWMValue(0, 0, Amax)

        # Fins PWM limits
        self.declare_parameter('fins_pwm_default', 231)
        self.declare_parameter('fins_pwm_min',     153.5)
        self.declare_parameter('fins_pwm_max',     307)
        fins_def = self.get_parameter('fins_pwm_default').value
        fins_min = self.get_parameter('fins_pwm_min').value
        fins_max = self.get_parameter('fins_pwm_max').value
        self.left_fin_pwm  = PWMValue(fins_def, fins_min, fins_max)
        self.right_fin_pwm = PWMValue(fins_def, fins_min, fins_max)

        # Timer movement limits
        self.declare_parameter('timer_movement_default', 0.6)
        self.declare_parameter('timer_movement_min',     0.3)
        self.declare_parameter('timer_movement_max',     1.5)
        tdef = self.get_parameter('timer_movement_default').value
        tmin = self.get_parameter('timer_movement_min').value
        tmax = self.get_parameter('timer_movement_max').value
        self.timer_movement = PWMValue(tdef, tmin, tmax)

        # Tuning scalars
        self.declare_parameter('tail_max_deflect', 125)
        self.declare_parameter('tail_blend',       0.2)
        self.declare_parameter('fin_max_deflect',  75)
        self.declare_parameter('fin_blend',        0.5)
        self.declare_parameter('vel_blend',        0.2)
        tail_deflect = self.get_parameter('tail_max_deflect').value
        tail_blend   = self.get_parameter('tail_blend').value
        fin_deflect  = self.get_parameter('fin_max_deflect').value
        fin_blend    = self.get_parameter('fin_blend').value
        self.vel_blend = self.get_parameter('vel_blend').value

        self.tail_pwm.deflect   = tail_deflect
        self.tail_pwm.blend     = tail_blend
        self.left_fin_pwm.deflect  = fin_deflect
        self.right_fin_pwm.deflect = fin_deflect
        self.left_fin_pwm.blend    = fin_blend
        self.right_fin_pwm.blend   = fin_blend

        # Helpers for fins control
        self.fin_dive_ref = PWMValue(0, -fin_deflect, fin_deflect,
                                     deflect_=fin_deflect, blend_=fin_blend)
        self.fin_turn_off = PWMValue(0, -fin_deflect, fin_deflect,
                                     deflect_=fin_deflect, blend_=fin_blend)
        
        # 2a) **NEW** tail‐control parameters/state
        self.speed_min, self.speed_max = 0.0, 1.5
        self.freq_min,  self.freq_max  = 0.4, 2.5
        self.amp_min,   self.amp_max   = 20.0, 60.0
        self.k_freq,    self.k_amp     = 5.0, 5.0
        self.slew_f,    self.slew_a    = 1.0, 50.0
        self.alpha_f,   self.alpha_a   = 0.1, 0.1
        self.safe_f,    self.safe_a    = 0.2, 5.0

        self.desired_speed = 0.0
        self.current_freq  = self.freq_min
        self.current_amp   = self.amp_min

        # 3) ROS interfaces
        ns = self.get_namespace()  # e.g. '/pez/'

        # Subscribe to /pez/cmd_vel
        self.create_subscription(
            Twist,
            f'{ns}/cmd_vel',
            self.controller_callback,
            10
        )
        # Subscribe to /pez/camera_control
        self.create_subscription(
            Float64,
            f'{ns}/camera_control',
            self.cam_controller_callback,
            10
        )

        # Teleoperation services
        self.create_service(Trigger, 'teleoperation/start_swim',      self.handle_start)
        self.create_service(Trigger, 'teleoperation/stop_swim',       self.handle_stop)
        self.create_service(Trigger, 'teleoperation/toggle_magnet',   self.handle_toggle_magnet)

        # 4) Internal state & thread
        self.sync_flag  = True
        self.running    = False
        self.magnet_on  = False
        self.exit_event = threading.Event()

        self.thread_tail     = threading.Thread(target=self.send_movement_tail)
        self.thread_tail.start()

        self.thread_fins     = threading.Thread(target=self.send_movement_fins)
        self.thread_fins.start()

        self.get_logger().info('[PezController] Initialized.')

    def handle_start(self, request, response):
        self.tail_amplitude.current = self.tail_amplitude.default
        self.tail_offset.current    = self.tail_offset.default
        self.fin_dive_ref.current   = 0
        self.fin_turn_off.current   = 0
        self.cam_pwm.current = self.cam_pwm.default
        self.timer_movement.current = self.timer_movement.default

        self.running = True
        self.get_logger().info('[PezController] start_swim → running')
        response.success = True
        response.message = 'started'
        return response

    def handle_stop(self, request, response):
        self.tail_amplitude.current = self.tail_amplitude.default
        self.tail_offset.current    = self.tail_offset.default
        self.fin_dive_ref.current   = 0
        self.fin_turn_off.current   = 0
        self.cam_pwm.current = self.cam_pwm.default

        self.running = False
        time.sleep(self.timer_movement.current)
        self.timer_movement.current = self.timer_movement.default

        self.get_logger().info('[PezController] stop_swim → stopped')
        navigator.set_pwm_channels_values(
            [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
            [self.tail_pwm.default,
             self.left_fin_pwm.default,
             self.right_fin_pwm.default]
        )
        response.success = True
        response.message = 'stopped'
        return response

    def handle_toggle_magnet(self, request, response):
        self.magnet_on = not self.magnet_on
        state = 'ON' if self.magnet_on else 'OFF'
        value = 1.0 if self.magnet_on else 0.0
        navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch9, value)
        self.get_logger().info(f'[PezController] Magnet → {state}')
        response.success = True
        response.message = f'magnet {state.lower()}'
        return response

    def send_movement_fins(self):
        rate = 50.0
        while not self.exit_event.is_set():
            if not self.running:
                time.sleep(1.0 / rate)
                continue

            if self.sync_flag:
                lf     = int(self.left_fin_pwm.current)
                rf     = int(self.right_fin_pwm.current)
                t      = self.timer_movement.current

                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                    [lf, rf]
                )
                time.sleep(t)
                if self.exit_event.is_set():
                    break
                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch14, PwmChannel.Ch15],
                    [lf, rf]
                )
                time.sleep(1.0/10)

    def send_movement_tail(self):
        dt = 0.05
        while not self.exit_event.is_set():
            if not self.running:
                navigator.set_pwm_enable(False)
                time.sleep(dt)
                continue

            navigator.set_pwm_enable(True)

            # 1) normalize speed
            vnorm = (self.desired_speed - self.speed_min) / (self.speed_max - self.speed_min)
            vnorm = max(0.0, min(1.0, vnorm))

            # 2) target freq & amp
            f_tgt = self._sigma(vnorm, self.freq_min, self.freq_max, self.k_freq)
            a_tgt = self._sigma(vnorm, self.amp_min, self.amp_max, self.k_amp)

            # 3) slew + LPF, frequency
            df = max(-self.slew_f*dt, min(self.slew_f*dt, f_tgt - self.current_freq))
            f_tmp = self.current_freq + df
            self.current_freq = f_tmp + self.alpha_f*(f_tgt - f_tmp)

            # 4) slew + LPF, amplitude
            da = max(-self.slew_a*dt, min(self.slew_a*dt, a_tgt - self.current_amp))
            a_tmp = self.current_amp + da
            self.current_amp = a_tmp + self.alpha_a*(a_tgt - a_tmp)

            # 5) safety check
            if self.current_freq < self.safe_f or self.current_amp < self.safe_a:
                navigator.set_pwm_enable(False)
                time.sleep(dt)
                continue

            # 6) oscillate
            period = 1.0 / self.current_freq
            center = self.tail_pwm.default + self.tail_offset.current
            L = int(center - self.current_amp)
            R = int(center + self.current_amp)

            navigator.set_pwm_channels_values([PwmChannel.Ch16], [L])
            time.sleep(period/2)
            navigator.set_pwm_channels_values([PwmChannel.Ch16], [R])
            time.sleep(period/2)


    def controller_callback(self, msg: Twist):
        self.sync_flag = False
        x, y, z = msg.linear.x, msg.linear.y, msg.linear.z

        # → Cruise‐control for tail speed
        if x != 0.0:
            v = self.desired_speed + x
            self.desired_speed = max(self.speed_min,
                                     min(self.speed_max, v))

        # ← keep your old tail‐offset and fins logic
        target = self.tail_offset.default + y * self.tail_pwm.deflect
        self.tail_offset.current += (target - self.tail_offset.current) * self.tail_pwm.blend
        self.tail_offset.current = self.tail_offset.clamp(self.tail_offset.current)

        def blend(comp, val):
            tgt = val * comp.deflect
            comp.current = comp.clamp_deflect(comp.current + (tgt - comp.current) * comp.blend)

        if z != 0.0:
            blend(self.fin_dive_ref, z)
        blend(self.fin_turn_off, y)

        for pwm, sign in ((self.left_fin_pwm, +1), (self.right_fin_pwm, -1)):
            tgt = pwm.default + self.fin_dive_ref.current + sign * self.fin_turn_off.current
            pwm.current = pwm.clamp(pwm.current + (tgt - pwm.current) * pwm.blend)

        self.sync_flag = True

    def cam_controller_callback(self, msg: Float64):
        cam_offset = msg.data
        self.cam_pwm.current = self.cam_pwm.clamp(self.cam_pwm.current + cam_offset)
        cam = int(self.cam_pwm.current)
        navigator.set_pwm_channels_value([PwmChannel.Ch11], cam)
        self.get_logger().info(f'[PezController] Camera → {cam}')


    def _sigma(self, x, lo, hi, k):
        e = math.exp(-k * (x - 0.5))
        return lo + (hi - lo) / (1 + e)

    def destroy_node(self):
        self.exit_event.set()
        self.thread_tail.join(timeout=1.0)
        self.thread_fins.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PezController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
