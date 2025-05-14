#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import math
import time

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

        # 1) Test‐mode and navigator selection
        self.declare_parameter('test_flag', False)
        if self.get_parameter('test_flag').value:
            from pez_core.aux import FakeNav
            self.nav = FakeNav(self)
        else:
            import bluerobotics_navigator.bluerobotics_navigator as navigator
            self.nav = navigator

        # 2) Hardware setup
        self.nav.init()
        self.nav.set_pwm_enable(True)
        self.nav.set_pwm_freq_hz(50)

        # 3) Camera PWM limits
        self.declare_parameter('cam_pwm_default', 315)
        self.declare_parameter('cam_pwm_min',     215)
        self.declare_parameter('cam_pwm_max',     415)
        cam_def = self.get_parameter('cam_pwm_default').value
        cam_min = self.get_parameter('cam_pwm_min').value
        cam_max = self.get_parameter('cam_pwm_max').value
        self.cam_pwm = PWMValue(cam_def, cam_min, cam_max)

        # 4) Tail PWM limits & derived values
        self.declare_parameter('tail_pwm_default', 303)
        self.declare_parameter('tail_pwm_min',     176)
        self.declare_parameter('tail_pwm_max',     434)
        tail_def = self.get_parameter('tail_pwm_default').value
        tail_min = self.get_parameter('tail_pwm_min').value
        tail_max = self.get_parameter('tail_pwm_max').value
        self.tail_pwm = PWMValue(tail_def, tail_min, tail_max)

        #   Compute zero‐center offset limits and amplitude ceiling
        tail_off_min = tail_min - tail_def
        tail_off_max = tail_max - tail_def
        Amax        = min(abs(tail_off_min), abs(tail_off_max))
        self.tail_offset    = PWMValue(0, tail_off_min, tail_off_max)
        self.tail_amplitude = PWMValue(0, 0, Amax)

        # 5) Fins PWM limits
        self.declare_parameter('fins_pwm_default', 231)
        self.declare_parameter('fins_pwm_min',     153.5)
        self.declare_parameter('fins_pwm_max',     307)
        fins_def = self.get_parameter('fins_pwm_default').value
        fins_min = self.get_parameter('fins_pwm_min').value
        fins_max = self.get_parameter('fins_pwm_max').value
        self.left_fin_pwm  = PWMValue(fins_def, fins_min, fins_max)
        self.right_fin_pwm = PWMValue(fins_def, fins_min, fins_max)

        # 6) Fin tuning (deflection + blend)
        self.declare_parameter('fin_max_deflect', 75)
        self.declare_parameter('fin_blend',       0.5)
        fin_deflect = self.get_parameter('fin_max_deflect').value
        fin_blend   = self.get_parameter('fin_blend').value

        self.left_fin_pwm.deflect  = fin_deflect
        self.right_fin_pwm.deflect = fin_deflect
        self.left_fin_pwm.blend    = fin_blend
        self.right_fin_pwm.blend   = fin_blend
        # Helpers for fins control
        self.fin_dive_ref = PWMValue(0, -fin_deflect, fin_deflect,
                                     deflect_=fin_deflect, blend_=fin_blend)
        self.fin_turn_off = PWMValue(0, -fin_deflect, fin_deflect,
                                     deflect_=fin_deflect, blend_=fin_blend)

        # 7) Steering offset limit (tail deflect used for O_max)
        self.declare_parameter('tail_max_deflect', tail_off_max)  # you may set a separate O_max if desired
        tail_deflect = self.get_parameter('tail_max_deflect').value
        self.O_max = tail_deflect

        # 8) Tail‐dynamics parameters (accel, drag, freq & amp envelopes)
        self.declare_parameter('k_accel', 0.5)    # speed ↑ when x>0
        self.declare_parameter('k_brake', -0.7)   # speed ↓ when x<0
        self.declare_parameter('k_drag',  0.3)    # coast slowdown when x=0

        self.declare_parameter('f_min',  0.5)     # tail‐beat min frequency (Hz)
        self.declare_parameter('f_max',  2.0)     # tail‐beat max frequency (Hz)
        self.declare_parameter('A_min', 10.0)     # tail amplitude at top speed
        self.declare_parameter('A_max', 75.0)     # tail amplitude at low speed

        self.declare_parameter('rate_hz', 50.0)   # update rate for send_movement_tail

        self.k_accel = self.get_parameter('k_accel').value
        self.k_brake = self.get_parameter('k_brake').value
        self.k_drag  = self.get_parameter('k_drag').value

        self.f_min = self.get_parameter('f_min').value
        self.f_max = self.get_parameter('f_max').value
        self.A_min = self.get_parameter('A_min').value
        self.A_max = self.get_parameter('A_max').value

        self.rate_hz = self.get_parameter('rate_hz').value

        # 9) Initialize state for send_movement_tail
        self.v = 0.0
        self._last_time = time.monotonic()
        self._last_cmd_x = 0.0
        self._last_cmd_y = 0.0
        self.start = time.monotonic()
        self._phase = 0.0


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
        self.create_service(Trigger, 'teleoperation/toggle_neutral',   self.handle_toggle_neutral)

        # 4) Internal state & thread
        self.sync_flag  = True
        self.running    = False
        self.magnet_on  = False
        self.neutral_on = False
        self.exit_event = threading.Event()
        self.thread_tail     = threading.Thread(target=self.send_movement_tail)
        self.thread_tail.start()
        self.thread_fins     = threading.Thread(target=self.send_movement_fins)
        self.thread_fins.start()

        self.get_logger().info('[PezController] Initialized.')

    def default_values(self):
        self.tail_amplitude.current = self.tail_amplitude.default
        self.tail_offset.current    = self.tail_offset.default
        self.fin_dive_ref.current   = 0
        self.fin_turn_off.current   = 0
        self.cam_pwm.current = self.cam_pwm.default


    def handle_start(self, request, response):
        self.default_values()

        self.running = True
        self.get_logger().info('[PezController] start_swim → running')
        response.success = True
        response.message = 'started'
        return response

    def handle_stop(self, request, response):
        self.default_values()

        self.running = False
        self.get_logger().info('[PezController] stop_swim → stopped')
        time.sleep(0.4) # Rate 5Hz * 2
        self.nav.set_pwm_channels_values(
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
        self.nav.set_pwm_channel_duty_cycle(PwmChannel.Ch9, value)
        self.get_logger().info(f'[PezController] Magnet → {state}')
        response.success = True
        response.message = f'magnet {state.lower()}'
        return response
    
    def handle_toggle_neutral(self, request, response):
        self.neutral_on = not self.neutral_on
        state = 'ON' if self.neutral_on else 'OFF'
        self.running = not self.neutral_on
        self.get_logger().info(f'[PezController] Neutral → {state}')
        response.success = True
        response.message = f'neutal {state.lower()}'
        return response
    
    def _update_speed(self, x, dt):
        """
        v ← v + k_accel*x*dt  (x<0 now *subtracts*)
        – k_drag*v*dt       (always applies drag)
        clamp into [0,1].
        """
        # 1) apply throttle accel (x in [-1,1])
        self.v += self.k_accel * x * dt

        # 2) apply drag (always slowing proportional to current v)
        self.v -= self.k_drag * self.v * dt

        # 3) clamp
        self.v = max(0.0, min(1.0, self.v))

    
    def _tail_freq(self):
        # v=0 → slow gait; v=1 → fast gait
        tail_freq_slow = 1.5
        tail_freq_fast = 5
        return (tail_freq_slow + (tail_freq_fast - tail_freq_slow) * self.v)

    def _tail_ampl(self):
        """
        Linearly interpolate half‐stroke amplitude in PWM units
        between slow and fast settings based on v∈[0,1].
        """

        tail_pwm_amp_slow, tail_pwm_amp_fast = 43.0, 28.7
        return (tail_pwm_amp_slow + (tail_pwm_amp_fast - tail_pwm_amp_slow) * self.v
        )
    def _tail_pwm(self, y):
        """
        y: lateral input ∈[-1,1] used for DC steering offset
        """
        center = self.tail_pwm.default
        offset = self.O_max * y
        A      = self._tail_ampl()
        pwm    = center + offset + A * math.sin(self._phase)
        return int(self.tail_pwm.clamp(pwm))


    def send_movement_tail(self):
        period  = 1.0 / self.rate_hz
        while not self.exit_event.is_set():
            if not self.running:
                time.sleep(period)
                continue
            now = time.monotonic()
            dt  = min(now - self._last_time, 1.0/self.rate_hz * 1.5)  # max 1.5× your period
            self._last_time = now

            # read the latest x,y
            x, y = self._last_cmd_x, self._last_cmd_y

            # 1) update internal v
            self._update_speed(x, dt)

            # 2) advance phase
            f = self._tail_freq()
            self._phase = (self._phase + 2*math.pi * f * dt) % (2*math.pi)


            # 3) compute & send new tail PWM
            pwm = self._tail_pwm(y)
            self.nav.set_pwm_channels_values([PwmChannel.Ch16], [pwm])

            time.sleep(period)
        self.get_logger().info(f'[PezController] Tail → Ended')


    def send_movement_fins(self):
        while not self.exit_event.is_set():
            if not self.running:
                time.sleep(1.0 / self.rate_hz)
                continue
            if self.sync_flag:
                lf, rf     = int(self.left_fin_pwm.current), int(self.right_fin_pwm.current)
                if lf != self.left_fin_pwm.default or rf != self.right_fin_pwm.default:
                    self.nav.set_pwm_channels_values(
                        [PwmChannel.Ch14, PwmChannel.Ch15],
                        [lf, rf]
                    )
                time.sleep(1.0 / self.rate_hz)
        self.get_logger().info(f'[PezController] Fins → Ended')


    def controller_callback(self, msg: Twist):
        self.sync_flag = False
        y, z = msg.linear.y, msg.linear.z
        self._last_cmd_x = msg.linear.x
        self._last_cmd_y = y

        # Fins dive & turn
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
        if cam_offset == 0:
            return
        self.cam_pwm.current = self.cam_pwm.clamp(self.cam_pwm.current + cam_offset)
        cam = int(self.cam_pwm.current)
        self.nav.set_pwm_channels_values([PwmChannel.Ch11], [cam])
        self.get_logger().info(f'[PezController] Camera → {cam}')




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
        node.get_logger().info('KeyboardInterrupt—shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()