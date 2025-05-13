#!/usr/bin/env python3

import time
import threading
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from pez_core.actuators import HopfCPG, TailActuator, FinActuator, PWMValue, PwmChannel

class FakeNav:
    def __init__(self):
        self.enabled = False
    def init(self):         print("[fake_nav] init()")
    def set_pwm_enable(self, e): 
        self.enabled = e
        print(f"[fake_nav] set_pwm_enable({e})")
    def set_pwm_freq_hz(self, f): print(f"[fake_nav] set_pwm_freq_hz({f})")
    def set_pwm_channels_values(self, ch, vals):
        print(f"[fake_nav] set {list(ch)} → {list(vals)}")
    def set_pwm_channel_duty_cycle(self, ch, v):
        print(f"[fake_nav] duty {ch} → {v}")



class PezController(Node):
    def __init__(self):
        super().__init__('pez_controller')

        # --- declare all parameters with their defaults
        self.declare_parameter('test_nav', False)

        # tail PWM
        self.declare_parameter('tail_pwm_default', 303)
        self.declare_parameter('tail_pwm_min', 176)
        self.declare_parameter('tail_pwm_max', 434)

        # tail offset (relative)
        self.declare_parameter('tail_offset_default', 0)
        self.declare_parameter('tail_offset_min', -127)
        self.declare_parameter('tail_offset_max', 131)

        # fins PWM
        self.declare_parameter('fins_pwm_default', 231)
        self.declare_parameter('fins_pwm_min', 153.5)
        self.declare_parameter('fins_pwm_max', 307)

        # camera PWM (if used)
        self.declare_parameter('cam_pwm_default', 315)
        self.declare_parameter('cam_pwm_min', 215)
        self.declare_parameter('cam_pwm_max', 415)

        # cruise‐control
        self.declare_parameter('cruise_speed_min', 0.0)
        self.declare_parameter('cruise_speed_max', 1.5)
        self.declare_parameter('cruise_freq_min', 0.4)
        self.declare_parameter('cruise_freq_max', 2.5)
        self.declare_parameter('cruise_amp_min', 20.0)
        self.declare_parameter('cruise_amp_max', 60.0)
        self.declare_parameter('cruise_k_freq', 5.0)
        self.declare_parameter('cruise_k_amp', 5.0)

        # Tuning scalars
        self.declare_parameter('tail_max_deflect', 75)
        self.declare_parameter('tail_blend',       0.2)
        self.declare_parameter('fin_max_deflect',  75)
        self.declare_parameter('fin_blend',        0.5)

        # Hopf‐CPG
        self.declare_parameter('cpg_alpha', 1.0)
        self.declare_parameter('cpg_dt',    0.05)

        # --- read them back into Python
        p = self.get_parameter

        if p('test_nav').value:
            self.nav = FakeNav()
        else:
            import bluerobotics_navigator.bluerobotics_navigator as navigator
            self.nav = navigator
        
        self.nav.init()
        self.nav.set_pwm_enable(True)
        self.nav.set_pwm_freq_hz(50)

        # build PWMValue objects
        self.tail_pwm       = PWMValue(p('tail_pwm_default').value,
                                       p('tail_pwm_min').value,
                                       p('tail_pwm_max').value,
                                       p('tail_max_deflect').value,
                                       p('tail_blend').value)
        self.tail_offset    = PWMValue(p('tail_offset_default').value,
                                       p('tail_offset_min').value,
                                       p('tail_offset_max').value
                                       )

        self.left_fin_pwm  = PWMValue(p('fins_pwm_default').value,
                                      p('fins_pwm_min').value,
                                      p('fins_pwm_max').value)
        
        self.right_fin_pwm = PWMValue(p('fins_pwm_default').value,
                                      p('fins_pwm_min').value,
                                      p('fins_pwm_max').value)

        self.cam_pwm = PWMValue(p('cam_pwm_default').value,
                                p('cam_pwm_min').value,
                                p('cam_pwm_max').value)
        
        fin_deflect = p('fin_max_deflect').value
        fin_blend = p('fin_blend').value
        self.fin_dive_ref = PWMValue(0,    -fin_deflect, fin_deflect, deflect_=fin_deflect, blend_=fin_blend)
        self.fin_turn_off = PWMValue(0,    -fin_deflect, fin_deflect, deflect_=fin_deflect, blend_=fin_blend)

        # cruise‐control params
        self.speed_min, self.speed_max = p('cruise_speed_min').value, p('cruise_speed_max').value
        self.freq_min,  self.freq_max  = p('cruise_freq_min').value,  p('cruise_freq_max').value
        self.amp_min,   self.amp_max   = p('cruise_amp_min').value,   p('cruise_amp_max').value
        self.k_freq,    self.k_amp     = p('cruise_k_freq').value,    p('cruise_k_amp').value

        # Hopf‐CPG
        self.cpg = HopfCPG(alpha=p('cpg_alpha').value, dt=p('cpg_dt').value)

        # tail & fins actuators
        self.tail_act = TailActuator(
            default_pwm       = p('tail_pwm_default').value,
            offset_pwm_value  = self.tail_offset,
            channel           = PwmChannel.Ch16,
            navigator         = self.nav,
        )
        self.fin_act = FinActuator(
            left_pwm_value  = self.left_fin_pwm,
            right_pwm_value = self.right_fin_pwm,
            left_ch         = PwmChannel.Ch14,
            right_ch        = PwmChannel.Ch15,
            navigator       = self.nav,
        )

        # — ROS interfaces (same as before)
        ns = self.get_namespace()
        self.create_subscription(Twist,   f'{ns}/cmd_vel',        self.controller_callback,    10)
        self.create_subscription(Float64, f'{ns}/camera_control',  self.cam_controller_callback,10)
        self.create_service(Trigger, 'teleoperation/start_swim',    self.handle_start)
        self.create_service(Trigger, 'teleoperation/stop_swim',     self.handle_stop)
        self.create_service(Trigger, 'teleoperation/toggle_magnet', self.handle_toggle_magnet)
        self.create_service(Trigger, 'teleoperation/toggle_neutral', self.handle_toggle_neutral)

        # — internal state
        self.desired_speed = 0.0
        self.running       = False
        self.sync_flag = True
        self.magnet_on = False
        self.exit_event    = threading.Event()

        # — two threads, one for tail‐CPG, one for fins
        self.thread_tail = threading.Thread(target=self._tail_loop)
        self.thread_tail.start()
        self.thread_fins = threading.Thread(target=self._fins_loop)
        self.thread_fins.start()

        self.get_logger().info('[PezController] Initialized.')

    def _sigma(self, x, lo, hi, k):
        """Logistic map from [0,1]→[lo,hi]."""
        e = math.exp(-k*(x-0.5))
        return lo + (hi-lo)/(1+e)

    def _tail_loop(self):
        dt = self.cpg.dt
        while not self.exit_event.is_set():
            if not self.running:
                self.nav.set_pwm_enable(False)
                time.sleep(dt)
                continue
            if self.sync_flag:
                self.nav.set_pwm_enable(True)

                # 1) normalize cruise speed
                vnorm = (self.desired_speed - self.speed_min)/(self.speed_max - self.speed_min)
                vnorm = max(0.0, min(1.0, vnorm))

                # 2) σ-map → amplitude & frequency targets
                A_tgt = self._sigma(vnorm, self.amp_min,  self.amp_max,  self.k_amp)
                f_tgt = self._sigma(vnorm, self.freq_min, self.freq_max, self.k_freq)

                # 3) form Hopf params
                mu    = A_tgt * A_tgt
                omega = 2.0 * math.pi * f_tgt

                # 4) step CPG
                x, _ = self.cpg.step(mu, omega)

                # 5) output tail
                self.tail_act.output(x)

                time.sleep(dt)

    def _fins_loop(self):
        rate = 50.0
        while not self.exit_event.is_set():
            if not self.running:
                time.sleep(1.0 / rate)
                continue
            self.nav.set_pwm_enable(True)
            # just replay whatever fin PWMValue.current is
            self.fin_act.output()
            time.sleep(1.0 / rate*0.2)

    def default_values(self):
        self.tail_offset.current    = self.tail_offset.default
        self.fin_dive_ref.current   = 0
        self.fin_turn_off.current   = 0
        self.cam_pwm.current = self.cam_pwm.default
        self.desired_speed = 0.0


    def handle_start(self, request, response):
        self.default_values()
        self.nav.set_pwm_enable(True)
        self.running = True
        self.get_logger().info('[PezController] start_swim → running')
        response.success = True
        response.message = 'started'
        return response

    def handle_stop(self, request, response):
        self.default_values()
        self.running = False
        time.sleep(self.cpg.dt)
        self.nav.set_pwm_enable(False)

        self.get_logger().info('[PezController] stop_swim → stopped')
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
        response.message = f'neutral {state.lower()}'
        return response
    
    def controller_callback(self, msg: Twist):
        self.sync_flag = False
        x, y, z = msg.linear.x, msg.linear.y, msg.linear.z

        # → Cruise‐control for tail speed
        if x != 0.0:
            v = self.desired_speed + x
            self.desired_speed = max(self.speed_min,
                                     min(self.speed_max, v))

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
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
