#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from bluerobotics_navigator import PwmChannel
import bluerobotics_navigator as navigator


class PWMValue:
    def __init__(self, default, min_, max_, deflect_=0, blend_=0):
        self.default = default
        self.current = default
        self.min = min_
        self.max = max_
        self.deflect = deflect_
        self.blend = blend_

    def clamp(self, value):
        return min(max(value, self.min), self.max)

    def clamp_deflect(self, v):
        return min(max(v, -self.deflect), self.deflect)


class PezController(Node):
    def __init__(self):
        super().__init__('move_controller')

        # —————————————
        # 1) Initialize hardware
        # —————————————
        navigator.init()
        navigator.set_pwm_enable(True)
        navigator.set_pwm_freq_hz(50)

        # —————————————
        # 2) Declare & read parameters
        # —————————————
        # tail_pwm dict
        self.declare_parameter(
            'tail_pwm',
            {'default': 303, 'min': 176, 'max': 434}
        )
        tail_params = self.get_parameter('tail_pwm').value
        self.tail_pwm = PWMValue(
            tail_params['default'],
            tail_params['min'],
            tail_params['max']
        )
        # compute deflection limits & amplitude range
        tail_offset_min = self.tail_pwm.min - self.tail_pwm.default
        tail_offset_max = self.tail_pwm.max - self.tail_pwm.default
        Amax = min(abs(tail_offset_min), abs(tail_offset_max))
        self.tail_offset = PWMValue(0, tail_offset_min, tail_offset_max)
        self.tail_amplitude = PWMValue(0, 0, Amax)

        # fins_pwm dict
        self.declare_parameter(
            'fins_pwm',
            {'default': 231, 'min': 153.5, 'max': 307}
        )
        fin_params = self.get_parameter('fins_pwm').value
        self.left_fin_pwm  = PWMValue(fin_params['default'],
                                      fin_params['min'],
                                      fin_params['max'])
        self.right_fin_pwm = PWMValue(fin_params['default'],
                                      fin_params['min'],
                                      fin_params['max'])

        # timer_movement dict
        self.declare_parameter(
            'timer_movement',
            {'default': 0.6, 'min': 0.3, 'max': 1.5}
        )
        timer_params = self.get_parameter('timer_movement').value
        self.timer_movement = PWMValue(timer_params['default'],
                                       timer_params['min'],
                                       timer_params['max'])

        # scalar params
        self.declare_parameter('tail_max_deflect', 125)
        self.declare_parameter('tail_blend', 0.2)
        self.declare_parameter('fin_max_deflect', 75)
        self.declare_parameter('fin_blend', 0.5)
        self.declare_parameter('vel_blend', 0.2)

        self.tail_pwm.deflect   = self.get_parameter('tail_max_deflect').value
        self.tail_pwm.blend     = self.get_parameter('tail_blend').value
        fin_deflect             = self.get_parameter('fin_max_deflect').value
        fin_blend               = self.get_parameter('fin_blend').value
        self.left_fin_pwm.deflect  = fin_deflect
        self.right_fin_pwm.deflect = fin_deflect
        self.left_fin_pwm.blend    = fin_blend
        self.right_fin_pwm.blend   = fin_blend
        self.vel_blend = self.get_parameter('vel_blend').value

        # fine‐tuning PWM helpers
        self.fin_dive_ref = PWMValue(
            0, -fin_deflect, fin_deflect,
            deflect_=fin_deflect, blend_=fin_blend
        )
        self.fin_turn_off = PWMValue(
            0, -fin_deflect, fin_deflect,
            deflect_=fin_deflect, blend_=fin_blend
        )

        # —————————————
        # 3) ROS interfaces
        # —————————————
        ns = self.get_namespace()  # e.g. '/pez/'

        # Subscriber for cmd_vel under the node's namespace
        self.create_subscription(
            Twist,
            f'{ns}cmd_vel',
            self.controller_callback,
            10
        )

        # Services
        self.create_service(Trigger,
                            'teleoperation/start_swim',
                            self.handle_start)
        self.create_service(Trigger,
                            'teleoperation/stop_swim',
                            self.handle_stop)
        self.create_service(Trigger,
                            'teleoperation/toggle_magnet',
                            self.handle_toggle_magnet)

        # —————————————
        # 4) Internal state & thread
        # —————————————
        from threading import Event
        self.sync_flag    = True
        self.running      = False
        self.magnet_on    = False
        self.exit_event   = Event()
        self.thread       = threading.Thread(target=self.send_movement)
        self.thread.start()

        self.get_logger().info('[PezController] Initialized.')

    # —————————————
    # Service callbacks
    # —————————————
    def handle_start(self, request, response):
        self.tail_amplitude.current = self.tail_amplitude.default
        self.tail_offset.current    = self.tail_offset.default
        self.fin_dive_ref.current   = 0
        self.fin_turn_off.current   = 0

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

        self.running = False
        self.get_logger().info('[PezController] stop_swim → stopped')
        # center servos
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
        self.get_logger().info(f'[PezController] Magnet → {state}')
        response.success = True
        response.message = f'magnet {state.lower()}'
        return response

    # —————————————
    # Main PWM‐driving thread
    # —————————————
    def send_movement(self):
        from rclpy.context import Context  # just to silence linter
        rate = 50.0
        while not self.exit_event.is_set():
            if not self.running:
                time.sleep(1.0 / rate)
                continue

            if self.sync_flag:
                tc   = self.tail_pwm.default + self.tail_offset.current
                th   = self.tail_pwm.clamp(tc + self.tail_amplitude.current)
                tl   = self.tail_pwm.clamp(tc - self.tail_amplitude.current)
                lf   = self.left_fin_pwm.current
                rf   = self.right_fin_pwm.current
                t    = self.timer_movement.current

                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                    [th, lf, rf]
                )
                time.sleep(t)
                if self.exit_event.is_set():
                    break
                navigator.set_pwm_channels_values(
                    [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15],
                    [tl, lf, rf]
                )
                time.sleep(t)

    # —————————————
    # Joystick callback
    # —————————————
    def controller_callback(self, msg: Twist):
        self.sync_flag = False
        x, y, z = msg.linear.x, msg.linear.y, msg.linear.z

        # tail deflection (x → amplitude & speed)
        if x != 0.0:
            self._update_tail_motion(x)

        # y → offset
        target = self.tail_offset.default + y * self.tail_pwm.deflect
        self.tail_offset.current += (target - self.tail_offset.current) * self.tail_pwm.blend
        self.tail_offset.current = self.tail_offset.clamp(self.tail_offset.current)

        # fins: dive & auto‐center
        self._update_fins(y, z)

        self.sync_flag = True

    def _update_tail_motion(self, x):
        norm = (x + 1.0) / 2.0
        amp_t = (1.0 - abs(x)) * self.tail_amplitude.max
        tim_t = (self.timer_movement.max -
                 norm * (self.timer_movement.max - self.timer_movement.min))
        for pwm, tgt in ((self.tail_amplitude, amp_t),
                         (self.timer_movement, tim_t)):
            pwm.current = pwm.clamp(pwm.current + (tgt - pwm.current) * self.vel_blend)

    def _update_fins(self, y, z):
        def blend(comp, val):
            tgt = val * comp.deflect
            comp.current = comp.clamp_deflect(comp.current +
                                              (tgt - comp.current) * comp.blend)

        if z != 0.0:
            blend(self.fin_dive_ref, z)
        blend(self.fin_turn_off, y)

        for pwm, sign in ((self.left_fin_pwm, +1), (self.right_fin_pwm, -1)):
            tgt = pwm.default + self.fin_dive_ref.current + sign * self.fin_turn_off.current
            pwm.current = pwm.clamp(pwm.current + (tgt - pwm.current) * pwm.blend)

    def destroy_node(self):
        # signal thread to exit
        self.exit_event.set()
        self.thread.join(timeout=1.0)
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
