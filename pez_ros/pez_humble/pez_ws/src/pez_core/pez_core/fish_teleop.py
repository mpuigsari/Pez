#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import Trigger
import math
import time

class PWMValue:
    def __init__(self, default, min_, max_):
        self.default = default
        self.current = default
        self.min     = min_
        self.max     = max_

    def clamp(self, value):
        return min(max(value, self.min), self.max)



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

         # --- 1. PARAMETER SPECS: name, default, type, (min,max,step) ---
        param_specs = [
            ('k_accel',           0.5,   float, (0.0, 2.0, 0.05)),
            ('k_drag',            0.3,   float, (0.0, 3.0, 0.05)),
            ('tail_freq_slow',    1.5,   float, (0.1, 3.0, 0.1)),
            ('tail_freq_fast',    5.0,   float, (2.0,10.0,0.2)),
            ('tail_deg_amp_slow', 30.0,  float, (0.0,60.0,1.0)),
            ('tail_deg_amp_fast', 20.0,  float, (0.0,60.0,1.0)),
            ('tail_max_deflect',  75,    int,   (0,150,1)),
            ('rate_hz',           50,    int,   (10,200,5)),
            ('fin_max_deflect',   75,    int,   (0,150,1)),
            ('fin_turn_max_deflect', 20, int,   (0,50,1)),
            ('fin_blend',         0.5,   float, (0.0,1.0,0.05)),
        ]

        # --- 2. PWM SPECS: attr-name, default, min, max ---
        pwm_specs = [
            ('cam_pwm',  290, 190, 390),
            ('tail_pwm', 303, 176, 434),
            ('left_fin_pwm',  231, 153.5, 307),
            ('right_fin_pwm',  231, 153.5, 307)
        ]

        # loop over param_specs → declare + read into self.<name>
        for name, default, ptype, (mn, mx, st) in param_specs:
            desc = ParameterDescriptor()
            if ptype is float:
                desc.floating_point_range = [
                    FloatingPointRange(from_value=mn, to_value=mx, step=st)
                ]
            else:
                desc.integer_range = [
                    IntegerRange(from_value=int(mn), to_value=int(mx), step=int(st))
                ]
            self.declare_parameter(name, default, desc)
            setattr(self, name, self.get_parameter(name).value)

        # loop over pwm_specs → declare *_default/min/max + build PWMValue
        for attr, d, lo, hi in pwm_specs:
            for suffix in ('default','min','max'):
                key = f'{attr}_{suffix}'
                val = {'default':d, 'min':lo, 'max':hi}[suffix]
                self.declare_parameter(key, val)
            dd = self.get_parameter(f'{attr}_default').value
            mn = self.get_parameter(f'{attr}_min').value
            mx = self.get_parameter(f'{attr}_max').value
            setattr(self, attr, PWMValue(dd, mn, mx))

        # catch any slider moves at runtime
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        #  ➤ compute amplitude in PWM units from degree sliders
        deg_to_pwm = (self.tail_pwm.max - self.tail_pwm.min) / 180.0
        self.tail_pwm_amp_slow = self.tail_deg_amp_slow * deg_to_pwm
        self.tail_pwm_amp_fast = self.tail_deg_amp_fast * deg_to_pwm

        self.fin_dive_ref   = 0
        self.fin_turn_off   = 0

        # 4) Initialize state for send_movement_tail
        self.v = 0.0
        self._last_time = time.monotonic()
        self._last_cmd_x = 0.0
        self._last_cmd_y = 0.0
        self.start = time.monotonic()
        self._phase = 0.0
        


        # 5) ROS interfaces
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

        # Publishers for PlotJuggler
        self.pwm_pub   = self.create_publisher(Float32MultiArray, f'{ns}/pwm',   10)
        self._last_pwm = {
            'tail':   float(self.tail_pwm.default),
            'fin_l':  float(self.left_fin_pwm.default),
            'fin_r':  float(self.right_fin_pwm.default),
            'camera': float(self.cam_pwm.default),
            'magnet': 0.0
        }


        # Teleoperation services
        self.create_service(Trigger, f'{ns}/teleoperation/swim_start',      self.handle_start)
        self.create_service(Trigger, f'{ns}/teleoperation/swim_stop',       self.handle_stop)
        self.create_service(Trigger, f'{ns}/teleoperation/toggle_magnet',   self.handle_toggle_magnet)
        self.create_service(Trigger, f'{ns}/teleoperation/toggle_neutral',   self.handle_toggle_neutral)

        # 6) Internal state & thread
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

    
    def _on_parameters_changed(self, params):

        deg_to_pwm = (self.tail_pwm.max - self.tail_pwm.min) / 180.0

        for p in params:
            if p.type_ not in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                continue
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
            # if either amplitude‐in‐degrees slider moved, recompute the PWM half‐stroke
            if p.name in ('tail_deg_amp_slow', 'tail_deg_amp_fast'):
                self.tail_pwm_amp_slow = self.tail_deg_amp_slow * deg_to_pwm
                self.tail_pwm_amp_fast = self.tail_deg_amp_fast * deg_to_pwm

        return SetParametersResult(successful=True)


    def default_values(self):
        self.fin_dive_ref   = 0
        self.fin_turn_off   = 0
        self.cam_pwm.current = self.cam_pwm.default
        self.left_fin_pwm.current = self.left_fin_pwm.default
        self.right_fin_pwm.current = self.right_fin_pwm.default


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
        time.sleep(2.0 / self.rate_hz) # Rate * 2
        self.nav.set_pwm_channels_values(
            [PwmChannel.Ch16, PwmChannel.Ch14, PwmChannel.Ch15, PwmChannel.Ch11],
            [self.tail_pwm.default,
             self.left_fin_pwm.default,
             self.right_fin_pwm.default,
             self.cam_pwm.default]
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
        self._last_pwm['magnet'] = value
        self._publish_all_pwm()
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
        return (self.tail_freq_slow + (self.tail_freq_fast - self.tail_freq_slow) * self.v)

    def _tail_ampl(self):
        """
        Linearly interpolate half‐stroke amplitude in PWM units
        between slow and fast settings based on v∈[0,1].
        """
        return (self.tail_pwm_amp_slow + (self.tail_pwm_amp_fast - self.tail_pwm_amp_slow) * self.v
        )
    def _tail_pwm(self, y):
        """
        y: lateral input ∈[-1,1] used for DC steering offset
        """
        center = self.tail_pwm.default
        offset = self.tail_max_deflect * y
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
            self._last_pwm['tail'] = float(pwm)
            self._publish_all_pwm()

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
                self._last_pwm['fin_l'] = float(lf)
                self._last_pwm['fin_r'] = float(rf)
                self._publish_all_pwm()
                time.sleep(1.0 / self.rate_hz)
        self.get_logger().info(f'[PezController] Fins → Ended')


    def controller_callback(self, msg: Twist):
        self.sync_flag = False
        y, z = msg.linear.y, msg.linear.z
        self._last_cmd_x = msg.linear.x
        self._last_cmd_y = y

        # Fins dive & turn
        def blend_ref(ref, input_val):
            return ref + (input_val - ref) * self.fin_blend

        # update refs
        self.fin_dive_ref   = blend_ref(self.fin_dive_ref,   z* self.fin_max_deflect)
        self.fin_turn_off   = blend_ref(self.fin_turn_off,   y * self.fin_turn_max_deflect)

        # compute & blend each fin’s PWM
        for pwm_obj, sign in ((self.left_fin_pwm, +1),(self.right_fin_pwm, -1)):
            # target = default + dive + (± turn)
            target = pwm_obj.default + sign*self.fin_dive_ref + self.fin_turn_off  # Sign applied for inverse position between fins
            # clamp then blend into current
            clamped = pwm_obj.clamp(target)
            pwm_obj.current += (clamped - pwm_obj.current) * self.fin_blend

        self.sync_flag = True

    def cam_controller_callback(self, msg: Float64):
        cam_offset = msg.data
        if cam_offset == 0:
            return
        
        self.cam_pwm.current = self.cam_pwm.clamp(self.cam_pwm.current + cam_offset)
        cam = int(self.cam_pwm.current)
        self.nav.set_pwm_channels_values([PwmChannel.Ch11], [cam])
        self._last_pwm['camera'] = float(self.cam_pwm.current)
        self._publish_all_pwm()
        self.get_logger().info(f'[PezController] Camera → {cam}')

    def _publish_all_pwm(self):
        msg = Float32MultiArray()
        # One dimension of length 5
        msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label='tail',      size=5, stride=5),
                MultiArrayDimension(label='fin_left',  size=5, stride=5),
                MultiArrayDimension(label='fin_right', size=5, stride=5),
                MultiArrayDimension(label='camera',    size=5, stride=5),
                MultiArrayDimension(label='magnet',    size=5, stride=5),
            ],
            data_offset=0
        )
        msg.data = [
            self._last_pwm['tail'],
            self._last_pwm['fin_l'],
            self._last_pwm['fin_r'],
            self._last_pwm['camera'],
            self._last_pwm['magnet'],
        ]
        self.pwm_pub.publish(msg)




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