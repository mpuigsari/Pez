#!/usr/bin/env python3
"""
pez_comms: ROS2 node to send cmd_vel and teleoperation services over acoustic modems
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from packet_comms import PacketBuilder, ModemCommunicator
from transmission_scheduler import TransmissionStep, TransmissionScheduler
import threading
import time

# shared state
axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}
service_cmd = None  # tuple (service_id, value)
last_seq = 0
ack_buffer = {'got': False, 'seq': None}

# quantization

def quantize(val: float, bits: int) -> int:
    levels = (1 << bits)
    q = round((val + 1) / 2 * (levels - 1))
    return max(0, min(levels - 1, q))

# Packet A send

def send_packet_a():
    x_q = quantize(axes['x'], 3)
    y_q = quantize(axes['y'], 3)
    z_q = quantize(axes['z'], 2)
    pkt = PacketBuilder.build_packet_a_normal(x_q, y_q, z_q)
    modem.send_packet(pkt)
    # debug
    # node.get_logger().debug(f"[PacketA] {pkt.hex()}")

# Packet B send

def send_packet_b():
    global last_seq
    if service_cmd is None:
        return
    service_id, value = service_cmd
    last_seq ^= 1
    pkt = PacketBuilder.build_packet_b(service_id, value, last_seq)
    modem.send_packet(pkt)
    # node.get_logger().debug(f"[PacketB] {pkt.hex()}")

# read ack for Packet B

def wait_for_ack():
    # blocks until ack_buffer['got'] True
    while not ack_buffer['got']:
        data = modem.read_bytes(1)
        if data:
            byte = data[0]
            seq = byte & 0x1
            ack_buffer['got'] = True
            ack_buffer['seq'] = seq
    return True

# clear service and ack

def clear_cmd():
    global service_cmd
    service_cmd = None
    ack_buffer['got'] = False

# ROS2 node

class PezComms(Node):
    def __init__(self):
        super().__init__('pez_comms_master')
        # serial modem
        global modem
        modem = ModemCommunicator(port='/dev/ttyModem0', baud=9600, timeout=0.5)
        # ROS subscriptions
        ns = self.get_namespace()
        self.create_subscription(Twist, f'{ns}/cmd_vel', self.vel_callback, 10)
        # services: start/stop, magnet, neutral, cam mapped to Packet B
        self.create_service(Trigger, f'{ns}/teleoperation/start_swim', self.handle_start)
        self.create_service(Trigger, f'{ns}/teleoperation/stop_swim', self.handle_stop)
        self.create_service(Trigger, f'{ns}/teleoperation/toggle_magnet', self.handle_magnet)
        self.create_service(Trigger, f'{ns}/teleoperation/toggle_neutral', self.handle_neutral)
        self.create_service(Trigger, f'{ns}/teleoperation/cam_left', self.handle_cam_left)
        self.create_service(Trigger, f'{ns}/teleoperation/cam_right', self.handle_cam_right)
        # scheduler thread
        steps = [
            TransmissionStep('loop_A', send_packet_a),
            TransmissionStep('await_srv', None, wait_for=lambda: service_cmd is not None),
            TransmissionStep('send_B', send_packet_b, duration=0.25),
            TransmissionStep('wait_ack', None, wait_for=wait_for_ack),
            TransmissionStep('clear', clear_cmd),
        ]
        self.scheduler = TransmissionScheduler(steps, loop=True)
        self.scheduler.start()
        self.get_logger().info('PezComms initialized')

    def vel_callback(self, msg: Twist):
        axes['x'] = msg.linear.x
        axes['y'] = msg.linear.y
        axes['z'] = msg.linear.z

    # common handler wrapper
    def _svc_handler(self, service_id, val, request, response):
        global service_cmd
        if service_cmd is not None:
            response.success = False
            response.message = 'busy'
            return response
        service_cmd = (service_id, val)
        # wait until ack arrives
        while not ack_buffer['got']:
            time.sleep(0.01)
        # verify seq matches
        response.success = True
        response.message = 'ok'
        return response

    def handle_start(self, request, response):
        return self._svc_handler(0, 1, request, response)
    def handle_stop(self, request, response):
        return self._svc_handler(0, 0, request, response)
    def handle_magnet(self, request, response):
        return self._svc_handler(1, 1, request, response)
    def handle_neutral(self, request, response):
        return self._svc_handler(2, 1, request, response)
    def handle_cam_left(self, request, response):
        return self._svc_handler(3, 0, request, response)  # cam -1
    def handle_cam_right(self, request, response):
        return self._svc_handler(3, 1, request, response)  # cam +1

    def destroy_node(self):
        self.scheduler.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PezComms()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
