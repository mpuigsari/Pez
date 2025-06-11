# src/pez_comms/plugins/host_side.py

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet
from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler


def register(node: Node, cfg: dict):
    """
    plugin: host_side
    cfg keys expected:
      - packet_a:       string packet ID for A_NORMAL
      - packet_b:       string packet ID for B_COMMAND
      - topic_cmd_vel:  e.g. '/pez/cmd_vel'
      - topic_camera:   e.g. '/pez/camera_control'
      - services:       dict mapping svc_id→ros service name
      - camera_svc_id:  int (the svc_id for camera_control)
      - broadcast_rate: float (Hz for Packet A loop)
      - ack_timeout:    float (secs to wait for service response)
      - svc_wait:       float (secs to wait for service to become available)
    """
    # 1) Packet defs
    packetA = get_packet(cfg['packet_a'])
    packetB = get_packet(cfg['packet_b'])

    # 2) Internal state on node
    node._axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    node._service_cmd = None
    node._last_seq = 0
    node._ack_got = False
    node._last_ack_success = False
    node._ack_seq = None
    node._ack_ready = False

    # 3) Publishers
    cmd_pub = node.create_publisher(Twist,   cfg['topic_cmd_vel'],  10)
    cam_pub = node.create_publisher(Float32, cfg['topic_camera'], 10)

    # 4) Service clients
    svc_clients = {
        int(sid): node.create_client(Trigger, name)
        for sid, name in cfg['services'].items()
    }
    cam_id = cfg['camera_svc_id']

    # 5) /cmd_vel subscriber
    def vel_cb(msg: Twist):
        node._axes['x'] = msg.linear.x
        node._axes['y'] = msg.linear.y
        node._axes['z'] = msg.linear.z
    node.create_subscription(Twist, cfg['topic_cmd_vel'], vel_cb, 10)

    # 6) /camera_control subscriber
    def cam_cb(msg: Float32):
        if node._service_cmd is not None:
            node.get_logger().warn("Host busy, ignoring camera cmd")
            return
        val = 1 if msg.data >= 0 else 0
        node._service_cmd = (cam_id, val)
        # reset ACK flags
        node._ack_got = node._ack_ready = False
        node.get_logger().info(f"Host queued camera Packet B (svc_id={cam_id}, val={val})")
    node.create_subscription(Float32, cfg['topic_camera'], cam_cb, 10)

    # 7) Teleop service servers
    def make_handler(sid: int, sval: int):
        def handle(req, res):
            if node._service_cmd is not None:
                res.success = False
                res.message = "busy"
                return res
            node._service_cmd = (sid, sval)
            node._ack_got = node._ack_ready = False
            node.get_logger().info(f"Host queued Packet B svc_id={sid}, val={sval}")
            # wait for ack_ready
            t0 = time.time()
            while rclpy.ok() and not node._ack_ready and time.time()-t0 < cfg['ack_timeout']:
                time.sleep(0.01)
            if node._last_ack_success:
                res.success = True
                res.message = "ok"
            else:
                res.success = False
                res.message = "nack"
            return res
        return handle

    for sid_str, srv_name in cfg['services'].items():
        sid = int(sid_str)
        if sid == cam_id:
            continue
        # map value: for start/stop (svc_id 0) we need two handlers
        if sid == 0:
            node.create_service(Trigger, srv_name + "_start", make_handler(0,1))
            node.create_service(Trigger, srv_name + "_stop",  make_handler(0,0))
        else:
            node.create_service(Trigger, srv_name, make_handler(sid,1))

    # 8) Setup the 4‐step scheduler
    def loop_A():
        # send Packet A continuously until a Packet B is queued
        rate = cfg.get('broadcast_rate', 4.0)
        period = 1.0/rate
        while node._service_cmd is None and rclpy.ok():
            x_q = _quant(node._axes['x'], bits=3)
            y_q = _quant(node._axes['y'], bits=2)
            z_q = _quant(node._axes['z'], bits=2)
            raw = packetA.encode(x=x_q,y=y_q,z=z_q)
            node.modem.send_packet(raw)
            time.sleep(period)
        return

    def send_B():
        sid,val = node._service_cmd
        node._last_seq ^= 1
        raw = packetB.encode(service_id=sid, value=val, seq=node._last_seq)
        node.modem.send_packet(raw)

    def wait_ack() -> bool:
        t0 = time.time()
        while rclpy.ok() and time.time()-t0 < cfg['svc_wait']:
            raw = node.modem.get_byte(timeout=0.1)
            if not raw: continue
            fields = packetB.decode(raw)
            if fields['seq'] == node._last_seq:
                node._last_ack_success = True
            else:
                node._last_ack_success = False
            node._ack_seq = fields['seq']
            node._ack_got = node._ack_ready = True
            return True
        node._ack_ready = True
        return True

    def clear():
        node._service_cmd = None
        node._ack_got = False
        node._ack_seq = None

    steps = [
        TransmissionStep('loop_A',     loop_A),
        TransmissionStep('send_B',     send_B,   duration=0.25),
        TransmissionStep('wait_ack',   None,     wait_for=wait_ack),
        TransmissionStep('clear_cmd',  clear),
    ]
    sched = TransmissionScheduler(steps, loop=True)
    sched.start()

    # 9) Cleanup on shutdown
    def cleanup():
        sched.stop()
        sched.join(timeout=1.0)
    node.add_on_shutdown(cleanup)


def _quant(val, bits):
    levels = (1<<bits)-1
    return max(0,min(levels,int(round((val+1)/2*levels))))
