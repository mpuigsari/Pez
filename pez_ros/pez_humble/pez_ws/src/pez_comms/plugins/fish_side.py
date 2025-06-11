# pez_comms/plugins/fish_side.py

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet


def register(node: Node, cfg: dict):
    """
    Called by comms_full_node with:
      node: the running CommsFullNode (has node.modem, node._dequant, etc.)
      cfg: the dict from config/plugins entry
    Expected cfg keys:
      port_param:       name of ROS param holding serial port
      baud:             integer baud
      timeout:          float read timeout
      packet_a:         packet ID string for A_NORMAL
      packet_b:         packet ID string for B_COMMAND
      topic_cmd_vel:    e.g. '/pez/cmd_vel'
      topic_camera:     e.g. '/pez/camera_control'
      service_map:      map of svc_id→service_name (as strings)
      camera_svc_id:    int (the svc_id that triggers camera)
    """

    # 1) Get serial settings from parameters / cfg
    port = node.get_parameter(cfg['port_param']).get_parameter_value().string_value
    baud = cfg.get('baud', 9600)
    timeout = cfg.get('timeout', 0.1)

    # Reconfigure the existing modem (assumes comms_full already opened node.modem)
    node.modem.reopen(port=port, baud=baud, timeout=timeout)
    node.get_logger().info(f"[fish_side] modem reconfigured: {port}@{baud}")

    # 2) Packet definitions
    packetA = get_packet(cfg['packet_a'])
    packetB = get_packet(cfg['packet_b'])

    # 3) Publishers
    cmd_vel_pub = node.create_publisher(Twist, cfg['topic_cmd_vel'], 10)
    camera_pub  = node.create_publisher(Float32, cfg['topic_camera'], 10)

    # 4) Service clients mapping
    service_map = {int(k): v for k, v in cfg['service_map'].items()}
    camera_id   = cfg.get('camera_svc_id', 3)

    # 5) Serial‐read loop
    stop_event = threading.Event()

    def serial_loop():
        while rclpy.ok() and not stop_event.is_set():
            raw = node.modem.get_byte(timeout)  # returns bytes or None
            if raw is None:
                continue

            byte = raw[0]
            # Packet A (bit7=0)
            if (byte & 0x80) == 0:
                fields = packetA.decode(raw)
                x = node._dequant(fields['x_q'], bits=3)
                y = node._dequant(fields['y_q'], bits=2)
                z = node._dequant(fields['z_q'], bits=2)
                twist = Twist()
                twist.linear.x = x
                twist.linear.y = y
                twist.linear.z = z
                cmd_vel_pub.publish(twist)

            # Packet B (bit7=1)
            else:
                fields = packetB.decode(raw)
                svc_id = fields['service_id']
                val    = fields['value']
                seq    = fields['seq']

                ack = False

                # 5a) Fish‐teleop services (0,1,2)
                if svc_id in service_map:
                    cli = node.create_client(Trigger, service_map[svc_id])
                    if cli.wait_for_service(timeout_sec=5.0):
                        fut = cli.call_async(Trigger.Request())
                        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                        if fut.done() and fut.result().success:
                            ack = True

                # 5b) Camera control (svc_id == camera_id)
                elif svc_id == camera_id:
                    d = 1.0 if val == 1 else -1.0
                    msg = Float32(data=d)
                    camera_pub.publish(msg)
                    ack = True

                # 5c) Send ACK / NACK
                resp_seq = seq if ack else (1 - seq)
                ack_pkt = packetB.encode(
                    service_id=svc_id, value=val, seq=resp_seq
                )
                node.modem.send_packet(ack_pkt)

    thread = threading.Thread(target=serial_loop, daemon=True)
    thread.start()

    # 6) Clean up on shutdown
    def cleanup():
        stop_event.set()
        thread.join(timeout=1.0)

    node.add_on_shutdown(cleanup)
