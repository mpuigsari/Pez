import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet


def register(node: Node, cfg: dict):
    """
    Fish‐side plugin: handles either:
      • AB mode: PacketA → cmd_vel, PacketB → service/camera + ACK
      • 40 mode: PACKET_40 → cmd_vel or service/camera + ACK via PacketB

    cfg keys:
      mode           : 'AB' or '40'
      packet_a       : str  # e.g. 'A_NORMAL'
      packet_b       : str  # e.g. 'B_COMMAND'
      packet_40      : str  # e.g. 'PACKET_40'
      topic_cmd_vel  : str  # e.g. 'cmd_vel'
      topic_camera   : str  # e.g. 'camera_control'
      service_map    : dict # {svc_id (str): service_name}
      camera_svc_id  : int
    """

    mode      = cfg.get('mode', 'AB').upper()
    packetA   = get_packet(cfg['packet_a'])
    packetB   = get_packet(cfg['packet_b'])
    packet40  = get_packet(cfg.get('packet_40', ''))
    cmd_vel   = node.create_publisher(Twist,   cfg['topic_cmd_vel'], 10)
    cam_pub   = node.create_publisher(Float32, cfg['topic_camera'],  10)
    svc_map   = {int(k): v for k, v in cfg['service_map'].items()}
    cam_id    = int(cfg['camera_svc_id'])

    def handle_A(raw: bytes):
        f = packetA.decode(raw)
        x = node._dequant(f['x_q'], bits=3)
        y = node._dequant(f['y_q'], bits=2)
        z = node._dequant(f['z_q'], bits=2)
        twist = Twist(linear=twist.linear.__class__(x, y, z))
        cmd_vel.publish(twist)

    def handle_B(raw: bytes):
        f    = packetB.decode(raw)
        sid  = f['service_id']
        val  = f['value']
        seq  = f['seq']
        ack  = False

        # teleop services
        if sid in svc_map:
            cli = node.create_client(Trigger, svc_map[sid])
            if cli.wait_for_service(timeout_sec=5.0):
                fut = cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                ack = fut.result().success if fut.done() else False

        # camera  
        elif sid == cam_id:
            msg = Float32(data=1.0 if val else -1.0)
            cam_pub.publish(msg)
            ack = True

        # send ACK/NACK
        resp_seq = seq if ack else (1 - seq)
        pkt      = packetB.encode(service_id=sid, value=val, seq=resp_seq)
        node.modem.send_packet(pkt)

    def handle_40(raw: bytes):
        f           = packet40.decode(raw)
        seq         = f['seq']
        vx, vy, vz  = f['vx'], f['vy'], f['vz']
        pending     = f['svc_pending']
        sid, val    = f['svc_id'], f['svc_val']

        if pending == 0:
            # publish 3-axis cmd_vel
            x = node._dequant(vx, bits=8)
            y = node._dequant(vy, bits=8)
            z = node._dequant(vz, bits=8)
            twist = Twist(linear=twist.linear.__class__(x, y, z))
            cmd_vel.publish(twist)
        else:
            # service / camera
            ack = False
            if sid in svc_map:
                cli = node.create_client(Trigger, svc_map[sid])
                if cli.wait_for_service(timeout_sec=5.0):
                    fut = cli.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                    ack = fut.result().success if fut.done() else False
            elif sid == cam_id:
                msg = Float32(data=1.0 if val else -1.0)
                cam_pub.publish(msg)
                ack = True

            # respond with Packet B
            resp_seq = seq if ack else (1 - seq)
            pkt      = packetB.encode(service_id=sid, value=val, seq=resp_seq)
            node.modem.send_packet(pkt)

    # Register based on mode
    if mode == 'AB':
        node.register_serial_handler(packetA.packet_id, handle_A)
        node.register_serial_handler(packetB.packet_id, handle_B)
    elif mode == '40':
        node.register_serial_handler(packet40.packet_id, handle_40)
    else:
        raise RuntimeError(f"fish_side: unsupported mode '{mode}'")
