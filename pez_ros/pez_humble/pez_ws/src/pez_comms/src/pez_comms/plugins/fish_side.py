import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet

def register(node: Node, cfg: dict):
    """
    Fish-side plugin supporting both AB and 40-bit modes.

    In 'AB' mode:
      • PacketA → cmd_vel
      • PacketB_Full → teleop services or camera control + ACK/NACK

    In '40' mode:
      • Packet40 → cmd_vel
      • svc_pending in Packet40 → teleop services or camera control + ACK/NACK via PacketB_Full

    cfg keys:
      mode           : 'AB' or '40'
      packet_a       : str
      packet_b       : str    # new 40-bit PacketB_Full key
      packet_40      : str
      topic_cmd_vel  : str
      topic_camera   : str
      service_map    : dict
      camera_svc_id  : int
    """
    mode = str(cfg.get('mode', 'AB')).upper()
    node.get_logger().info(f"Initializing fish_side plugin in mode '{mode}'")

    # Load packets
    packetA = get_packet(cfg['packet_a'])
    packetB = get_packet(cfg['packet_b'])  # PacketB_Full expected
    packet40 = None
    if mode == '40':
        try:
            packet40 = get_packet(cfg['packet_40'])
        except KeyError:
            node.get_logger().error(
                f"Packet ID '{cfg.get('packet_40')}' not found. Falling back to AB mode.")
            mode = 'AB'
            packet40 = None

    # Precompute packet lengths
    pA_len = len(packetA.encode(x=0, y=0, z=0))
    pB_len = len(packetB.encode(seq=0, service_id=0, value=0))
    p40_len = len(packet40.encode(
        seq=0, vx=0, vy=0, vz=0, svc_pending=0, svc_id=0, svc_val=0
    )) if packet40 else 0

    # Publishers and service map
    cmd_pub = node.create_publisher(Twist, cfg['topic_cmd_vel'], 10)
    cam_pub = node.create_publisher(Float64, cfg['topic_camera'], 10)
    svc_map = {int(k): v for k, v in cfg['service_map'].items()}
    cam_id = int(cfg['camera_svc_id'])

    def handle_A(fields: dict):
        twist = Twist()
        twist.linear.x = node._dequant(fields['x_q'], bits=3)
        twist.linear.y = node._dequant(fields['y_q'], bits=2)
        twist.linear.z = node._dequant(fields['z_q'], bits=2)
        cmd_pub.publish(twist)
        node.get_logger().info("Processed PacketA")

    def handle_B(fields: dict):
        sid = fields['service_id']
        val = fields['value']
        seq = fields['seq']
        ack = False
        # Teleop services
        if sid in svc_map:
            base = svc_map[sid]
            name = f"{base}_start" if sid == 0 and val else (
                   f"{base}_stop"  if sid == 0 else base)
            node.get_logger().info(f"Calling service {name}")
            cli = node.create_client(Trigger, name)
            if cli.wait_for_service(timeout_sec=5.0):
                fut = cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                ack = fut.done() and fut.result().success
                node.get_logger().info(f"Service {name} responded ack={ack}")
        # Camera control
        elif sid == cam_id:
            cam_pub.publish(Float64(data=1.0 if val else -1.0))
            ack = True
        # Prepare ACK/NACK
        resp_seq = seq if ack else (1 - seq)
        pkt = packetB.encode(seq=resp_seq, service_id=sid, value=val)
        node.modem.send_packet(pkt)
        node.get_logger().info(f"Sent PacketB_Full ACK seq={resp_seq}")

    def handle_40(fields: dict):
        twist = Twist()
        twist.linear.x = node._dequant(fields['vx'], bits=8)
        twist.linear.y = node._dequant(fields['vy'], bits=8)
        twist.linear.z = node._dequant(fields['vz'], bits=8)
        cmd_pub.publish(twist)
        node.get_logger().info("Processed Packet40")
        if fields.get('svc_pending', 0):
            node.get_logger().info("Handling pending service from Packet40")
            handle_B({
                'service_id': fields['svc_id'],
                'value':      fields['svc_val'],
                'seq':        fields['seq']
            })

    def serial_loop():
        node.get_logger().info("Starting fish-side serial loop")
        while rclpy.ok():
            first = node.modem.get_byte(timeout=0.1)
            if first is None:
                continue
            pid = first[0]
            # Determine expected length
            if mode == '40' and packet40 and pid == packet40.packet_id:
                exp_len = p40_len
            elif pid == packetA.packet_id:
                exp_len = pA_len
            elif pid == packetB.packet_id:
                exp_len = pB_len
            else:
                node.get_logger().debug(f"Unknown PID {pid}, skipping")
                continue
            # Accumulate full frame
            buf = bytearray(first)
            start = time.time()
            while len(buf) < exp_len and (time.time() - start) < 0.1:
                nxt = node.modem.get_byte(timeout=0.1)
                if nxt:
                    buf.extend(nxt)
            if len(buf) != exp_len:
                node.get_logger().warn(f"Incomplete packet: got {len(buf)}/{exp_len} bytes")
                continue
            raw = bytes(buf)
            hex_str = ' '.join(f"{b:02X}" for b in raw)
            node.get_logger().info(f"Received full packet ({exp_len} bytes): {hex_str}")
            # Decode
            try:
                if pid == packetA.packet_id:
                    handle_A(packetA.decode(raw))
                elif pid == packetB.packet_id:
                    handle_B(packetB.decode(raw))
                elif packet40 and pid == packet40.packet_id:
                    handle_40(packet40.decode(raw))
            except Exception as e:
                node.get_logger().warn(f"Decode failed: {e}, data={hex_str}")

    thread = threading.Thread(target=serial_loop, daemon=True)
    thread.start()
    node.add_on_shutdown(lambda: thread.join(timeout=1.0))
