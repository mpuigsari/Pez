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
    Host-side plugin: continuously streams control packets and handles service commands.

    In 'AB' mode:
      • PacketA at broadcast_rate until a service command, then PacketB_Full + await ACK/NACK
    In '40' mode:
      • Packet40 once per second (with svc_pending flag), handling ACK/NACK via PacketB_Full

    cfg keys:
      mode           : 'AB' or '40'
      packet_a       : str    # e.g. 'A_NORMAL'
      packet_b       : str    # new 40-bit PacketB_Full key
      packet_40      : str    # e.g. 'PACKET_40'
      topic_cmd_vel  : str
      topic_camera   : str
      services       : dict
      camera_svc_id  : int
      broadcast_rate : float
      ack_timeout    : float
      svc_wait       : float
    """
    # Parse configuration
    mode = cfg.get('mode', 'AB').upper()
    node.get_logger().info(f"Initializing host_side plugin in mode '{mode}'")

    # Load packet definitions
    packetA = get_packet(cfg['packet_a'])
    packetB = get_packet(cfg['packet_b'])  # should be PacketB_Full
    packet40 = None
    if mode == '40':
        try:
            packet40 = get_packet(cfg['packet_40'])
        except KeyError:
            node.get_logger().error(
                f"Packet ID '{cfg.get('packet_40')}' not found. Falling back to AB mode.")
            mode = 'AB'

    # Precompute ACK frame length (ID + 32-bit payload + CRC8)
    pB_len = len(packetB.encode(seq=0, service_id=0, value=0))

    # Timing parameters
    rate = float(cfg.get('broadcast_rate', 4.0))
    ack_timeout = float(cfg.get('ack_timeout', 5.0))
    svc_wait = float(cfg.get('svc_wait', 5.0))
    cam_id = int(cfg['camera_svc_id'])

    # Shared state
    axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    svc_cmd = None
    last_seq = 0
    ack_ready = threading.Event()
    ack_success = False
    stop_event = threading.Event()

    # Quantization helper
    def quant(value, bits):
        max_level = (1 << bits) - 1
        q = max(0, min(max_level,
                       int(round((value + 1.0)/2.0 * max_level))))
        node.get_logger().debug(f"Quantized {value:.3f} to {q} ({bits} bits)")
        return q

    # Velocity subscription
    node.create_subscription(
        Twist, cfg['topic_cmd_vel'],
        lambda msg: axes.update(
            x=msg.linear.x, y=msg.linear.y, z=msg.linear.z),
        10)

    # Camera tilt subscription
    def camera_cb(msg: Float64):
        nonlocal svc_cmd
        if svc_cmd is None and msg.data != 0:
            svc_cmd = (cam_id, 1 if msg.data > 0 else 0)
            ack_ready.clear()
            node.get_logger().info(
                f"Camera command queued: svc_id={svc_cmd[0]}, value={svc_cmd[1]}")
    node.create_subscription(
        Float64, cfg['topic_camera'], camera_cb, 10)

    # Teleop service servers
    for sid_str, base in cfg['services'].items():
        sid = int(sid_str)
        if sid == cam_id:
            continue
        def make_handler(svc_id, val):
            def handler(req, resp):
                nonlocal svc_cmd, ack_ready, ack_success
                if svc_cmd is not None:
                    resp.success = False
                    resp.message = 'busy'
                    return resp
                svc_cmd = (svc_id, val)
                node.get_logger().info(
                    f"Service command queued: svc_id={svc_id}, value={val}")
                start = time.time()
                while (rclpy.ok() and not ack_ready.is_set() and
                       (time.time() - start) < ack_timeout):
                    time.sleep(0.01)
                resp.success = ack_success
                resp.message = 'ack' if ack_success else 'nack'
                node.get_logger().info(
                    f"Service svc_id={svc_id} completed: success={ack_success}")
                return resp
            return handler
        # Create start/stop for sid=0, else single call
        if sid == 0:
            node.create_service(Trigger, f"{base}_start", make_handler(sid, 1))
            node.create_service(Trigger, f"{base}_stop", make_handler(sid, 0))
        else:
            node.create_service(Trigger, base, make_handler(sid, 1))

    # Reader thread: assemble and decode PacketB_Full ACK frames
    def reader_loop():
        nonlocal ack_success, last_seq
        node.get_logger().info("Starting host-side reader thread")
        while rclpy.ok() and not stop_event.is_set():
            if ack_ready.is_set():
                time.sleep(0.01)
                continue

            first = node.modem.get_byte(timeout=5)
            if first is None:
                node.get_logger().warn(
                    f"Incomplete ACK frame: got None bytes"
                )
                continue

            b0   = first[0]
            pid4 = b0 >> 4         # top‐4‐bit ID for PacketB_Full

            # Only proceed if this matches PacketB_Full’s ID
            if pid4 != packetB.packet_id:
                node.get_logger().warn(
                    f"Response's packet ID ({packetB.packet_id}) not matched: {pid4:02X}"
                )
                continue

            # Gather full frame
            buf = bytearray(first)
            start = time.time()
            while len(buf) < pB_len and (time.time() - start) < 0.1:
                nxt = node.modem.get_byte(timeout=0.1)
                if nxt:
                    buf.extend(nxt)

            if len(buf) != pB_len:
                node.get_logger().warn(
                    f"Incomplete ACK frame: got {len(buf)}/{pB_len} bytes"
                )
                continue

            raw = bytes(buf)
            hex_str = ' '.join(f"{b:02X}" for b in raw)
            node.get_logger().info(f"Reader received full ACK: {hex_str}")

            # Decode and signal
            try:
                fields = packetB.decode(raw)
                seq = fields.get('seq', 0)
                # success iff the returned seq matches what we last sent
                ack_success = (seq == last_seq)
                node.get_logger().info(
                    f"Received ACK/NACK seq={seq}, success={ack_success}"
                )
                
            except Exception as e:
                node.get_logger().warn(
                    f"PacketB.decode error: {e}, data={hex_str}"
                )
            finally:
                ack_ready.set()
                


    # Transmission thread: send PacketA/B or Packet40
    def tx_loop():
        nonlocal svc_cmd, last_seq
        node.get_logger().info("Starting host-side transmission thread")

        if mode == 'AB':
            period = 1.0 / rate
            while rclpy.ok() and not stop_event.is_set():
                if svc_cmd is None:
                    xa = quant(axes['x'], 3)
                    ya = quant(axes['y'], 2)
                    za = quant(axes['z'], 2)
                    pkt = packetA.encode(x=xa, y=ya, z=za)
                    node.modem.send_packet(pkt)
                    node.get_logger().info(f"Sent PacketA x={xa},y={ya},z={za}")
                    time.sleep(period)
                else:
                    sid, val = svc_cmd
                    last_seq = 1 - last_seq
                    pkt = packetB.encode(seq=last_seq,
                                        service_id=sid,
                                        value=val)
                    node.modem.send_packet(pkt)
                    node.get_logger().info(
                        f"Sent PacketB_Full svc_id={sid},value={val},seq={last_seq}"
                    )
                    start = time.time()
                    while (rclpy.ok() and not ack_ready.is_set() and
                        (time.time() - start) < svc_wait):
                        time.sleep(0.01)
                    svc_cmd = None

        else:  # mode == '40'
            if not packet40:
                node.get_logger().error(
                    "Packet40 unavailable, cannot operate in 40-bit mode. Exiting tx loop."
                )
                return

            while rclpy.ok() and not stop_event.is_set():
                if svc_cmd is not None:
                    sid, val = svc_cmd
                    # pass raw floats into Packet40
                    pkt = packet40.encode(
                        seq=last_seq,
                        vx=axes['x'],
                        vy=axes['y'],
                        vz=axes['z'],
                        svc_pending=1,
                        svc_id=sid,
                        svc_val=val
                    )
                    node.modem.send_packet(pkt)
                    node.get_logger().info(
                        f"Sent Packet40 pending svc_id={sid},value={val},seq={last_seq}"
                    )

                    # wait for ACK/NACK
                    ack_ready.clear()
                    start = time.time()
                    while (rclpy.ok() and not ack_ready.is_set() and
                        (time.time() - start) < svc_wait):
                        time.sleep(0.01)
                    node.get_logger().info(f"Stopped waiting")
                    svc_cmd = None
                    # toggle only the LSB for next seq
                    last_seq = last_seq ^ 0x1

                else:
                    # regular heartbeat with no pending service
                    pkt = packet40.encode(
                        seq=last_seq,
                        vx=axes['x'],
                        vy=axes['y'],
                        vz=axes['z'],
                        svc_pending=0,
                        svc_id=0,
                        svc_val=0
                    )
                    node.modem.send_packet(pkt)
                    node.get_logger().info(
                        f"Sent Packet40 seq={last_seq} (4 bytes): {packet40.to_hex_string(pkt)}"
                    )

                    # advance full 3-bit counter
                    last_seq = (last_seq + 1) & 0x07
                    time.sleep(1.0)


    # Start threads
    reader_thread = threading.Thread(target=reader_loop, daemon=True)
    tx_thread = threading.Thread(target=tx_loop, daemon=True)
    reader_thread.start()
    tx_thread.start()
    node.add_on_shutdown(lambda: stop_event.set())