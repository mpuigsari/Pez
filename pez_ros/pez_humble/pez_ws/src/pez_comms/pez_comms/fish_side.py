#!/usr/bin/env python3
"""
fish_side.py

PezComms “fish” node (using core library):
  - Runs a background thread that continuously pops bytes from modem_io.queue.
    • If byte is Packet A, decode (x_q,y_q,z_q), dequantize, publish /cmd_vel.
    • If byte is Packet B, decode (svc_id, value, seq), then:
         – if svc_id in {0,1,2}: try to call the corresponding fish_teleop Trigger service
             • bounded wait (2 sec) for the service call to complete 
             • if timeout or service never existed → send back a “negative ACK” (seq^1)
             • if the service callback DOES return → send back a normal echo (seq)
         – if svc_id == 3: publish a Float32 to /pez/camera_control immediately,
           then send back a normal echo (seq)
       After handling, send back exactly one Packet B–ACK, either “pos” or “neg.”
  - On shutdown, close modem_iomgr thread cleanly.
"""
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet
from pez_comms.core.modem_io import ModemIOMgr


class FishComms(Node):
    def __init__(self):
        super().__init__('pez_comms_fish')

        # ─────────────────────────────────────────────────────────────
        # 1) SERIAL PORT & MODEM I/O MANAGER
        # ─────────────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyModem0')
        port = self.get_parameter('port').get_parameter_value().string_value
        try:
            self.modem_io = ModemIOMgr(port=port, baud=9600, timeout=0.1)
            self.get_logger().info(f"Opened modem on port: {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open modem on {port}: {e}")
            raise

        # ─────────────────────────────────────────────────────────────
        # 2) PACKET DEFINITIONS
        # ─────────────────────────────────────────────────────────────
        # If these IDs were not registered, get_packet() will KeyError.
        self.packetA = get_packet("A_NORMAL")
        self.packetB = get_packet("B_COMMAND")

        # ─────────────────────────────────────────────────────────────
        # 3) ROS PUBLISHERS & SERVICE CLIENTS
        # ─────────────────────────────────────────────────────────────
        # Publisher: incoming Packet A → /pez/cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/pez/cmd_vel', 10)

        # Publisher: camera_control (Float32) if Packet B with svc_id=3
        self.camera_pub = self.create_publisher(Float32, '/pez/camera_control', 10)
        self.get_logger().info("Advertised /pez/camera_control (Float32)")

        # Service clients: fish_teleop for svc_id=0,1,2
        ns = self.get_namespace()
        self.start_cli   = self.create_client(Trigger, f'{ns}/teleoperation/start_swim')
        self.stop_cli    = self.create_client(Trigger, f'{ns}/teleoperation/stop_swim')
        self.magnet_cli  = self.create_client(Trigger, f'{ns}/teleoperation/toggle_magnet')
        self.neutral_cli = self.create_client(Trigger, f'{ns}/teleoperation/toggle_neutral')

        # ─────────────────────────────────────────────────────────────
        # 4) SERIAL‐READ LOOP THREAD
        # ─────────────────────────────────────────────────────────────
        self._stop_event = threading.Event()
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()
        self.get_logger().info('FishComms initialized (fish). Listening for packets.')


    def _serial_loop(self):
        """
        Continuously pop bytes from modem_io.get_byte(timeout). 
        If it’s Packet A: decode→dequantize→publish /cmd_vel.
        If it’s Packet B: decode→attempt service call (0/1/2) or camera pub (3),
        then echo back ACK (same seq) or NACK (seq^1) if the service failed/timed out.
        """
        while rclpy.ok() and not self._stop_event.is_set():
            raw = self.modem_io.get_byte(timeout=0.1)
            if raw is None:
                continue

            byte = raw[0]  # single‐byte packet
            if (byte & 0x80) == 0:  # → Packet A (bit7=0)
                fields = self.packetA.decode(raw)
                x_q = fields["x_q"]
                y_q = fields["y_q"]
                z_q = fields["z_q"]
                self.get_logger().info(f"Packet A received → x_q={x_q}, y_q={y_q}, z_q={z_q}")

                x = self._dequant(x_q, bits=3)
                y = self._dequant(y_q, bits=2)
                z = self._dequant(z_q, bits=2)
                twist = Twist()
                twist.linear.x = x
                twist.linear.y = y
                twist.linear.z = z
                self.get_logger().debug(
                    f"Publishing Twist → linear ({x:.3f}, {y:.3f}, {z:.3f})"
                )
                self.cmd_vel_pub.publish(twist)

            else:
                # → Packet B (bit7=1)
                fields = self.packetB.decode(raw)
                svc_id = fields["service_id"]
                val    = fields["value"]
                seq    = fields["seq"]
                self.get_logger().info(f"Packet B received → svc_id={svc_id}, value={val}, seq={seq}")

                # If svc_id ∈ {0,1,2} → try to call Trigger service with a bounded timeout
                if svc_id in (0, 1, 2):
                    client = self._select_client(svc_id, val)

                    got_positive_ack = False

                    # 1) Wait up to 5s for the service to appear
                    if client.wait_for_service(timeout_sec=5.0):
                        self.get_logger().info(
                            f"Service for svc_id={svc_id} is available; calling it now"
                        )

                        req = Trigger.Request()
                        future = client.call_async(req)
                        # 2) Wait at most 2 seconds for the service to actually return
                        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

                        if future.done():
                            # The service callback returned in time → positive ACK
                            got_positive_ack = True
                            self.get_logger().info(
                                f"Fish teleop service completed (svc_id={svc_id}, value={val})"
                            )
                        else:
                            # Timed out waiting for the service call to finish
                            self.get_logger().warn(
                                f"Fish teleop service timeout for svc_id={svc_id}, value={val}"
                            )
                    else:
                        # Service never became available within 5 seconds
                        self.get_logger().warn(
                            f"Fish teleop service unavailable (svc_id={svc_id}, value={val})"
                        )

                    # 3) Build ACK/NACK depending on whether the service succeeded
                    if got_positive_ack:
                        response_seq = seq      # echo the same seq → “positive ACK”
                    else:
                        response_seq = 1 - seq  # flip seq → “negative ACK”

                    ack_byte = self.packetB.encode(
                        service_id=svc_id, value=val, seq=response_seq
                    )
                    try:
                        self.modem_io.send_packet(ack_byte)
                        if got_positive_ack:
                            self.get_logger().info(
                                f"Sent POSITIVE‐ACK → svc_id={svc_id}, value={val}, seq={response_seq}, "
                                f"byte=0x{ack_byte.hex().upper()}"
                            )
                        else:
                            self.get_logger().info(
                                f"Sent NEGATIVE‐ACK → svc_id={svc_id}, value={val}, seq={response_seq}, "
                                f"byte=0x{ack_byte.hex().upper()}"
                            )
                        continue  # proceed back to reading next raw‐byte
                    except Exception as e:
                        self.get_logger().error(f"Failed to send ACK/NACK: {e}")
                        continue

                # ─────────────────────────────────────────────────────────────
                # If svc_id == 3 → camera via topic
                else:  
                    cam_val = 1.0 if val == 1 else -1.0
                    self.get_logger().info(f"Publishing camera_control → {cam_val:.1f}")
                    msg = Float32()
                    msg.data = cam_val
                    self.camera_pub.publish(msg)

                    # Always send a normal echo (positive ACK) for camera, since publishing a topic
                    # never “fails” in this node’s logic
                    ack_byte = self.packetB.encode(service_id=svc_id, value=val, seq=seq)
                    try:
                        self.modem_io.send_packet(ack_byte)
                        self.get_logger().info(
                            f"Sent POSITIVE‐ACK (camera) → svc_id={svc_id}, value={val}, seq={seq}, "
                            f"byte=0x{ack_byte.hex().upper()}"
                        )
                    except Exception as e:
                        self.get_logger().error(f"Failed to send camera ACK: {e}")
                    continue

        # Exit thread
        self.get_logger().info("Serial loop exiting.")


    def _select_client(self, svc_id: int, val: int):
        """Return the appropriate Trigger client for a given service ID and value."""
        if svc_id == 0:  # start vs. stop
            return self.start_cli if val == 1 else self.stop_cli
        if svc_id == 1:  # magnet on/off
            return self.magnet_cli
        # svc_id == 2 → neutral
        return self.neutral_cli


    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        """Map quantized integer back into [-1,1]"""
        levels = (1 << bits) - 1
        return (q / levels) * 2.0 - 1.0


    def destroy_node(self):
        self.get_logger().info("Shutting down FishComms; stopping serial thread.")
        self._stop_event.set()
        try:
            self.modem_io.close()
            self.get_logger().info("Serial port closed.")
        except Exception:
            pass
        self._serial_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FishComms()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
