#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from pez_comms.utility import PacketBuilder, ModemCommunicator


class FishComms(Node):
    def __init__(self):
        super().__init__('pez_comms_fish')

        # ─────────────────────────────────────────────────────────────
        # Serial modem communicator (same as host)
        # ─────────────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyModem0')
        port = self.get_parameter('port').get_parameter_value().string_value
        try:
            self.modem = ModemCommunicator(port=port, baud=9600)
            self.get_logger().info(f"Opened modem on port: {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open modem on {port}: {e}")
            raise
        
        ns = self.get_namespace()

        # ─────────────────────────────────────────────────────────────
        # Publisher: Received Packet A → /pez/cmd_vel
        # ─────────────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)

        # ─────────────────────────────────────────────────────────────
        # Publisher: camera_control (Float32)
        # ─────────────────────────────────────────────────────────────
        self.camera_pub = self.create_publisher(Float32, f'{ns}/camera_control', 10)

        # ─────────────────────────────────────────────────────────────
        # Service clients for fish_teleop (Trigger services) for svc_id = 0, 1, 2
        # ─────────────────────────────────────────────────────────────
        
        self.start_cli   = self.create_client(Trigger, f'{ns}/teleoperation/start_swim')
        self.stop_cli    = self.create_client(Trigger, f'{ns}/teleoperation/stop_swim')
        self.magnet_cli  = self.create_client(Trigger, f'{ns}/teleoperation/toggle_magnet')
        self.neutral_cli = self.create_client(Trigger, f'{ns}/teleoperation/toggle_neutral')
        # Note: No cam_left_cli or cam_right_cli—camera is driven via topic now.

        # ─────────────────────────────────────────────────────────────
        # Start background thread to read from modem
        # ─────────────────────────────────────────────────────────────
        threading.Thread(target=self._serial_loop, daemon=True).start()
        self.get_logger().info('FishComms initialized and listening for packets.')

    def _serial_loop(self):
        """
        Continuously read one byte at a time. 
        - Packet A (MSB=0): decode → publish cmd_vel.
        - Packet B (MSB=1): decode → 
            • if svc_id in {0,1,2} → call corresponding teleop service
            • if svc_id == 3      → publish Float32 on /pez/camera_control
          Then echo back ACK.
        """
        while rclpy.ok():
            try:
                data = self.modem.read_bytes(1)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

            if not data:
                continue

            byte = data[0]
            # Log raw byte received (hex)
            self.get_logger().debug(f"Raw byte received: 0x{byte:02X}")

            # ─────────────────────────────────────────────────────────
            # Packet A (MSB = 0)
            # ─────────────────────────────────────────────────────────
            if (byte & 0x80) == 0:
                x_q, y_q, z_q = PacketBuilder.parse_packet_a_normal(byte)
                self.get_logger().info(f"Packet A received → x_q={x_q}, y_q={y_q}, z_q={z_q}")

                # de-quantize back into [-1,1]
                x = self._dequant(x_q, bits=3)
                y = self._dequant(y_q, bits=2)
                z = self._dequant(z_q, bits=2)
                twist = Twist()
                twist.linear.x = x
                twist.linear.y = y
                twist.linear.z = z

                # Log de-quantized Twist
                self.get_logger().info(
                    f"Publishing Twist → linear: ({x:.3f}, {y:.3f}, {z:.3f}), angular: (0.0, 0.0, 0.0)"
                )
                self.cmd_vel_pub.publish(twist)

            # ─────────────────────────────────────────────────────────
            # Packet B (MSB = 1)
            # ─────────────────────────────────────────────────────────
            else:
                svc_id, val, seq = PacketBuilder.parse_packet_b(byte)
                self.get_logger().info(f"Packet B received → svc_id={svc_id}, value={val}, seq={seq}")

                # If svc_id in {0,1,2} → call the appropriate Trigger service
                if svc_id in (0, 1, 2):
                    client = self._select_client(svc_id, val)
                    if client.wait_for_service(timeout_sec=5.0):
                        self.get_logger().info(f"Calling teleop service for svc_id={svc_id}, value={val}")
                        req = Trigger.Request()
                        future = client.call_async(req)
                        rclpy.spin_until_future_complete(self, future)
                        self.get_logger().info(f"Teleop service completed for svc_id={svc_id}, value={val}")
                    else:
                        self.get_logger().warn(
                            f"Service for svc_id={svc_id}, value={val} not available within 5 seconds."
                        )

                # If svc_id == 3 → camera control via topic (Float32)
                else:  # svc_id == 3
                    cam_val = 1.0 if val == 1 else -1.0
                    self.get_logger().info(f"Publishing camera_control → {cam_val:.1f}")
                    msg = Float32()
                    msg.data = cam_val
                    self.camera_pub.publish(msg)

                # Echo back as ACK using same fields
                ack = PacketBuilder.build_packet_b(svc_id, val, seq)
                try:
                    self.modem.send_packet(ack)
                    self.get_logger().info(
                        f"Sent ACK → svc_id={svc_id}, value={val}, seq={seq}, byte=0x{ack.hex().upper()}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed to send ACK: {e}")

    def _dequant(self, q: int, bits: int) -> float:
        """Map quantized integer back into [-1,1]"""
        levels = (1 << bits) - 1
        return (q / levels) * 2.0 - 1.0

    def _select_client(self, svc_id: int, val: int):
        """
        Return the appropriate Trigger client for svc_id in {0,1,2}:
          - svc_id=0: start_swim if val==1, otherwise stop_swim
          - svc_id=1: toggle_magnet
          - svc_id=2: toggle_neutral
        """
        if svc_id == 0:
            return self.start_cli if (val == 1) else self.stop_cli
        if svc_id == 1:
            return self.magnet_cli
        # svc_id == 2
        return self.neutral_cli
        
    # ────────────────────────────────────────────────────────────────────────────
    # CLEANUP on shutdown
    # ────────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down FishComms; stopping serial thread and closing modem.")

        # Tell the serial thread to stop
        self._serial_thread_stop.set()

        # Close the serial port so read_bytes(1) unblocks
        try:
            self.modem.close()
            self.get_logger().info("Serial port closed.")
        except Exception:
            pass

        # Wait briefly for the thread to notice the stop event
        self._serial_thread.join(timeout=1.0)

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FishComms()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User hit Ctrl-C: exit spin loop
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
