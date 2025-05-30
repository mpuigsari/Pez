#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Trigger
from packet_comms import PacketBuilder, ModemCommunicator


class FishComms(Node):
    def __init__(self):
        super().__init__('pez_comms_fish')
        # Serial modem communicator (same as host)
        
        self.declare_parameter('port', '/dev/ttyModem0')
        port = self.get_parameter('port').get_parameter_value().string_value
        self.modem = ModemCommunicator(port=port, baud=9600)

        # Publisher for received Packet A → cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/pez/cmd_vel', 10)

        # Service clients for fish_teleop (Trigger services)
        ns = self.get_namespace()
        self.start_cli    = self.create_client(Trigger, f'{ns}/teleoperation/start_swim')
        self.stop_cli     = self.create_client(Trigger, f'{ns}/teleoperation/stop_swim')
        self.magnet_cli   = self.create_client(Trigger, f'{ns}/teleoperation/toggle_magnet')
        self.neutral_cli  = self.create_client(Trigger, f'{ns}/teleoperation/toggle_neutral')
        self.cam_left_cli = self.create_client(Trigger, f'{ns}/teleoperation/cam_left')
        self.cam_right_cli= self.create_client(Trigger, f'{ns}/teleoperation/cam_right')

        # Start background thread to read from modem
        threading.Thread(target=self._serial_loop, daemon=True).start()

    def _serial_loop(self):
        """
        Continuously read one byte at a time. Packet A: decode → publish cmd_vel.
        Packet B: decode → call teleop service → echo back ACK.
        """
        while rclpy.ok():
            data = self.modem.read_bytes(1)
            if not data:
                continue
            byte = data[0]

            # Packet A (MSB = 0)
            if byte & 0x80 == 0:
                # parse quantized fields
                x_q, y_q, z_q = PacketBuilder.parse_packet_a_normal(byte)
                # de-quantize back into [-1,1]
                twist = Twist()
                twist.linear.x = self._dequant(x_q, bits=3)
                twist.linear.y = self._dequant(y_q, bits=3)
                twist.linear.z = self._dequant(z_q, bits=2)
                self.cmd_vel_pub.publish(twist)

            # Packet B (service request, MSB = 1)
            else:
                svc_id, val, seq = PacketBuilder.parse_packet_b(byte)
                client = self._select_client(svc_id, val)
                # ensure service is available
                if client.wait_for_service(timeout_sec=5.0):
                    req = Trigger.Request()
                    # synchronous call: block until done
                    future = client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                # echo back as ACK using same fields fileciteturn1file1
                ack = PacketBuilder.build_packet_b(svc_id, val, seq)
                self.modem.send_packet(ack)

    def _dequant(self, q: int, bits: int) -> float:
        """Map quantized integer back into [-1,1]"""
        levels = (1 << bits) - 1
        return (q / levels) * 2.0 - 1.0

    def _select_client(self, svc_id: int, val: int) -> rclpy.client.Client:
        """Return the appropriate Trigger client for a given service ID and value."""
        if svc_id == 0:
            return self.start_cli if val == 1 else self.stop_cli
        if svc_id == 1:
            return self.magnet_cli
        if svc_id == 2:
            return self.neutral_cli
        # svc_id == 3: camera control → cam_left or cam_right
        return self.cam_right_cli if val == 1 else self.cam_left_cli


def main(args=None):
    rclpy.init(args=args)
    node = FishComms()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
