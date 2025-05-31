#!/usr/bin/env python3
"""
host_side.py

PezComms Master:
  - Streams cmd_vel (Packet A) continuously at 4 Hz.
  - Sends teleop requests (Packet B) on demand (start/stop, magnet, neutral via services; camera via topic),
    then waits for an ACK echo.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.utility import (
    TransmissionStep,
    TransmissionScheduler,
    PacketBuilder,
    ModemCommunicator
)


class PezComms(Node):
    def __init__(self):
        super().__init__('host_comms')

        # ─────────────────────────────────────────────────────────────
        # 1) SERIAL PORT SETUP
        # ─────────────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyModem0')
        port = self.get_parameter('port').get_parameter_value().string_value

        try:
            self.modem = ModemCommunicator(port=port, baud=9600)
            self.get_logger().info(f"Opened modem on port: {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open modem on {port}: {e}")
            raise

        # ─────────────────────────────────────────────────────────────
        # 2) INTERNAL STATE
        # ─────────────────────────────────────────────────────────────
        # Latest /cmd_vel components
        self._axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        # If a teleop (Packet B) is requested, we store (service_id, value) here:
        self._service_cmd = None
        # Sequence bit toggles each Packet B
        self._last_seq = 0
        # ACK‐tracking (did we get the echo for our last Packet B?)
        self._ack_got = False
        self._ack_seq = None

        # ─────────────────────────────────────────────────────────────
        # 3) ROS SUBSCRIPTIONS & SERVICES
        # ─────────────────────────────────────────────────────────────
        ns = self.get_namespace()

        # Subscribe to /cmd_vel to update self._axes
        self.create_subscription(
            Twist,
            f'{ns}/cmd_vel',
            self._vel_callback,
            10
        )

        # Create four Trigger‐services for start/stop, magnet, neutral
        # Camera commands will come via /pez/camera_control topic instead.
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/start_swim',
            self._handle_start
        )
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/stop_swim',
            self._handle_stop
        )
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/toggle_magnet',
            self._handle_magnet
        )
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/toggle_neutral',
            self._handle_neutral
        )

        # Subscribe to /pez/camera_control (Float32). Positive→camera right, Negative→camera left
        self.create_subscription(
            Float32,
            f'{ns}/camera_control',
            self._camera_callback,
            10
        )
        self.get_logger().info("Subscribed to /camera_control for camera‐left/right commands")

        # ─────────────────────────────────────────────────────────────
        # 4) TRANSMISSION SCHEDULER
        # ─────────────────────────────────────────────────────────────
        # This scheduler runs in a loop:
        #   1) “loop_A”    → send Packet A
        #   2) “await_srv” → wait until _service_cmd != None
        #   3) “send_B”    → send Packet B
        #   4) “wait_ack”  → block until ACK‐echo comes back
        #   5) “clear”     → reset _service_cmd and ACK flags
        steps = [
            TransmissionStep('loop_A',    self._loop_A_until_request),
            TransmissionStep('send_B',    self._send_packet_b, duration=0.25),
            TransmissionStep('wait_ack',  None,           wait_for=self._wait_for_ack),
            TransmissionStep('clear',     self._clear_cmd),
        ]
        self._scheduler = TransmissionScheduler(steps, loop=True)
        self._scheduler.start()

        self.get_logger().info('PezComms initialized (host). TransmissionScheduler started.')


    # ────────────────────────────────────────────────────────────────────────────
    #  /cmd_vel CALLBACK
    # ────────────────────────────────────────────────────────────────────────────
    def _vel_callback(self, msg: Twist):
        """
        Save the latest Twist into self._axes so Packet A can be built from it.
        """
        self._axes['x'] = msg.linear.x
        self._axes['y'] = msg.linear.y
        self._axes['z'] = msg.linear.z

        # Debug‐level log so you can monitor raw inputs if needed
        self.get_logger().debug(
            f"Updated axes from /cmd_vel → x={msg.linear.x:.3f}, "
            f"y={msg.linear.y:.3f}, z={msg.linear.z:.3f}"
        )


    # ────────────────────────────────────────────────────────────────────────────
    #  CAMERA TOPIC CALLBACK
    # ────────────────────────────────────────────────────────────────────────────
    def _camera_callback(self, msg: Float32):
        """
        When /camera_control Float32 arrives:
          - if msg.data ≥ 0 → “camera right” (svc_id=3, value=1)
          - if msg.data <  0 → “camera left”  (svc_id=3, value=0)

        This essentially wraps the same logic as the old cam_left/cam_right services:
        set self._service_cmd and wait for ACK.
        """
        val = msg.data
        if self._service_cmd is not None:
            # Already busy sending another Packet B; ignore or warn
            self.get_logger().warn(
                f"Camera command {val:.3f} ignored: another Packet B already in progress."
            )
            return

        if val >= 0.0:
            service_id = 3
            service_val = 1  # camera right
            self.get_logger().info(f"Camera‐control topic → right (val={val:.3f})")
        else:
            service_id = 3
            service_val = 0  # camera left
            self.get_logger().info(f"Camera‐control topic → left  (val={val:.3f})")

        # Latch the packet the same way a service would
        self._service_cmd = (service_id, service_val)
        self.get_logger().info(f"Queued Packet B for camera (svc_id={service_id}, value={service_val}); awaiting ACK")

        # Wait for ACK in this callback (blocking). If you prefer non-blocking,
        # you can remove this spin loop and let the scheduler’s “wait_for_ack” handle it.
        while not self._ack_got:
            time.sleep(0.01)

        # Once ACK is received:
        self.get_logger().info(
            f"Camera ACK matched (svc_id={service_id}, value={service_val}, seq={self._ack_seq})"
        )


    # ────────────────────────────────────────────────────────────────────────────
    #  PACKET A: ALWAYS STREAM AT 4 HZ
    # ────────────────────────────────────────────────────────────────────────────
    def _loop_A_until_request(self):
        """
        Continuously send Packet A at 4 Hz (0.25 s intervals) until self._service_cmd != None.

        This method is used as the “action” for TransmissionStep('loop_A', ...). As soon as
        self._service_cmd becomes non‐None, this method returns, allowing the scheduler to
        move on to “send_B” → “wait_ack” → “clear” → loop back to loop_A, etc.
        """
        # Keep sending Packet A @ 4 Hz until a B‐request arrives
        while self._service_cmd is None and rclpy.ok():
            # Build and send Packet A
            x_q = self._quantize(self._axes['x'], bits=3)
            y_q = self._quantize(self._axes['y'], bits=2)
            z_q = self._quantize(self._axes['z'], bits=2)

            pkt = PacketBuilder.build_packet_a_normal(x_q, y_q, z_q)
            try:
                self.modem.send_packet(pkt)
                self.get_logger().info(
                    f"Sent Packet A → x_q={x_q}, y_q={y_q}, z_q={z_q}, byte=0x{pkt.hex().upper()}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send Packet A: {e}")
                # If serial fails, break out completely
                break

            # Sleep 0.25 s before sending next Packet A
            time.sleep(0.25)

        # Once we exit this loop, either:
        #   • self._service_cmd is no longer None, OR
        #   • ROS is shutting down, OR
        #   • serial failure occurred.
        # In all cases, return—scheduler will proceed to the next step.
        return


    # ────────────────────────────────────────────────────────────────────────────
    #  PACKET B: SEND ONCE WHEN A TELEOP SERVICE IS CALLED, THEN WAIT FOR ACK
    # ────────────────────────────────────────────────────────────────────────────
    def _send_packet_b(self):
        """
        Build and send Packet B from self._service_cmd=(service_id,value).
        Toggles the sequence‐bit each time. Logs the fields and raw hex‐byte.
        """
        service_id, value = self._service_cmd
        self._last_seq ^= 1  # flip 0 <-> 1
        pkt = PacketBuilder.build_packet_b(service_id, value, self._last_seq)

        try:
            self.modem.send_packet(pkt)
            self.get_logger().info(
                f"Sent Packet B → svc_id={service_id}, value={value}, "
                f"seq={self._last_seq}, byte=0x{pkt.hex().upper()}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send Packet B: {e}")


    def _wait_for_ack(self) -> bool:
        """
        Blocks until an ACK (Packet B echo) arrives. Returns True if we see it,
        False if a serial error occurs. Logs both “Waiting for ACK…” and the raw ACK when received.
        """
        self.get_logger().info("Waiting for ACK...")
        while True:
            try:
                data = self.modem.read_bytes(1)
            except Exception as e:
                self.get_logger().error(f"Serial read error while waiting for ACK: {e}")
                return False

            if not data:
                continue

            byte = data[0]
            seq = byte & 0x1
            self._ack_got = True
            self._ack_seq = seq
            self.get_logger().info(f"Received ACK → raw_byte=0x{byte:02X}, seq={seq}")
            return True


    def _clear_cmd(self):
        """
        Called after Packet B + ACK have both happened. Clears out the latched
        teleop command and resets the ACK flags.
        """
        self.get_logger().info("Clearing service command and resetting ACK buffer.")
        self._service_cmd = None
        self._ack_got = False
        self._ack_seq = None


    # ────────────────────────────────────────────────────────────────────────────
    #  GENERIC TELEOP SERVICE HANDLER (start/stop, magnet, neutral)
    # ────────────────────────────────────────────────────────────────────────────
    def _svc_handler(self, service_id: int, value: int, request, response):
        """
        Common wrapper for the Trigger services:
          1) If already busy (_service_cmd != None), reject with busy.
          2) Otherwise, latch self._service_cmd=(service_id,value).
          3) Block until _wait_for_ack() logs the incoming ACK.
          4) Return success=True.
        """
        self.get_logger().info(f"Service request → svc_id={service_id}, value={value}")
        if self._service_cmd is not None:
            response.success = False
            response.message = 'busy'
            self.get_logger().warn("Rejecting service request: already processing another command.")
            return response

        # 1) Latch the command so scheduler’s “await_srv” step can see it
        self._service_cmd = (service_id, value)
        self.get_logger().info(f"Queued Packet B → svc_id={service_id}, value={value}. Awaiting ACK.")

        # 2) Block until ACK arrives (logged inside _wait_for_ack())
        while not self._ack_got:
            time.sleep(0.01)

        # 3) Once ACK arrives, log the sequence:
        self.get_logger().info(
            f"ACK matched for svc_id={service_id}, value={value}, seq={self._ack_seq}"
        )

        response.success = True
        response.message = 'ok'
        return response


    # ────────────────────────────────────────────────────────────────────────────
    #  FOUR TELEOP SERVICE CALLBACKS (start/stop, magnet, neutral)
    # ────────────────────────────────────────────────────────────────────────────
    def _handle_start(self, request, response):
        return self._svc_handler(0, 1, request, response)

    def _handle_stop(self, request, response):
        return self._svc_handler(0, 0, request, response)

    def _handle_magnet(self, request, response):
        return self._svc_handler(1, 1, request, response)

    def _handle_neutral(self, request, response):
        return self._svc_handler(2, 1, request, response)


    # ────────────────────────────────────────────────────────────────────────────
    #  HELPER: QUANTIZATION
    # ────────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _quantize(val: float, bits: int) -> int:
        """
        Convert a float in [-1, +1] into an integer in [0 .. (2^bits - 1)].
        """
        levels = (1 << bits)
        q = round((val + 1) / 2 * (levels - 1))
        return max(0, min(levels - 1, q))


    # ────────────────────────────────────────────────────────────────────────────
    #  CLEANUP (stop the scheduler thread cleanly)
    # ────────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        # Tell the scheduler thread to stop
        self.get_logger().info("Shutting down PezComms and stopping scheduler.")
        self._scheduler.stop()

        # Wait a small amount for the scheduler thread to exit cleanly
        # (TransmissionScheduler.stop() sets an Event; it checks that in its loop)
        self._scheduler.join(timeout=2.0)

        # Optionally close the serial port
        try:
            self.modem.close()
            self.get_logger().info("Serial port closed.")
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PezComms()
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
