#!/usr/bin/env python3
"""
host_side.py

PezComms Master (refactored to use pez_comms/core as a generic library):
  - Streams Packet A Normal continuously @ 4 Hz (via loop_A step).
  - When a teleop service or camera topic request arrives, loop_A returns,
    then send exactly one Packet B Command, wait for its ACK/NACK echo, then clear, then resume loop_A.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet, list_packet_ids
from pez_comms.core.modem_io import ModemIOMgr
from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler


class PezComms(Node):
    def __init__(self):
        super().__init__('host_comms')

        # ─────────────────────────────────────────────────────────────
        # 1) SERIAL PORT & MODEM I/O MANAGER
        # ─────────────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyModem0')
        port = self.get_parameter('port').get_parameter_value().string_value

        try:
            # ModemIOMgr spins up a background thread that pushes incoming bytes into a queue
            self.modem_io = ModemIOMgr(port=port, baud=9600, timeout=0.1)
            self.get_logger().info(f"Opened modem on port: {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open modem on {port}: {e}")
            raise

        # ─────────────────────────────────────────────────────────────
        # 2) PACKET REGISTRATION
        # ─────────────────────────────────────────────────────────────
        packets = list_packet_ids()
        if "A_NORMAL" not in packets or "B_COMMAND" not in packets:
            raise RuntimeError("Built‐in packets not registered! Found: " + ", ".join(packets))

        self.packetA = get_packet("A_NORMAL")   # PacketDefinition for Packet A
        self.packetB = get_packet("B_COMMAND")  # PacketDefinition for Packet B

        # ─────────────────────────────────────────────────────────────
        # 3) INTERNAL STATE
        # ─────────────────────────────────────────────────────────────
        # Latest /cmd_vel axes (floats in [-1, +1])
        self._axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # When a teleop or camera topic comes in, store (service_id, value) here:
        # The scheduler’s “await request” step will pick that up and move on to send_B.
        self._service_cmd = None

        # Last sequence bit used for Packet B (toggles 0↔1 each time we send a B)
        self._last_seq = 0

        # These four track the most‐recent echo:
        #   _ack_seq:      the sequence bit that the fish echoed back
        #   _ack_got:      True as soon as ANY B‐echo arrives (positive or negative)
        #   _last_ack_success: True if echo_seq == self._last_seq (positive‐ACK), False if echo_seq != self._last_seq (NACK)
        #   _ack_ready_for_handler: separate flag that the service‐handler waits on (set exactly once when echo arrives).
        self._ack_seq = None
        self._ack_got = False
        self._last_ack_success = False
        self._ack_ready_for_handler = False

        # ─────────────────────────────────────────────────────────────
        # 4) ROS SUBSCRIPTIONS & SERVICES
        # ─────────────────────────────────────────────────────────────
        ns = self.get_namespace()

        # Subscribe to /cmd_vel → update self._axes
        self.create_subscription(
            Twist,
            f'{ns}/cmd_vel',
            self._vel_callback,
            10
        )

        # Factory to build each teleop service handler in a compact way:
        def make_teleop_handler(svc_id: int, service_val: int):
            def _handler(request, response):
                # Clear any old echo flags so we wait fresh for this new request
                self._ack_got = False
                self._ack_ready_for_handler = False
                self._last_ack_success = False

                # If another Packet B is already in progress, reject:
                if self._service_cmd is not None:
                    response.success = False
                    response.message = 'busy'
                    return response

                # Latch this as a Packet B request and spin until we see the echo:
                self._service_cmd = (svc_id, service_val)
                self.get_logger().info(
                    f"Queued Packet B → svc_id={svc_id}, value={service_val}; waiting for echo…"
                )
                while rclpy.ok() and not self._ack_ready_for_handler:
                    time.sleep(0.01)

                # By now, an echo arrived – check if it was positive or negative:
                if self._last_ack_success:
                    response.success = True
                    response.message = 'ok'
                else:
                    response.success = False
                    response.message = 'service-failure'
                return response

            return _handler

        # Create the four teleop services (start/stop, magnet, neutral)
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/start_swim',
            make_teleop_handler(0, 1)
        )
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/stop_swim',
            make_teleop_handler(0, 0)
        )
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/toggle_magnet',
            make_teleop_handler(1, 1)
        )
        self.create_service(
            Trigger,
            f'{ns}/teleoperation/toggle_neutral',
            make_teleop_handler(2, 1)
        )

        # Subscribe to /camera_control (Float32): positive→camera right, negative→camera left
        self.create_subscription(
            Float32,
            f'{ns}/camera_control',
            self._camera_callback,
            10
        )
        self.get_logger().info("Subscribed to /camera_control for camera-left/right commands")

        # ─────────────────────────────────────────────────────────────
        # 5) TRANSMISSION SCHEDULER: four steps
        # ─────────────────────────────────────────────────────────────
        steps = [
            # 1) loop_A sends Packet A at 4 Hz until _service_cmd != None
            TransmissionStep('loop_A',    self._loop_A_until_request),

            # 2) send_B builds and sends exactly one Packet B
            TransmissionStep('send_B',    self._send_packet_b, duration=0.25),

            # 3) wait_ack spins until ANY Packet B echo arrives
            TransmissionStep('wait_ack',  None,                 wait_for=self._wait_for_ack),

            # 4) clear resets _service_cmd so we go back to Packet A streaming
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
        Save the latest Twist into self._axes so Packet A can be rebuilt.
        """
        self._axes['x'] = msg.linear.x
        self._axes['y'] = msg.linear.y
        self._axes['z'] = msg.linear.z

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
          - if msg.data ≥ 0 → “camera right”  (svc_id=3, value=1)
          - if msg.data <  0 → “camera left”   (svc_id=3, value=0)

        Latch a new Packet B and then spin until we see the echo.  Once the echo arrives,
        log positive vs negative ACK.  Meanwhile the scheduler will run send_B → wait_ack → clear.
        """
        val = msg.data

        if self._service_cmd is not None:
            self.get_logger().warn(
                f"Camera command {val:.3f} ignored: another Packet B already in progress."
            )
            return

        if val >= 0.0:
            service_id = 3
            service_val = 1
            self.get_logger().info(f"Camera-control topic → right (val={val:.3f})")
        else:
            service_id = 3
            service_val = 0
            self.get_logger().info(f"Camera-control topic → left  (val={val:.3f})")

        # Clear old echo flags so we truly wait for the new one:
        self._ack_got = False
        self._ack_ready_for_handler = False
        self._last_ack_success = False

        # Latch a new Packet B so that the scheduler’s loop_A breaks out:
        self._service_cmd = (service_id, service_val)
        self.get_logger().info(
            f"Queued Packet B for camera (svc_id={service_id}, value={service_val}); waiting for echo…"
        )

        # Spin until wait_for_ack sets _ack_ready_for_handler:
        while rclpy.ok() and not self._ack_ready_for_handler:
            time.sleep(0.01)

        # Log whether positive or negative:
        if self._last_ack_success:
            self.get_logger().info(
                f"Camera-control received POSITIVE ACK "
                f"(svc_id={service_id}, value={service_val}, seq={self._ack_seq})"
            )
        else:
            self.get_logger().warn(
                f"Camera-control received NEGATIVE ACK "
                f"(svc_id={service_id}, value={service_val}, seq={self._ack_seq})"
            )


    # ────────────────────────────────────────────────────────────────────────────
    #  _loop_A_until_request
    # ────────────────────────────────────────────────────────────────────────────
    def _loop_A_until_request(self):
        """
        Continuously send Packet A Normal @ 4 Hz until self._service_cmd becomes non-None.
        Once _service_cmd is set, return so the scheduler moves on to send_B.
        """
        while self._service_cmd is None and rclpy.ok():
            x_q = self._quantize(self._axes['x'], bits=3)
            y_q = self._quantize(self._axes['y'], bits=2)
            z_q = self._quantize(self._axes['z'], bits=2)

            rawA = self.packetA.encode(x=x_q, y=y_q, z=z_q)
            try:
                self.modem_io.send_packet(rawA)
                self.get_logger().info(
                    f"Sent Packet A → x_q={x_q}, y_q={y_q}, z_q={z_q}, byte=0x{rawA.hex().upper()}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send Packet A: {e}")
                break

            # Sleep 0.25 s to get ~4 Hz
            time.sleep(0.25)

        # As soon as _service_cmd is no longer None, exit so the scheduler goes to send_B
        return


    # ────────────────────────────────────────────────────────────────────────────
    #  _send_packet_b
    # ────────────────────────────────────────────────────────────────────────────
    def _send_packet_b(self):
        """
        Build and send exactly one Packet B Command (service_id, value, seq).
        Then return.  The scheduler’s next step is wait_for_ack().
        """
        if self._service_cmd is None:
            return  # shouldn’t happen

        service_id, value = self._service_cmd
        self._last_seq ^= 1
        rawB = self.packetB.encode(
            service_id=service_id,
            value=value,
            seq=self._last_seq
        )
        try:
            self.modem_io.send_packet(rawB)
            self.get_logger().info(
                f"Sent Packet B → svc_id={service_id}, value={value}, seq={self._last_seq}, "
                f"byte=0x{rawB.hex().upper()}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send Packet B: {e}")
            # Even if sending fails, wait_for_ack() might still catch an echo if the fish echoed it.


    # ────────────────────────────────────────────────────────────────────────────
    #  _wait_for_ack
    # ────────────────────────────────────────────────────────────────────────────
    def _wait_for_ack(self) -> bool:
        """
        Blocks until the fish echoes *any* Packet B.  As soon as one arrives,
        set _last_ack_success = True/False (depending on seq), mark both
        _ack_got=True and _ack_ready_for_handler=True, then return True so the
        scheduler’s next step (clear) can run.
        """
        self.get_logger().info("Waiting for ACK/NACK…")
        while rclpy.ok():
            raw = self.modem_io.get_byte(timeout=0.1)
            if raw is None:
                continue

            fields   = self.packetB.decode(raw)
            echo_seq = fields.get("seq")

            if echo_seq == self._last_seq:
                # POSITIVE ACK
                self._last_ack_success = True
                self._ack_seq          = echo_seq
                self._ack_got          = True
                self._ack_ready_for_handler = True
                self.get_logger().info(
                    f"Received POSITIVE ACK → raw_byte=0x{raw.hex().upper()}, seq={echo_seq}"
                )
            else:
                # NEGATIVE ACK (sequence bit flipped)
                self._last_ack_success = False
                self._ack_seq          = echo_seq
                self._ack_got          = True
                self._ack_ready_for_handler = True
                self.get_logger().info(
                    f"Received NEGATIVE ACK → raw_byte=0x{raw.hex().upper()}, seq={echo_seq}"
                )

            # Return True so TransmissionScheduler moves on to clear()
            return True

        return False  # if ROS is shutting down


    # ────────────────────────────────────────────────────────────────────────────
    #  _clear_cmd
    # ────────────────────────────────────────────────────────────────────────────
    def _clear_cmd(self):
        """
        Reset the latched Packet B command (self._service_cmd) and _ack_got flag,
        so that loop_A can resume.  We intentionally do NOT clear
        _ack_ready_for_handler here—only clear _ack_got so next cycle can catch a new echo.
        """
        self.get_logger().info("Clearing service command and resetting ACK buffer.")
        self._service_cmd = None
        self._ack_got     = False
        self._ack_seq     = None
        # Note: _ack_ready_for_handler stays True until the handler itself clears it.


    # ────────────────────────────────────────────────────────────────────────────
    #  HELPER: quantize
    # ────────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _quantize(val: float, bits: int) -> int:
        """
        Convert a float in [−1, +1] to an integer in [0 .. (2^bits − 1)].
        """
        levels = (1 << bits)
        q = round((val + 1) / 2 * (levels - 1))
        return max(0, min(levels - 1, q))


    # ────────────────────────────────────────────────────────────────────────────
    #  CLEANUP: stop scheduler + close modem
    # ────────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down PezComms: stopping scheduler.")
        self._scheduler.stop()
        self._scheduler.join(timeout=1.0)

        self.get_logger().info("Closing modem I/O.")
        try:
            self.modem_io.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PezComms()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
