import serial
import time
import struct
from threading import Thread, Event

# CRC utility
# _crc_generic: computes a width-bit CRC over the integer data using given polynomial
#   data: integer whose lower bits contain the fields to protect
#   width: number of CRC bits
#   poly: CRC polynomial, including top bit (e.g. for CRC-3 poly=0b1011)
def _crc_generic(data: int, width: int, poly: int) -> int:
    mask = (1 << width) - 1
    data = data << width
    # total bit‐length including appended zeros
    total = data.bit_length()
    # process from highest down to the CRC width
    for shift in range(total - width - 1, -1, -1):
        if data & (1 << (shift + width)):
            data ^= (poly << shift)
    return data & mask


# CRC polynomials
_CRC2_POLY = 0b111    # CRC-2: x^2 + x + 1
_CRC3_POLY = 0b1011   # CRC-3: x^3 + x + 1
_CRC6_POLY = 0b100111 # CRC-6-ITU: x^6 + x + x^2 + x + 1

class PacketBuilder:
    """Builds Packet A and Packet B for acoustic modem commands."""

    @staticmethod
    def build_packet_b(service_id: int, value: int, seq: int) -> bytes:
        """
        Packet B format (8 bits):
          bit7        = Type B (1)
          bits6-5     = service_id (2 bits)
                       00=start/stop, 01=magnet, 10=neutral, 11=cam
          bit4        = value (1 bit)
                       For start/stop: 0=stop,1=start
                       For magnet/neutral: 0=off,1=on
                       For cam: 0=-1,1=+1
          bits3-1     = CRC-3 over bits[7:4]
          bit0        = seq (1 bit) alternating ACK/toggle
        """
        header = (1 << 7) | ((service_id & 0x3) << 5) | ((value & 0x1) << 4)
        crc3 = _crc_generic(header >> 4, width=3, poly=_CRC3_POLY)
        pkt = header | ((crc3 & 0x7) << 1) | (seq & 0x1)
        return struct.pack('B', pkt)

    @staticmethod
    def build_packet_a_simple(axis_template: int, x: int, yz: int) -> bytes:
        """
        Packet A Simple + CRC-2 (8 bits):
          bit7        = Type A (0)
          bit6        = axis_template (1 bit): 0=xy,1=xz
          bits5-4     = x (2 bits)
          bits3-2     = y or z (2 bits)
          bits1-0     = CRC-2 over bits[7:2]
        """
        header = ((axis_template & 0x1) << 6) | ((x & 0x3) << 4) | ((yz & 0x3) << 2)
        crc2 = _crc_generic(header >> 2, width=2, poly=_CRC2_POLY)
        pkt = header | (crc2 & 0x3)
        return struct.pack('B', pkt)

    @staticmethod
    def build_packet_a_normal(x: int, y: int, z: int) -> bytes:
        """
        Packet A Normal (8 bits):
          bit7        = Type A (0)
          bits6-4     = x (3 bits)
          bits3-2     = y (2 bits)
          bit1-0        = z (2 bits)
        """
        pkt = ((x & 0x7) << 4) | ((y & 0x3) << 2) | (z & 0x3)
        return struct.pack('B', pkt)

    @staticmethod
    def build_packet_a_crc6(x: int, y: int, z: int) -> bytes:
        """
        Packet A + CRC-6 (16 bits):
          bit15       = Type A (0)
          bits14-12   = x (3 bits)
          bits11-9    = y (3 bits)
          bits8-6     = z (3 bits)
          bits5-0     = CRC-6 over bits[15:6]
        """
        header = ((x & 0x7) << 12) | ((y & 0x7) << 9) | ((z & 0x7) << 6)
        crc6 = _crc_generic(header >> 6, width=6, poly=_CRC6_POLY)
        pkt = (header | (crc6 & 0x3F)) & 0xFFFF
        return struct.pack('>H', pkt)

    # ───────────────────────────────────────────────────────────────────────────
    # NEW: parse_packet_a_normal
    # ───────────────────────────────────────────────────────────────────────────
    @staticmethod
    def parse_packet_a_normal(byte_val: int):
        """
        Reverse of build_packet_a_normal (8 bits):
          bit7  = 0 (Type A)
          bits6-4 = x (3 bits)
          bits3-2 = y (2 bits)
          bit1-0    = z (2 bits)
        Returns (x_q, y_q, z_q) as integers.
        """
        x_q = (byte_val >> 4) & 0x7
        y_q = (byte_val >> 2) & 0x3
        z_q = byte_val & 0x3
        return x_q, y_q, z_q

    # ───────────────────────────────────────────────────────────────────────────
    # NEW: parse_packet_b
    # ───────────────────────────────────────────────────────────────────────────
    @staticmethod
    def parse_packet_b(byte_val: int):
        """
        Reverse of build_packet_b (8 bits):
          bit7     = Type B (1)
          bits6-5  = service_id (2 bits)
          bit4     = value (1 bit)
          bits3-1  = CRC-3 (ignored here)
          bit0     = seq (1 bit)
        Returns (service_id, value, seq) as integers.
        """
        service_id = (byte_val >> 5) & 0x3
        value      = (byte_val >> 4) & 0x1
        seq        = byte_val & 0x1
        return service_id, value, seq


class ModemCommunicator:
    """Handles serial communication to the acoustic modem."""

    def __init__(self, port: str, baud: int = 9600, timeout: float = 1.0):
        self._ser = serial.Serial(port, baud, timeout=timeout)

    def send_packet(self, packet: bytes) -> None:
        """Write raw packet bytes to modem."""
        self._ser.write(packet)

    def read_bytes(self, length: int) -> bytes:
        """Read a fixed number of bytes from modem."""
        return self._ser.read(length)

    def request_response(self, packet: bytes, reply_length: int, timeout: float = 1.0) -> bytes:
        """Send packet and wait for reply of specified length."""
        self._ser.timeout = timeout
        self._ser.write(packet)
        return self._ser.read(reply_length)

    def close(self) -> None:
        """Close serial port."""
        self._ser.close()


class TransmissionStep:
    """Defines a single step in a transmission sequence."""
    def __init__(self, name, action, duration=None, wait_for=None):
        self.name     = name
        self.action   = action       # function to call when step begins
        self.duration = duration     # seconds to wait after action
        self.wait_for = wait_for     # function returning True when ready to proceed


class TransmissionScheduler(Thread):
    """Runs a configurable sequence of TransmissionSteps."""
    def __init__(self, steps, loop=False):
        super().__init__(daemon=True)
        self.steps       = steps
        self.loop        = loop
        self.stop_event  = Event()

    def run(self):
        while not self.stop_event.is_set():
            for step in self.steps:
                if self.stop_event.is_set():
                    return
                # execute action, if any
                if step.action:
                    step.action()
                # wait fixed duration or until condition holds
                if step.duration is not None:
                    end_time = time.monotonic() + step.duration
                    while time.monotonic() < end_time:
                        if self.stop_event.is_set():
                            return
                        time.sleep(0.01)
                elif step.wait_for is not None:
                    while not step.wait_for():
                        if self.stop_event.is_set():
                            return
                        time.sleep(0.01)
                # otherwise, proceed immediately
            if not self.loop:
                break

    def stop(self):
        self.stop_event.set()
