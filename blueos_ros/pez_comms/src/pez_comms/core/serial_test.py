#!/usr/bin/env python3
import time
import argparse
import serial
import struct

# ────────────────────────────────────────────────────────────────────────────────
# CRC helpers (copied from pez_comms.utility.PacketBuilder)
# ────────────────────────────────────────────────────────────────────────────────
def _crc_generic(data: int, width: int, poly: int) -> int:
    """
    Compute a width‐bit CRC over the integer `data` using given polynomial `poly`.
    """
    mask = (1 << width) - 1
    data = data << width
    total = data.bit_length()
    for shift in range(total - width - 1, -1, -1):
        if data & (1 << (shift + width)):
            data ^= (poly << shift)
    return data & mask


_CRC2_POLY = 0b111    # x^2 + x + 1
_CRC3_POLY = 0b1011   # x^3 + x + 1
_CRC6_POLY = 0b100111 # CRC‐6‐ITU: x^6 + x^2 + x + 1


class PacketBuilder:
    """
    Builds Packet A (velocity: 3/2/2 bits) and Packet B (teleop: service commands) 
    in exactly the same format as pez_comms.utility.PacketBuilder.
    """

    @staticmethod
    def build_packet_a_normal(x: int, y: int, z: int) -> bytes:
        """
        Packet A Normal (8 bits):
          bit7        = Type A (0)
          bits6-4     = x (3 bits)
          bits3-2     = y (2 bits)
          bits1-0     = z (2 bits)
        """
        pkt = ((x & 0x7) << 4) | ((y & 0x3) << 2) | (z & 0x3)
        return struct.pack('B', pkt)

    @staticmethod
    def build_packet_b(service_id: int, value: int, seq: int) -> bytes:
        """
        Packet B format (8 bits):
          bit7        = Type B (1)
          bits6-5     = service_id (2 bits)
          bit4        = value (1 bit)
          bits3-1     = CRC-3 over bits[7:4]
          bit0        = seq (1 bit)
        """
        header = (1 << 7) | ((service_id & 0x3) << 5) | ((value & 0x1) << 4)
        crc3 = _crc_generic(header >> 4, width=3, poly=_CRC3_POLY)
        pkt = header | ((crc3 & 0x7) << 1) | (seq & 0x1)
        return struct.pack('B', pkt)


# ────────────────────────────────────────────────────────────────────────────────
# quantization helper (same as pez_comms)
# ────────────────────────────────────────────────────────────────────────────────
def _quantize(val: float, bits: int) -> int:
    """
    Convert a float in [-1.0, +1.0] into an integer in [0 .. (2^bits - 1)].
    """
    levels = (1 << bits)
    q = round((val + 1.0) / 2.0 * (levels - 1))
    return max(0, min(levels - 1, q))


def send_packet_a(ser: serial.Serial, x: float, y: float, z: float) -> None:
    """
    Build and send one Packet A (3 bits for x, 2 bits for y, 2 bits for z).
    """
    x_q = _quantize(x, bits=3)
    y_q = _quantize(y, bits=2)
    z_q = _quantize(z, bits=2)

    pkt = PacketBuilder.build_packet_a_normal(x_q, y_q, z_q)
    ser.write(pkt)
    print(f"→ Sent Packet A: x_q={x_q}, y_q={y_q}, z_q={z_q}, byte=0x{pkt.hex().upper()}")


def send_packet_b(ser: serial.Serial, svc_id: int, value: int, seq: int) -> None:
    """
    Build and send one Packet B (service command).
    """
    pkt = PacketBuilder.build_packet_b(svc_id, value, seq)
    ser.write(pkt)
    print(f"→ Sent Packet B: svc_id={svc_id}, value={value}, seq={seq}, byte=0x{pkt.hex().upper()}")


def main():
    p = argparse.ArgumentParser(
        description="Test‐script to send pez_comms Packet A / Packet B over serial"
    )
    p.add_argument(
        "-p",
        "--port",
        default="/dev/ttyUSB0",
        help="Serial device (e.g. /dev/ttyUSB0)",
    )
    sub = p.add_subparsers(dest="mode", required=True)

    # 1) mode “a”: send one Packet A
    pa = sub.add_parser("a", help="send one Packet A")
    pa.add_argument("x", type=float, help="linear x in [-1.0,+1.0]")
    pa.add_argument("y", type=float, help="linear y in [-1.0,+1.0]")
    pa.add_argument("z", type=float, help="linear z in [-1.0,+1.0]")

    # 2) mode “b”: send one Packet B
    pb = sub.add_parser("b", help="send one Packet B (teleop)")
    pb.add_argument("svc_id", type=int, choices=[0, 1, 2, 3], help="service_id (0..3)")
    pb.add_argument("value", type=int, choices=[0, 1], help="value (0 or 1)")
    pb.add_argument("seq", type=int, choices=[0, 1], help="sequence bit (0 or 1)")

    # 3) mode “loop”: continuously send Packet A at 4 Hz
    pl = sub.add_parser("loop", help="continuously send Packet A at 4 Hz")
    pl.add_argument(
        "--x", type=float, default=0.0, help="linear x in [-1.0,+1.0] (default 0.0)"
    )
    pl.add_argument(
        "--y", type=float, default=0.0, help="linear y in [-1.0,+1.0] (default 0.0)"
    )
    pl.add_argument(
        "--z", type=float, default=0.0, help="linear z in [-1.0,+1.0] (default 0.0)"
    )

    args = p.parse_args()

    # Open serial port at 9600 baud, 8N1, no timeout on write
    ser = serial.Serial(args.port, 9600, timeout=1.0)

    if args.mode == "a":
        send_packet_a(ser, args.x, args.y, args.z)

    elif args.mode == "b":
        send_packet_b(ser, args.svc_id, args.value, args.seq)

    elif args.mode == "loop":
        try:
            print("→ Press Ctrl–C to stop.")
            while True:
                send_packet_a(ser, args.x, args.y, args.z)
                time.sleep(0.25)  # 4 Hz
        except KeyboardInterrupt:
            print("\nStopping loop.")

    ser.close()


if __name__ == "__main__":
    main()
