from abc import ABC, abstractmethod
from typing import Dict, Tuple, Any

# Packet registry
_PACKET_REGISTRY: Dict[str, 'PacketDefinition'] = {}

def register_packet(defn: 'PacketDefinition') -> None:
    pid = defn.packet_id
    if pid in _PACKET_REGISTRY:
        raise ValueError(f"Packet ID '{pid}' already registered")
    _PACKET_REGISTRY[pid] = defn

def get_packet(packet_id: str) -> 'PacketDefinition':
    return _PACKET_REGISTRY[packet_id]

def list_packet_ids() -> Tuple[str, ...]:
    return tuple(_PACKET_REGISTRY.keys())

class PacketDefinition(ABC):
    """
    Abstract base class for one “packet type.”
    """
    @property
    @abstractmethod
    def packet_id(self) -> str:
        pass

    @abstractmethod
    def encode(self, **kwargs) -> bytes:
        pass

    @abstractmethod
    def decode(self, raw: bytes) -> Dict[str, Any]:
        pass

# CRC utilities
import struct

def _crc_generic(data: int, width: int, poly: int) -> int:
    mask = (1 << width) - 1
    data = data << width
    total = data.bit_length()
    for shift in range(total - width - 1, -1, -1):
        if data & (1 << (shift + width)):
            data ^= (poly << shift)
    return data & mask

# Helper CRC-8 (poly 0x07)
def _crc8_bytes(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

# Built-in packet A
class PacketA_Normal(PacketDefinition):
    @property
    def packet_id(self) -> str:
        return 0xA

    def encode(self, x: int, y: int, z: int) -> bytes:
        header = ((x & 0x7) << 4) | ((y & 0x3) << 2) | (z & 0x3)
        return struct.pack('B', header)

    def decode(self, raw: bytes) -> Dict[str, int]:
        byte_val = raw[0] if isinstance(raw, (bytes, bytearray)) else int(raw)
        x_q = (byte_val >> 4) & 0x7
        y_q = (byte_val >> 2) & 0x3
        z_q = byte_val & 0x3
        return {"x_q": x_q, "y_q": y_q, "z_q": z_q}

# Built-in packet B
_CRC3_POLY = 0b1011
class PacketB_Command(PacketDefinition):
    @property
    def packet_id(self) -> str:
        return "PacketB"

    def encode(self, service_id: int, value: int, seq: int) -> bytes:
        header = (1 << 7) | ((service_id & 0x3) << 5) | ((value & 0x1) << 4)
        crc3 = _crc_generic(header >> 4, width=3, poly=_CRC3_POLY)
        pkt = header | ((crc3 & 0x7) << 1) | (seq & 0x1)
        return struct.pack('B', pkt)

    def decode(self, raw: bytes) -> Dict[str, int]:
        byte_val = raw[0] if isinstance(raw, (bytes, bytearray)) else int(raw)
        service_id = (byte_val >> 5) & 0x3
        value      = (byte_val >> 4) & 0x1
        seq        = byte_val & 0x1
        return {"service_id": service_id, "value": value, "seq": seq}



class Packet40(PacketDefinition):
    @property
    def packet_id(self) -> int:
        # This must match the first byte you put on the wire for a 40-bit packet.
        return 0x40

    def encode(self,
               seq: int,
               vx: int,
               vy: int,
               vz: int,
               svc_pending: int,
               svc_id: int,
               svc_val: int) -> bytes:
        # pack bits 0–31 into a 32-bit word
        word = (
            ((seq        & 0x0F) << 28) |
            ((vx         & 0xFF) << 20) |
            ((vy         & 0xFF) << 12) |
            ((vz         & 0xFF) <<  4) |
            ((svc_pending&   0x1) <<  3) |
            ((svc_id     &   0x3) <<  1) |
            ((svc_val    &   0x1) <<  0)
        )
        b4 = struct.pack('>I', word)
        crc = _crc8_bytes(b4)
        # Prepend the packet_id byte, then payload+CRC
        return bytes([self.packet_id]) + b4 + struct.pack('B', crc)

    def decode(self, raw: bytes) -> Dict[str, int]:
        # Expect exactly 1 (ID) + 4 (payload) + 1 (CRC) = 6 bytes total
        if len(raw) != 6:
            raise ValueError(f"Invalid length for PACKET_40: got {len(raw)}")
        pid, payload, recv_crc = raw[0], raw[1:5], raw[5]
        if pid != self.packet_id:
            raise ValueError(f"Wrong packet_id: {pid}")
        if _crc8_bytes(payload) != recv_crc:
            raise ValueError("CRC mismatch in PACKET_40")
        w = struct.unpack('>I', payload)[0]
        return {
            "seq":         (w >> 28) & 0x0F,
            "vx":          (w >> 20) & 0xFF,
            "vy":          (w >> 12) & 0xFF,
            "vz":          (w >>  4) & 0xFF,
            "svc_pending": (w >>  3) & 0x1,
            "svc_id":      (w >>  1) & 0x3,
            "svc_val":     (w >>  0) & 0x1,
        }
    
class PacketB_Full(PacketDefinition):
    """
    40-bit PacketB: full-width command ACK/NACK frame.

    Frame layout (40 bits):
      [ID (8)] [Payload (32)] [CRC8 (8)]
    Payload bits (MSB->LSB):
      • seq        : 4 bits
      • service_id : 4 bits
      • value      : 4 bits
      • reserved   : 20 bits (zero)
    """
    @property
    def packet_id(self) -> int:
        # First-byte identifier for PacketB_Full
        return 0x0B

    def encode(self, *, seq: int, service_id: int, value: int) -> bytes:
        # Pack payload: seq (4) | service_id (4) | value (4) | zeros (20)
        payload = (
            ((seq & 0x0F)        << 28) |
            ((service_id & 0x0F) << 24) |
            ((value & 0x0F)      << 20)
        )
        # 32-bit big-endian
        b4 = struct.pack('>I', payload)
        # 1-byte CRC over payload
        crc = _crc8_bytes(b4)
        # Return: ID + payload + CRC
        return bytes([self.packet_id]) + b4 + struct.pack('B', crc)

    def decode(self, raw: bytes) -> Dict[str, int]:
        if len(raw) != 6:
            raise ValueError(f"Invalid length for PacketB_Full: got {len(raw)} bytes")
        pid = raw[0]
        if pid != self.packet_id:
            raise ValueError(f"Wrong packet_id: expected 0x{self.packet_id:02X}, got 0x{pid:02X}")
        payload = raw[1:5]
        recv_crc = raw[5]
        if _crc8_bytes(payload) != recv_crc:
            raise ValueError("CRC mismatch in PacketB_Full")
        word = struct.unpack('>I', payload)[0]
        return {
            'seq':        (word >> 28) & 0x0F,
            'service_id': (word >> 24) & 0x0F,
            'value':      (word >> 20) & 0x0F,
        }


# Register all packets
register_packet(PacketA_Normal())
register_packet(PacketB_Command())
register_packet(Packet40())
register_packet(PacketB_Full())

