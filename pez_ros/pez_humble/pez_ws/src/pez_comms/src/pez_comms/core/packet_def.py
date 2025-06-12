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
        return "A_NORMAL"

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
        return "B_COMMAND"

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

# New 40-bit packet
class Packet40(PacketDefinition):
    @property
    def packet_id(self) -> str:
        return "PACKET_40"

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
            ((seq & 0x0F)        << 28) |
            ((vx  & 0xFF)        << 20) |
            ((vy  & 0xFF)        << 12) |
            ((vz  & 0xFF)        <<  4) |
            ((svc_pending & 0x1) <<  3) |
            ((svc_id       & 0x3) <<  1) |
            ((svc_val      & 0x1) <<  0)
        )
        b4 = struct.pack('>I', word)
        crc = _crc8_bytes(b4)
        return b4 + struct.pack('B', crc)

    def decode(self, raw: bytes) -> Dict[str, int]:
        if len(raw) != 5:
            raise ValueError("Invalid length for PACKET_40")
        payload, recv_crc = raw[:4], raw[4]
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

# Register all packets
register_packet(PacketA_Normal())
register_packet(PacketB_Command())
register_packet(Packet40())
