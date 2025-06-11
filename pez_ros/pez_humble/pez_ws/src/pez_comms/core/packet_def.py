# core/packet_def.py

from abc import ABC, abstractmethod
from typing import Dict, Tuple, Any


class PacketDefinition(ABC):
    """
    Abstract base class for one “packet type.” A PacketDefinition knows how to:
      • encode(...) → bytes (or a sequence of bytes)
      • decode(raw: bytes or int) → (dict of fields)
    """

    @abstractmethod
    def encode(self, **kwargs) -> bytes:
        """
        Given named fields, build the raw packet bytes.
        E.g. for “Packet A normal,” fields might be x=4, y=2, z=3.
        """
        raise NotImplementedError

    @abstractmethod
    def decode(self, raw: bytes) -> Dict[str, Any]:
        """
        Given raw bytes (or a single‐byte `bytes` object), parse it
        according to this packet’s format, and return a dict of fields.
        For Packet A normal, return {'x_q': int, 'y_q': int, 'z_q': int}.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def packet_id(self) -> str:
        """
        A unique string identifier for this packet type, e.g. "A_NORMAL" or "B_CMD".
        """
        raise NotImplementedError


# A simple registry so host_side/fish_side can say “register PacketA_Normal, PacketB_Command, etc.”
_PACKET_REGISTRY: Dict[str, PacketDefinition] = {}


def register_packet(defn: PacketDefinition) -> None:
    """
    Add a PacketDefinition to the registry under defn.packet_id.
    If a duplicate ID is registered, it overwrites the old one.
    """
    pid = defn.packet_id
    if pid in _PACKET_REGISTRY:
        raise ValueError(f"Packet ID '{pid}' already registered")
    _PACKET_REGISTRY[pid] = defn


def get_packet(packet_id: str) -> PacketDefinition:
    """
    Look up a PacketDefinition by ID. Raises KeyError if not found.
    """
    return _PACKET_REGISTRY[packet_id]


def list_packet_ids() -> Tuple[str, ...]:
    """Return a tuple of all registered packet IDs."""
    return tuple(_PACKET_REGISTRY.keys())


# ——— Now we can define your two built‐in packet types as classes below ———

import struct

# CRC utilities as before (or import from utility)
def _crc_generic(data: int, width: int, poly: int) -> int:
    mask = (1 << width) - 1
    data = data << width
    total = data.bit_length()
    for shift in range(total - width - 1, -1, -1):
        if data & (1 << (shift + width)):
            data ^= (poly << shift)
    return data & mask

_CRC2_POLY = 0b111
_CRC3_POLY = 0b1011
_CRC6_POLY = 0b100111


class PacketA_Normal(PacketDefinition):
    """
    Packet A “Normal” (1 byte):
      bit7 = 0
      bits6-4 = x (3 bits)
      bits3-2 = y (2 bits)
      bits1-0 = z (2 bits)
    We interpret z as a 2‐bit value in [0..3]. (Originally you stored 1 bit for z, but now you said 2 bits.)
    For consistency with your last edit, we assume (x:3 bits, y:2 bits, z:2 bits).
    """

    @property
    def packet_id(self) -> str:
        return "A_NORMAL"

    def encode(self, x: int, y: int, z: int) -> bytes:
        header = ((x & 0x7) << 4) | ((y & 0x3) << 2) | (z & 0x3)
        return struct.pack('B', header)

    def decode(self, raw: bytes) -> Dict[str, int]:
        # raw is a 1-byte bytes object or raw[0] is int in [0..255]
        byte_val = raw[0] if isinstance(raw, (bytes, bytearray)) else int(raw)
        x_q = (byte_val >> 4) & 0x7
        y_q = (byte_val >> 2) & 0x3
        z_q = byte_val & 0x3
        return {"x_q": x_q, "y_q": y_q, "z_q": z_q}


class PacketB_Command(PacketDefinition):
    """
    Packet B (1 byte):
      bit7      = 1 (type B)
      bits6-5   = service_id (2 bits)
      bit4      = value (1 bit)
      bits3-1   = CRC-3 over bits[7:4]   (we ignore for decode, but we could verify it)
      bit0      = seq (1 bit)
    """

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
        # (Optionally, you could verify the CRC-3 here.)
        seq        = byte_val & 0x1
        return {"service_id": service_id, "value": value, "seq": seq}


# Finally, register these two built‐ins when this file is imported:
register_packet(PacketA_Normal())
register_packet(PacketB_Command())
