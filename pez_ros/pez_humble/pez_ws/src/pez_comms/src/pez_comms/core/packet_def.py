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
    
    @staticmethod
    def to_hex_string(pkt: bytes) -> str:
        return ' '.join(f'{b:02X}' for b in pkt)
    
    @staticmethod
    def _quant(v: float, bits: int) -> int:
        # signed two’s-complement quant: v=0 → q=0 exactly
        max_mag = (1 << (bits - 1)) - 1               # e.g. 31 for 6 bits
        v_clamped = max(-1.0, min(1.0, v))
        q_signed  = int(round(v_clamped * max_mag))  # in [-max_mag..+max_mag]
        return q_signed & ((1 << bits) - 1)           # pack to unsigned

    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        # invert two’s-complement
        sign_bit = 1 << (bits - 1)
        if q & sign_bit:
            q -= (1 << bits)
        max_mag = (1 << (bits - 1)) - 1
        return q / max_mag

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



# CRC-4 polynomial x^4 + x + 1 → 0b1_0011
_CRC4_POLY = 0b10011

class Packet40(PacketDefinition):
    """
    32-bit wire format (big-endian):
      [ID:3][CRC-4][SEQ:3][VX:6][VY:6][VZ:6][SVC_P:1][SVC_ID:2][SVC_V:1]
    All velocities are two-s-compl. quantised to 6 bits, mapping −1.0…+1.0.
    """

    # ---------- constants ----------
    _ID_SHIFT   = 29          # bits 31-29
    _CRC_SHIFT  = 25          # bits 28-25
    _SEQ_SHIFT  = 22          # bits 24-22
    _VX_SHIFT   = 16          # bits 21-16
    _VY_SHIFT   = 10          # bits 15-10
    _VZ_SHIFT   = 4           # bits  9-4
    _SVCP_SHIFT = 3           # bit   3
    _SVCI_SHIFT = 1           # bits  2-1
    _SVCV_SHIFT = 0           # bit   0

    @property
    def packet_id(self) -> int:           # 0…7
        return 0x04

    # ---------- helpers ----------
    @staticmethod
    def _quant(v: float, bits: int) -> int:
        m = (1 << (bits - 1)) - 1
        q = int(round(max(-1.0, min(1.0, v)) * m))
        return q & ((1 << bits) - 1)

    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        if q & (1 << (bits - 1)):                       # sign-extend
            q -= 1 << bits
        return q / ((1 << (bits - 1)) - 1)

    # ---------- ENCODE ----------
    def encode(
        self, *, seq: int,
        vx: float, vy: float, vz: float,
        svc_pending: int, svc_id: int, svc_val: int
    ) -> bytes:

        qx, qy, qz = (self._quant(v, 6) for v in (vx, vy, vz))

        # body = everything *below* the CRC (25 bits)
        body25 = (
            (seq & 0x7)        << self._SEQ_SHIFT  |
            qx                 << self._VX_SHIFT   |
            qy                 << self._VY_SHIFT   |
            qz                 << self._VZ_SHIFT   |
            (svc_pending & 1)  << self._SVCP_SHIFT |
            (svc_id      & 3)  << self._SVCI_SHIFT |
            (svc_val     & 1)  << self._SVCV_SHIFT
        )

        crc4 = _crc_generic(body25, width=4, poly=_CRC4_POLY) & 0xF

        word32 = (
            (self.packet_id & 0x7) << self._ID_SHIFT |
            crc4                    << self._CRC_SHIFT |
            body25
        )
        return word32.to_bytes(4, "big")

    # ---------- DECODE ----------
    def decode(self, raw: bytes) -> Dict[str, Any]:
        if len(raw) != 4:
            raise ValueError("need 4 bytes")
        w = int.from_bytes(raw, "big")

        pid3     = (w >> self._ID_SHIFT)   & 0x7
        recv_crc = (w >> self._CRC_SHIFT)  & 0xF
        body25   =  w & 0x01_FF_FF_FF      # low 25 bits

        if pid3 != (self.packet_id & 0x7):
            raise ValueError(f"wrong packet_id {pid3}")

        if (_crc_generic(body25, width=4, poly=_CRC4_POLY) & 0xF) != recv_crc:
            raise ValueError("CRC mismatch")

        seq3 = (w >> self._SEQ_SHIFT) & 0x7
        qx   = (w >> self._VX_SHIFT)  & 0x3F
        qy   = (w >> self._VY_SHIFT)  & 0x3F
        qz   = (w >> self._VZ_SHIFT)  & 0x3F
        svcP = (w >> self._SVCP_SHIFT) & 0x1
        svcI = (w >> self._SVCI_SHIFT) & 0x3
        svcV =  w & 0x1

        return {
            "seq": seq3,
            "vx":  self._dequant(qx, 6),
            "vy":  self._dequant(qy, 6),
            "vz":  self._dequant(qz, 6),
            "svc_pending": svcP,
            "svc_id":      svcI,
            "svc_val":     svcV,
        }





class PacketB_Full(PacketDefinition):
    """
    32-bit PacketB_Full:

      [ ID:4 │ CRC:4 │ SEQ:4 │ SERVICE_ID:4 │ VALUE:4 │ RESERVED:12 ]
    """

    @property
    def packet_id(self) -> int:
        # We'll truncate this to 4 bits (0x0B & 0x0F == 0x0B)
        return 0x0B

    def encode(self, *, seq: int, service_id: int, value: int) -> bytes:
        # 1) mask all fields to their bit-width
        pid4  = self.packet_id   & 0x0F
        seq4  = seq              & 0x0F
        sid4  = service_id       & 0x0F
        val4  = value            & 0x0F
        # reserved12 = 0

        # 2) build the 32-bit word with CRC field zeroed
        #    bits 31–28: ID4
        #    bits 27–24: CRC4 (zero for now)
        #    bits 23–20: SEQ4
        #    bits 19–16: SERVICE_ID4
        #    bits 15–12: VALUE4
        #    bits 11– 0: RESERVED12 (all zero)
        word_no_crc = (
            (pid4 << 28) |
            (seq4 << 20) |
            (sid4 << 16) |
            (val4 << 12)
        )

        # 3) compute 4-bit CRC over that word
        crc4 = _crc_generic(word_no_crc, width=4, poly=_CRC4_POLY) & 0x0F

        # 4) splice CRC into bits 27–24
        word32 = word_no_crc | (crc4 << 24)

        # 5) emit as 4 bytes big-endian
        return word32.to_bytes(4, 'big')

    def decode(self, raw: bytes) -> Dict[str, int]:
        if len(raw) != 4:
            raise ValueError(f"Expected 4 bytes, got {len(raw)}")

        word32 = int.from_bytes(raw, 'big')

        # 1) extract CRC and clear it out
        recv_crc = (word32 >> 24) & 0x0F
        word_no_crc = word32 & ~(0x0F << 24)

        # 2) verify CRC
        if (_crc_generic(word_no_crc, width=4, poly=_CRC4_POLY) & 0x0F) != recv_crc:
            raise ValueError("CRC mismatch in PacketB_Full")

        # 3) peel out each field
        pid4 = (word_no_crc >> 28) & 0x0F
        if pid4 != (self.packet_id & 0x0F):
            raise ValueError(f"Wrong packet_id: {pid4}")

        return {
            "seq":         (word_no_crc >> 20) & 0x0F,
            "service_id":  (word_no_crc >> 16) & 0x0F,
            "value":       (word_no_crc >> 12) & 0x0F,
            # reserved is always zero; drop it
        }

class PacketBlueRov(PacketDefinition):
    """
    32-bit wire format (big-endian):
      [ID:3][CRC-4][SEQ:3]
      [VX:4][VY:4][VZ:5][WZ:4]
      [SVC_P:1][SVC_ID:3][SVC_V:1]

    Velocities are floats in [-0.5, +0.5], quantized to the signed range of each bit-field.
    """
    # bit-positions (LSB) of each field
    _ID_SHIFT   = 29
    _CRC_SHIFT  = 25
    _SEQ_SHIFT  = 22
    _VX_SHIFT   = 18  # 4 bits
    _VY_SHIFT   = 14  # 4 bits
    _VZ_SHIFT   = 9   # 5 bits
    _WZ_SHIFT   = 5   # 4 bits
    _SVCP_SHIFT = 4   # 1 bit
    _SVCI_SHIFT = 1   # 3 bits
    _SVCV_SHIFT = 0   # 1 bit

    @property
    def packet_id(self) -> int:
        return 0x02

    @staticmethod
    def _quant_half(v: float, bits: int) -> int:
        # scale [-0.5..0.5] → [-1..1] then quantize
        return PacketDefinition._quant(v * 2.0, bits)

    @staticmethod
    def _dequant_half(q: int, bits: int) -> float:
        # dequant to [-1..1] then scale back
        return PacketDefinition._dequant(q, bits) * 0.5

    def encode(
        self,
        *,
        seq: int,
        vx: float, vy: float, vz: float, wz: float,
        svc_pending: int, svc_id: int, svc_val: int
    ) -> bytes:
        qx = self._quant_half(vx, 4)
        qy = self._quant_half(vy, 4)
        qz = self._quant_half(vz, 5)
        qw = self._quant_half(wz, 4)

        body25 = (
            ((seq       & 0x7) << self._SEQ_SHIFT) |
            (qx               << self._VX_SHIFT)  |
            (qy               << self._VY_SHIFT)  |
            (qz               << self._VZ_SHIFT)  |
            (qw               << self._WZ_SHIFT)  |
            ((svc_pending & 1) << self._SVCP_SHIFT) |
            ((svc_id      & 0x7) << self._SVCI_SHIFT) |
            ((svc_val     & 1) << self._SVCV_SHIFT)
        )

        crc4 = _crc_generic(body25, width=4, poly=_CRC4_POLY) & 0xF

        word32 = (
            ((self.packet_id & 0x7) << self._ID_SHIFT) |
            (crc4                    << self._CRC_SHIFT) |
            body25
        )
        return word32.to_bytes(4, 'big')

    def decode(self, raw: bytes) -> Dict[str, Any]:
        if len(raw) != 4:
            raise ValueError("PacketBlueRov requires exactly 4 bytes")
        w = int.from_bytes(raw, 'big')

        pid = (w >> self._ID_SHIFT) & 0x7
        if pid != (self.packet_id & 0x7):
            raise ValueError(f"Bad packet_id: {pid}")

        recv_crc = (w >> self._CRC_SHIFT) & 0xF
        body25   = w & ((1 << self._CRC_SHIFT) - 1)
        if (_crc_generic(body25, width=4, poly=_CRC4_POLY) & 0xF) != recv_crc:
            raise ValueError("CRC mismatch")

        seq         = (w >> self._SEQ_SHIFT)  & 0x7
        qx          = (w >> self._VX_SHIFT)  & 0xF
        qy          = (w >> self._VY_SHIFT)  & 0xF
        qz          = (w >> self._VZ_SHIFT)  & 0x1F
        qw          = (w >> self._WZ_SHIFT)  & 0xF
        svc_pending = (w >> self._SVCP_SHIFT)& 0x1
        svc_id      = (w >> self._SVCI_SHIFT)& 0x7
        svc_val     =  w                       & 0x1

        return {
            'seq':         seq,
            'vx':          self._dequant_half(qx, 4),
            'vy':          self._dequant_half(qy, 4),
            'vz':          self._dequant_half(qz, 5),
            'wz':          self._dequant_half(qw, 4),
            'svc_pending': svc_pending,
            'svc_id':      svc_id,
            'svc_val':     svc_val,
        }

# register under new name
register_packet(PacketBlueRov())


# Register all packets
register_packet(PacketA_Normal())
register_packet(PacketB_Command())
register_packet(Packet40())
register_packet(PacketB_Full())

