import importlib
import inspect
import logging
import pkgutil
import struct
import sys
from types import ModuleType

from . import nanopb_pb2
# The generated pb2 files use absolute `import nanopb_pb2`, so alias it here.
sys.modules.setdefault("nanopb_pb2", nanopb_pb2)

from google.protobuf.message import Message as ProtoMessage

# Frame format: [SYNC_0: 0xAA] [SYNC_1: 0x55] [LENGTH: uint16_t LE] [TYPE: uint8_t] [PAYLOAD: LENGTH bytes]
SYNC_0: int = 0xAA
SYNC_1: int = 0x55
FRAME_HEADER_SIZE: int = 5
MAX_PAYLOAD_SIZE: int = 128
MAX_FRAME_SIZE: int = FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE

# Auto-discover all proto message classes with a nanopb msgid.
# Any *_pb2 module generated into this package is automatically included.
MSG_REGISTRY: dict[int, type] = {}
MSG_IDS: dict[type, int] = {}
MSG_NAMES: dict[str, type] = {}

for _mod_info in pkgutil.iter_modules(__path__):
    if not _mod_info.name.endswith("_pb2") or _mod_info.name == "nanopb_pb2":
        continue
    _mod: ModuleType = importlib.import_module(f".{_mod_info.name}", package=__name__)
    _cls: type
    for _, _cls in inspect.getmembers(_mod, inspect.isclass):
        if not issubclass(_cls, ProtoMessage) or _cls is ProtoMessage:
            continue
        if _cls.__module__ != _mod_info.name:
            continue
        _msgid: int = _cls.DESCRIPTOR.GetOptions().Extensions[nanopb_pb2.nanopb_msgopt].msgid
        if _msgid != 0:
            MSG_REGISTRY[_msgid] = _cls
            MSG_IDS[_cls] = _msgid
            MSG_NAMES[_cls.__name__] = _cls
            globals()[_cls.__name__] = _cls
        else:
            logging.warning(
                "pico_interface: %s has no msgid and cannot be sent as a standalone frame. "
                "Add (nanopb_msgopt).msgid to its .proto definition if this is unintentional.",
                _cls.__name__,
            )


class FrameError(Exception):
    pass


def encode_frame(msg: ProtoMessage) -> bytes:

    msg_class: type = type(msg)
    if msg_class not in MSG_IDS:
        raise FrameError(f"Unknown message type: {msg_class.__name__}")

    msg_id: int = MSG_IDS[msg_class]
    payload: bytes = msg.SerializeToString()
    payload_len: int = len(payload)

    if payload_len > MAX_PAYLOAD_SIZE:
        raise FrameError(f"Payload too large: {payload_len} > {MAX_PAYLOAD_SIZE}")

    header: bytes = struct.pack("<BBHB", SYNC_0, SYNC_1, payload_len, msg_id)
    return header + payload


def decode_frame(buf: bytes) -> tuple[int, bytes]:
    """Parse a complete frame buffer. Returns (msg_id, payload_bytes)."""

    if len(buf) < FRAME_HEADER_SIZE:
        raise FrameError(f"Buffer too small: {len(buf)} < {FRAME_HEADER_SIZE}")
    if buf[0] != SYNC_0 or buf[1] != SYNC_1:
        raise FrameError(f"Invalid sync bytes: {buf[0]:02x} {buf[1]:02x}")

    payload_len: int
    msg_id: int
    payload_len, msg_id = struct.unpack_from("<HB", buf, offset=2)

    if len(buf) < FRAME_HEADER_SIZE + payload_len:
        raise FrameError("Incomplete frame")

    payload: bytes = bytes(buf[FRAME_HEADER_SIZE:FRAME_HEADER_SIZE + payload_len])
    return msg_id, payload


def decode_message(buf: bytes) -> ProtoMessage | None:
    """Decode a complete frame and deserialize its payload. Returns a proto object, or None for unknown msg_id."""

    msg_id: int
    payload: bytes
    msg_id, payload = decode_frame(buf)

    msg_class: type | None = MSG_REGISTRY.get(msg_id)
    if msg_class is None:
        return None

    msg: ProtoMessage = msg_class()
    msg.ParseFromString(payload)
    return msg
