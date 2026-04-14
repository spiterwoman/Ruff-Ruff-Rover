from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional

try:
    import usb.core
    import usb.util
except Exception:
    usb = None


XVF3800_VID = 0x2886
XVF3800_PID = 0x001A
XVF3800_NAME_HINTS = (
    "XVF3800",
    "ReSpeaker USB 4-Mic Array",
    "ReSpeaker",
)


class XVF3800USBError(RuntimeError):
    pass


@dataclass
class XVF3800Snapshot:
    doa_deg: Optional[float]
    voice_activity: Optional[bool]


class XVF3800Tuning:
    TIMEOUT_MS = 100000
    DOA_RESID = 20
    DOA_CMDID = 18
    DOA_LENGTH = 4

    def __init__(self, dev) -> None:
        self.dev = dev

    @classmethod
    def find(cls, vid: int = XVF3800_VID, pid: int = XVF3800_PID) -> "XVF3800Tuning":
        if usb is None:
            raise XVF3800USBError("pyusb is not installed")
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            raise XVF3800USBError("XVF3800 USB device not found")
        return cls(dev)

    def _read_words(self, resid: int, cmdid: int, length: int) -> tuple[int, ...]:
        if usb is None:
            raise XVF3800USBError("pyusb is not installed")
        payload = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmdid,
            resid,
            length,
            self.TIMEOUT_MS,
        )
        raw = bytes(payload)
        if len(raw) < 2:
            raise XVF3800USBError("short USB response from XVF3800")
        if len(raw) % 2 != 0:
            raw += b"\x00"
        return struct.unpack("<" + ("H" * (len(raw) // 2)), raw)

    @property
    def snapshot(self) -> XVF3800Snapshot:
        words = self._read_words(self.DOA_RESID, self.DOA_CMDID, self.DOA_LENGTH)
        doa = float(words[0] % 360) if words else None
        vad = bool(words[1]) if len(words) > 1 else None
        return XVF3800Snapshot(doa_deg=doa, voice_activity=vad)

    def close(self) -> None:
        if usb is not None:
            usb.util.dispose_resources(self.dev)


def circular_mean_deg(angles_deg: list[float]) -> Optional[float]:
    if not angles_deg:
        return None
    x_value = sum(math.cos(math.radians(angle)) for angle in angles_deg)
    y_value = sum(math.sin(math.radians(angle)) for angle in angles_deg)
    if abs(x_value) < 1e-9 and abs(y_value) < 1e-9:
        return None
    return (math.degrees(math.atan2(y_value, x_value)) + 360.0) % 360.0


def find_xvf3800_input_device_index(pyaudio_instance) -> int:
    host_info = pyaudio_instance.get_host_api_info_by_index(0)
    device_count = int(host_info.get("deviceCount", 0))
    for device_index in range(device_count):
        info = pyaudio_instance.get_device_info_by_host_api_device_index(0, device_index)
        name = str(info.get("name", ""))
        max_inputs = int(info.get("maxInputChannels", 0))
        if max_inputs <= 0:
            continue
        if any(hint.lower() in name.lower() for hint in XVF3800_NAME_HINTS):
            return device_index
    raise XVF3800USBError("could not find an XVF3800 audio input device")
