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


RESPEAKER_VID = 0x2886
RESPEAKER_PID = 0x0018
RESPEAKER_NAME_HINTS = (
    "ReSpeaker 4 Mic Array",
    "ReSpeaker",
    "UAC1.0",
)

PARAMETERS = {
    "AGCONOFF": (19, 0, "int", 1, 0, "rw"),
    "VOICEACTIVITY": (19, 32, "int", 1, 0, "ro"),
    "SPEECHDETECTED": (19, 33, "int", 1, 0, "ro"),
    "GAMMAVAD_SR": (19, 39, "float", 1000, 0, "rw"),
    "DOAANGLE": (21, 0, "int", 359, 0, "ro"),
}


class ReSpeakerUSBError(RuntimeError):
    pass


@dataclass
class TuningSnapshot:
    doa_angle_deg: Optional[float]
    voice_activity: Optional[bool]
    speech_detected: Optional[bool]


class ReSpeakerTuning:
    """USB control wrapper for the Seeed ReSpeaker USB Mic Array."""

    TIMEOUT_MS = 100000

    def __init__(self, dev):
        self.dev = dev

    @classmethod
    def find(cls, vid: int = RESPEAKER_VID, pid: int = RESPEAKER_PID) -> "ReSpeakerTuning":
        if usb is None:
            raise ReSpeakerUSBError("pyusb is not installed. Install with: pip install pyusb")

        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            raise ReSpeakerUSBError(
                "ReSpeaker USB Mic Array not found over USB. "
                "Check the cable, permissions, and that the board is powered."
            )
        return cls(dev)

    def _read(self, name: str):
        if usb is None:
            raise ReSpeakerUSBError("pyusb is not installed.")
        if name not in PARAMETERS:
            raise KeyError(f"Unknown tuning parameter: {name}")

        group_id, offset, data_type, *_ = PARAMETERS[name]
        cmd = 0x80 | offset
        length = 8
        if data_type == "int":
            cmd |= 0x40

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmd,
            group_id,
            length,
            self.TIMEOUT_MS,
        )

        value_a, value_b = struct.unpack("ii", bytes(response))
        if data_type == "int":
            return value_a
        return value_a * (2.0 ** value_b)

    def _write(self, name: str, value):
        if usb is None:
            raise ReSpeakerUSBError("pyusb is not installed.")
        if name not in PARAMETERS:
            raise KeyError(f"Unknown tuning parameter: {name}")

        group_id, offset, data_type, *_ = PARAMETERS[name]
        if data_type == "int":
            payload = struct.pack("iii", offset, int(value), 1)
        else:
            payload = struct.pack("ifi", offset, float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            0,
            group_id,
            payload,
            self.TIMEOUT_MS,
        )

    @property
    def doa_angle(self) -> int:
        return int(self._read("DOAANGLE"))

    @property
    def voice_activity(self) -> bool:
        return bool(self._read("VOICEACTIVITY"))

    @property
    def speech_detected(self) -> bool:
        return bool(self._read("SPEECHDETECTED"))

    def set_vad_threshold(self, threshold_db: float) -> None:
        self._write("GAMMAVAD_SR", float(threshold_db))

    def set_agc_enabled(self, enabled: bool) -> None:
        self._write("AGCONOFF", 1 if enabled else 0)

    def snapshot(self) -> TuningSnapshot:
        doa = None
        vad = None
        speech = None

        try:
            doa = float(self.doa_angle)
        except Exception:
            pass

        try:
            vad = bool(self.voice_activity)
        except Exception:
            pass

        try:
            speech = bool(self.speech_detected)
        except Exception:
            pass

        return TuningSnapshot(doa_angle_deg=doa, voice_activity=vad, speech_detected=speech)

    def close(self) -> None:
        if usb is not None:
            usb.util.dispose_resources(self.dev)


class PixelRing:
    """LED Helper"""

    TIMEOUT_MS = 8000

    def __init__(self, dev):
        self.dev = dev

    @classmethod
    def find(cls, vid: int = RESPEAKER_VID, pid: int = RESPEAKER_PID) -> "PixelRing":
        if usb is None:
            raise ReSpeakerUSBError("pyusb is not installed. Install with: pip install pyusb")

        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            raise ReSpeakerUSBError("ReSpeaker USB Mic Array not found over USB.")
        return cls(dev)

    def _write(self, command: int, data=None) -> None:
        if usb is None:
            raise ReSpeakerUSBError("pyusb is not installed.")
        if data is None:
            data = [0]

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            command,
            0x1C,
            data,
            self.TIMEOUT_MS,
        )

    def off(self) -> None:
        self.mono(0, 0, 0)

    def mono(self, red: int, green: int, blue: int) -> None:
        self._write(1, [red & 0xFF, green & 0xFF, blue & 0xFF, 0])

    def listen(self) -> None:
        self._write(2)

    def think(self) -> None:
        self._write(4)

    def spin(self) -> None:
        self._write(5)

    def set_brightness(self, brightness: int) -> None:
        brightness = max(0, min(0x1F, int(brightness)))
        self._write(0x20, [brightness])

    def show_direction(self, angle_deg: float, red: int = 0, green: int = 255, blue: int = 0) -> None:
        led_index = int(round((angle_deg % 360.0) / 30.0)) % 12
        data = []
        for i in range(12):
            if i == led_index:
                data.extend([red & 0xFF, green & 0xFF, blue & 0xFF, 0])
            else:
                data.extend([0, 0, 0, 0])
        self._write(6, data)

    def close(self) -> None:
        if usb is not None:
            usb.util.dispose_resources(self.dev)


def circular_mean_deg(angles_deg: list[float]) -> Optional[float]:
    if not angles_deg:
        return None

    x = sum(math.cos(math.radians(a)) for a in angles_deg)
    y = sum(math.sin(math.radians(a)) for a in angles_deg)

    if abs(x) < 1e-12 and abs(y) < 1e-12:
        return None

    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def find_respeaker_input_device_index(pyaudio_instance) -> int:
    """Find the ReSpeaker input device index from PyAudio."""
    host_info = pyaudio_instance.get_host_api_info_by_index(0)
    device_count = int(host_info.get("deviceCount", 0))

    for i in range(device_count):
        info = pyaudio_instance.get_device_info_by_host_api_device_index(0, i)
        name = str(info.get("name", ""))
        max_inputs = int(info.get("maxInputChannels", 0))
        if max_inputs <= 0:
            continue
        if any(hint.lower() in name.lower() for hint in RESPEAKER_NAME_HINTS):
            return i

    raise ReSpeakerUSBError(
        "Could not find a ReSpeaker input device in PyAudio. "
        "Run a device list first and verify the board is connected."
    )