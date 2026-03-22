from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from typing import Callable, Optional

import numpy as np
import pyaudio

try:
    from .respeaker_usb import (
        PixelRing,
        ReSpeakerTuning,
        ReSpeakerUSBError,
        circular_mean_deg,
        find_respeaker_input_device_index,
    )
except ImportError:
    from respeaker_usb import (
        PixelRing,
        ReSpeakerTuning,
        ReSpeakerUSBError,
        circular_mean_deg,
        find_respeaker_input_device_index,
    )


@dataclass
class AudioConfig:
    rate: int = 16000
    chunk: int = 1024
    firmware_channels: int = 6  # 6 for factory firmware, 1 for 1-channel firmware
    detect_channel: int = 0     # channel 0 is processed audio in 6-channel firmware
    input_device_index: Optional[int] = None


@dataclass
class WhistleConfig:
    band_min_hz: float = 1800.0
    band_max_hz: float = 4500.0
    min_rms: float = 700.0
    min_band_ratio: float = 0.45
    min_peak_ratio: float = 0.18
    min_prominence_db: float = 12.0
    max_peak_drift_hz: float = 350.0
    consecutive_hits_required: int = 3
    cooldown_s: float = 1.0
    doa_samples: int = 5
    doa_sample_delay_s: float = 0.03
    use_respeaker_usb: bool = True
    use_led_ring: bool = False


@dataclass
class WhistleEvent:
    timestamp: float
    peak_freq_hz: float
    rms: float
    band_ratio: float
    peak_ratio: float
    prominence_db: float
    doa_angle_deg: Optional[float]
    voice_activity: Optional[bool]
    speech_detected: Optional[bool]


class NarrowBandWhistleDetector:
    """A whistle detector."""

    def __init__(self, audio_cfg: AudioConfig, whistle_cfg: WhistleConfig):
        self.audio_cfg = audio_cfg
        self.whistle_cfg = whistle_cfg
        self.window = np.hanning(audio_cfg.chunk).astype(np.float32)
        self.freqs = np.fft.rfftfreq(audio_cfg.chunk, d=1.0 / audio_cfg.rate)
        self.band_mask = (self.freqs >= whistle_cfg.band_min_hz) & (self.freqs <= whistle_cfg.band_max_hz)
        self.total_mask = (self.freqs >= 200.0) & (self.freqs <= 6000.0)

        self._consecutive_hits = 0
        self._last_peak_freq_hz: Optional[float] = None
        self._last_event_time = 0.0

    def process_int16(self, samples: np.ndarray) -> tuple[bool, dict]:
        x = samples.astype(np.float32)
        rms = float(np.sqrt(np.mean(np.square(x)) + 1e-12))

        if x.size != self.audio_cfg.chunk:
            raise ValueError(f"Expected {self.audio_cfg.chunk} samples, got {x.size}")

        spectrum = np.abs(np.fft.rfft(x * self.window)) ** 2
        band_power = float(np.sum(spectrum[self.band_mask]) + 1e-12)
        total_power = float(np.sum(spectrum[self.total_mask]) + 1e-12)
        band_ratio = band_power / total_power

        band_spec = spectrum[self.band_mask]
        band_freqs = self.freqs[self.band_mask]
        peak_idx = int(np.argmax(band_spec))
        peak_power = float(band_spec[peak_idx] + 1e-12)
        peak_freq_hz = float(band_freqs[peak_idx])
        peak_ratio = peak_power / band_power
        prominence_db = float(10.0 * np.log10(peak_power / (np.median(band_spec) + 1e-12)))

        stable_peak = True
        if self._last_peak_freq_hz is not None:
            stable_peak = abs(peak_freq_hz - self._last_peak_freq_hz) <= self.whistle_cfg.max_peak_drift_hz

        candidate = (
            rms >= self.whistle_cfg.min_rms
            and band_ratio >= self.whistle_cfg.min_band_ratio
            and peak_ratio >= self.whistle_cfg.min_peak_ratio
            and prominence_db >= self.whistle_cfg.min_prominence_db
            and stable_peak
        )

        if candidate:
            self._consecutive_hits += 1
        else:
            self._consecutive_hits = 0

        self._last_peak_freq_hz = peak_freq_hz

        now = time.time()
        confirmed = (
            self._consecutive_hits >= self.whistle_cfg.consecutive_hits_required
            and (now - self._last_event_time) >= self.whistle_cfg.cooldown_s
        )

        if confirmed:
            self._last_event_time = now
            self._consecutive_hits = 0

        features = {
            "rms": rms,
            "band_ratio": band_ratio,
            "peak_ratio": peak_ratio,
            "prominence_db": prominence_db,
            "peak_freq_hz": peak_freq_hz,
        }
        return confirmed, features


class ReSpeakerWhistleTracker:
    """Reads audio from the ReSpeaker and emits whistle events."""

    def __init__(self, audio_cfg: AudioConfig, whistle_cfg: WhistleConfig):
        self.audio_cfg = audio_cfg
        self.whistle_cfg = whistle_cfg
        self.detector = NarrowBandWhistleDetector(audio_cfg, whistle_cfg)

        self.pya = pyaudio.PyAudio()
        if self.audio_cfg.input_device_index is None:
            self.audio_cfg.input_device_index = find_respeaker_input_device_index(self.pya)

        self.tuning: Optional[ReSpeakerTuning] = None
        self.pixel_ring: Optional[PixelRing] = None

        if whistle_cfg.use_respeaker_usb:
            try:
                self.tuning = ReSpeakerTuning.find()
            except Exception:
                self.tuning = None

        if whistle_cfg.use_led_ring and whistle_cfg.use_respeaker_usb:
            try:
                self.pixel_ring = PixelRing.find()
                self.pixel_ring.listen()
            except Exception:
                self.pixel_ring = None

    def _extract_detection_channel(self, data: bytes) -> np.ndarray:
        samples = np.frombuffer(data, dtype=np.int16)
        if self.audio_cfg.firmware_channels == 1:
            return samples
        return samples[self.audio_cfg.detect_channel :: self.audio_cfg.firmware_channels]

    def _sample_direction(self) -> tuple[Optional[float], Optional[bool], Optional[bool]]:
        if self.tuning is None:
            return None, None, None

        doa_samples: list[float] = []
        for _ in range(self.whistle_cfg.doa_samples):
            try:
                doa_samples.append(float(self.tuning.doa_angle))
            except Exception:
                pass
            time.sleep(self.whistle_cfg.doa_sample_delay_s)

        doa_angle = circular_mean_deg(doa_samples)

        try:
            voice_activity = bool(self.tuning.voice_activity)
        except Exception:
            voice_activity = None

        try:
            speech_detected = bool(self.tuning.speech_detected)
        except Exception:
            speech_detected = None

        return doa_angle, voice_activity, speech_detected

    def run(self, on_event: Callable[[WhistleEvent], None]) -> None:
        stream = self.pya.open(
            format=pyaudio.paInt16,
            channels=self.audio_cfg.firmware_channels,
            rate=self.audio_cfg.rate,
            input=True,
            input_device_index=self.audio_cfg.input_device_index,
            frames_per_buffer=self.audio_cfg.chunk,
        )

        print(f"Using PyAudio device index: {self.audio_cfg.input_device_index}")
        print(
            f"Firmware channels: {self.audio_cfg.firmware_channels}, "
            f"detection channel: {self.audio_cfg.detect_channel}"
        )
        print("Listening for whistles... Ctrl+C to stop.")

        try:
            while True:
                raw = stream.read(self.audio_cfg.chunk, exception_on_overflow=False)
                detect_samples = self._extract_detection_channel(raw)
                confirmed, features = self.detector.process_int16(detect_samples)

                if confirmed:
                    doa_angle, voice_activity, speech_detected = self._sample_direction()
                    event = WhistleEvent(
                        timestamp=time.time(),
                        peak_freq_hz=float(features["peak_freq_hz"]),
                        rms=float(features["rms"]),
                        band_ratio=float(features["band_ratio"]),
                        peak_ratio=float(features["peak_ratio"]),
                        prominence_db=float(features["prominence_db"]),
                        doa_angle_deg=doa_angle,
                        voice_activity=voice_activity,
                        speech_detected=speech_detected,
                    )

                    if self.pixel_ring is not None:
                        try:
                            if doa_angle is not None:
                                self.pixel_ring.show_direction(doa_angle)
                            else:
                                self.pixel_ring.mono(0, 0, 255)
                        except Exception:
                            pass

                    on_event(event)
        finally:
            try:
                stream.stop_stream()
                stream.close()
            finally:
                self.pya.terminate()
                if self.pixel_ring is not None:
                    try:
                        self.pixel_ring.off()
                    except Exception:
                        pass
                if self.tuning is not None:
                    self.tuning.close()


def print_event(event: WhistleEvent) -> None:
    print(json.dumps(asdict(event)))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ReSpeaker whistle detector")
    parser.add_argument("--device-index", type=int, default=-1, help="PyAudio input device index")
    parser.add_argument("--rate", type=int, default=16000)
    parser.add_argument("--chunk", type=int, default=1024)
    parser.add_argument("--firmware-channels", type=int, default=6, choices=[1, 6])
    parser.add_argument("--detect-channel", type=int, default=0)
    parser.add_argument("--band-min-hz", type=float, default=1800.0)
    parser.add_argument("--band-max-hz", type=float, default=4500.0)
    parser.add_argument("--min-rms", type=float, default=700.0)
    parser.add_argument("--min-band-ratio", type=float, default=0.45)
    parser.add_argument("--min-peak-ratio", type=float, default=0.18)
    parser.add_argument("--min-prominence-db", type=float, default=12.0)
    parser.add_argument("--max-peak-drift-hz", type=float, default=350.0)
    parser.add_argument("--consecutive-hits-required", type=int, default=3)
    parser.add_argument("--cooldown-s", type=float, default=1.0)
    parser.add_argument("--doa-samples", type=int, default=5)
    parser.add_argument("--doa-sample-delay-s", type=float, default=0.03)
    parser.add_argument("--disable-usb-control", action="store_true")
    parser.add_argument("--use-led-ring", action="store_true")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    audio_cfg = AudioConfig(
        rate=args.rate,
        chunk=args.chunk,
        firmware_channels=args.firmware_channels,
        detect_channel=args.detect_channel,
        input_device_index=None if args.device_index < 0 else args.device_index,
    )

    whistle_cfg = WhistleConfig(
        band_min_hz=args.band_min_hz,
        band_max_hz=args.band_max_hz,
        min_rms=args.min_rms,
        min_band_ratio=args.min_band_ratio,
        min_peak_ratio=args.min_peak_ratio,
        min_prominence_db=args.min_prominence_db,
        max_peak_drift_hz=args.max_peak_drift_hz,
        consecutive_hits_required=args.consecutive_hits_required,
        cooldown_s=args.cooldown_s,
        doa_samples=args.doa_samples,
        doa_sample_delay_s=args.doa_sample_delay_s,
        use_respeaker_usb=not args.disable_usb_control,
        use_led_ring=args.use_led_ring,
    )

    tracker = ReSpeakerWhistleTracker(audio_cfg, whistle_cfg)
    tracker.run(print_event)


if __name__ == "__main__":
    main()