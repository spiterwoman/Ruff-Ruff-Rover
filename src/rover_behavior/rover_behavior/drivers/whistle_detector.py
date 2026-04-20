from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import pyaudio

try:
    from .xvf3800_usb import XVF3800Tuning, circular_mean_deg, find_xvf3800_input_device_index
except ImportError:
    from xvf3800_usb import XVF3800Tuning, circular_mean_deg, find_xvf3800_input_device_index


@dataclass
class AudioConfig:
    rate: int = 16000
    chunk: int = 1024
    channels: int = 2
    detect_channel: int = 0
    input_device_index: Optional[int] = None


@dataclass
class WhistleConfig:
    use_frequency_gate: bool = False
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
    use_usb_control: bool = True


@dataclass
class WhistleReading:
    timestamp: float
    peak_freq_hz: float
    confidence: float
    doa_deg: float


class NarrowBandWhistleDetector:
    def __init__(self, audio_cfg: AudioConfig, whistle_cfg: WhistleConfig) -> None:
        self.audio_cfg = audio_cfg
        self.whistle_cfg = whistle_cfg
        self.window = np.hanning(audio_cfg.chunk).astype(np.float32)
        self.freqs = np.fft.rfftfreq(audio_cfg.chunk, d=1.0 / audio_cfg.rate)
        self.total_mask = (self.freqs >= 200.0) & (self.freqs <= 6000.0)
        if whistle_cfg.use_frequency_gate:
            self.band_mask = (self.freqs >= whistle_cfg.band_min_hz) & (self.freqs <= whistle_cfg.band_max_hz)
        else:
            self.band_mask = self.total_mask.copy()
        self._consecutive_hits = 0
        self._last_peak_freq_hz: Optional[float] = None
        self._last_event_time = 0.0

    def _normalized_score(self, value: float, threshold: float, margin: float) -> float:
        return float(np.clip((value - threshold) / max(margin, 1e-6), 0.0, 1.0))

    def process_int16(self, samples: np.ndarray) -> tuple[bool, dict]:
        signal = samples.astype(np.float32)
        if signal.size != self.audio_cfg.chunk:
            raise ValueError(f"expected {self.audio_cfg.chunk} samples, got {signal.size}")

        rms = float(np.sqrt(np.mean(np.square(signal)) + 1e-12))
        spectrum = np.abs(np.fft.rfft(signal * self.window)) ** 2
        band_power = float(np.sum(spectrum[self.band_mask]) + 1e-12)
        total_power = float(np.sum(spectrum[self.total_mask]) + 1e-12)
        band_ratio = band_power / total_power

        band_spectrum = spectrum[self.band_mask]
        band_freqs = self.freqs[self.band_mask]
        peak_index = int(np.argmax(band_spectrum))
        peak_power = float(band_spectrum[peak_index] + 1e-12)
        peak_freq_hz = float(band_freqs[peak_index])
        peak_ratio = peak_power / band_power
        prominence_db = float(10.0 * np.log10(peak_power / (np.median(band_spectrum) + 1e-12)))

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

        self._consecutive_hits = self._consecutive_hits + 1 if candidate else 0
        self._last_peak_freq_hz = peak_freq_hz

        now = time.time()
        confirmed = (
            self._consecutive_hits >= self.whistle_cfg.consecutive_hits_required
            and (now - self._last_event_time) >= self.whistle_cfg.cooldown_s
        )

        if confirmed:
            self._last_event_time = now
            self._consecutive_hits = 0

        confidence = np.mean(
            [
                self._normalized_score(rms, self.whistle_cfg.min_rms, self.whistle_cfg.min_rms * 0.6),
                self._normalized_score(
                    band_ratio,
                    self.whistle_cfg.min_band_ratio,
                    1.0 - self.whistle_cfg.min_band_ratio,
                ),
                self._normalized_score(
                    peak_ratio,
                    self.whistle_cfg.min_peak_ratio,
                    1.0 - self.whistle_cfg.min_peak_ratio,
                ),
                self._normalized_score(
                    prominence_db,
                    self.whistle_cfg.min_prominence_db,
                    max(self.whistle_cfg.min_prominence_db, 6.0),
                ),
                1.0 if stable_peak else 0.0,
            ]
        )

        return confirmed, {
            "peak_freq_hz": peak_freq_hz,
            "confidence": float(np.clip(confidence, 0.0, 1.0)),
        }


class XVF3800WhistleTracker:
    def __init__(self, audio_cfg: AudioConfig, whistle_cfg: WhistleConfig) -> None:
        self.audio_cfg = audio_cfg
        self.whistle_cfg = whistle_cfg
        self.detector = NarrowBandWhistleDetector(audio_cfg, whistle_cfg)
        self.pya = pyaudio.PyAudio()
        if self.audio_cfg.input_device_index is None:
            self.audio_cfg.input_device_index = find_xvf3800_input_device_index(self.pya)
        self.tuning: Optional[XVF3800Tuning] = None
        if whistle_cfg.use_usb_control:
            self.tuning = XVF3800Tuning.find()

    def _extract_detection_channel(self, raw: bytes) -> np.ndarray:
        samples = np.frombuffer(raw, dtype=np.int16)
        if self.audio_cfg.channels <= 1:
            return samples
        return samples[self.audio_cfg.detect_channel :: self.audio_cfg.channels]

    def sample_direction(self) -> tuple[Optional[float], Optional[bool]]:
        if self.tuning is None:
            return None, None
        doa_samples: list[float] = []
        vad_samples: list[bool] = []
        for _ in range(self.whistle_cfg.doa_samples):
            snapshot = self.tuning.snapshot
            if snapshot.doa_deg is not None:
                doa_samples.append(snapshot.doa_deg)
            if snapshot.voice_activity is not None:
                vad_samples.append(snapshot.voice_activity)
            time.sleep(self.whistle_cfg.doa_sample_delay_s)
        return circular_mean_deg(doa_samples), any(vad_samples) if vad_samples else None

    def run(
        self,
        on_event: Callable[[WhistleReading], None],
        on_ready: Callable[[bool], None],
    ) -> None:
        stream = self.pya.open(
            format=pyaudio.paInt16,
            channels=self.audio_cfg.channels,
            rate=self.audio_cfg.rate,
            input=True,
            input_device_index=self.audio_cfg.input_device_index,
            frames_per_buffer=self.audio_cfg.chunk,
        )
        ready_sent = False
        try:
            doa_deg, vad_active = self.sample_direction()
            ready_sent = doa_deg is not None and vad_active is not None
            on_ready(ready_sent)
            while True:
                raw = stream.read(self.audio_cfg.chunk, exception_on_overflow=False)
                detect_samples = self._extract_detection_channel(raw)
                confirmed, features = self.detector.process_int16(detect_samples)
                if not confirmed:
                    continue
                doa_deg, vad_active = self.sample_direction()
                if doa_deg is None:
                    if ready_sent:
                        ready_sent = False
                        on_ready(False)
                    continue
                if not ready_sent and vad_active is not None:
                    ready_sent = True
                    on_ready(True)
                on_event(
                    WhistleReading(
                        timestamp=time.time(),
                        peak_freq_hz=features["peak_freq_hz"],
                        confidence=features["confidence"],
                        doa_deg=doa_deg,
                    )
                )
        finally:
            try:
                stream.stop_stream()
                stream.close()
            finally:
                self.pya.terminate()
                if self.tuning is not None:
                    self.tuning.close()
