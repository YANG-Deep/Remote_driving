"""Bounded exact-frame matching; callbacks may arrive late or out of order."""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


class FrameSynchronizer:
    def __init__(self, keys: tuple[str, ...], capacity: int = 8):
        if capacity < 1 or not keys:
            raise ValueError("nonempty keys and positive capacity required")
        self._pending = {key: {} for key in keys}
        self._capacity = capacity
        self._last_frame = -1

    def push(self, key: str, measurement) -> None:
        if measurement.frame <= self._last_frame:
            return
        pending = self._pending[key]
        pending[measurement.frame] = measurement
        while len(pending) > self._capacity:
            del pending[min(pending)]

    def pop_common(self) -> dict | None:
        common = set.intersection(*(set(frames) for frames in self._pending.values()))
        if not common:
            return None
        frame = max(common)
        result = {key: frames[frame] for key, frames in self._pending.items()}
        for frames in self._pending.values():
            for old in [number for number in frames if number <= frame]:
                del frames[old]
        self._last_frame = frame
        return result


@dataclass(frozen=True)
class FrameBundle:
    frame: int
    timestamp: float
    images: dict[str, np.ndarray]  # BGR; never RGB internally.
    transforms: dict[str, object]
