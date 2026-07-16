from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Rotation:
    yaw: float
    pitch: float = 0.0
    roll: float = 0.0


@dataclass(frozen=True)
class DroneInfo:
    uri: str
    name: str | None = None
    address: str | None = None
