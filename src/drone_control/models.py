from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Position:
    x: float
    y: float
    z: float

    def distance_to(self, other: Position) -> float:
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


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
