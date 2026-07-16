from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_control.connection import DroneConnection

from drone_control.connection import DroneConnection as _DroneConnection
from cflib.positioning.position_hl_commander import PositionHlCommander
from drone_control.controller import DroneController
from drone_control.models import DroneInfo, Position, Rotation
from drone_control.scanner import Scanner

logger = logging.getLogger(__name__)

ARRIVAL_THRESHOLD = 0.15


class Drone:
    CONTROLLER_PID = PositionHlCommander.CONTROLLER_PID
    CONTROLLER_MELLINGER = PositionHlCommander.CONTROLLER_MELLINGER

    def __init__(
        self,
        uri: str | None = None,
        takeoff_height: float = 0.5,
        controller: int | None = None,
    ):
        self._conn: DroneConnection | None = None
        self._ctrl: DroneController | None = None
        self._current_waypoint: Position | None = None
        self._takeoff_height = takeoff_height
        self._controller = controller
        if uri:
            self.connect(uri)

    def find(self, i : int) -> list[DroneInfo]:
        if i is None:
            return

        drones = Scanner().scan()
        if i < 0 or i >= len(drones):
            raise IndexError(f"Drone index {i} is out of range.")
        self.connect(drones[i].uri)

    def connect(self, uri: str) -> None:
        self._conn = _DroneConnection(uri=uri)
        self._conn.connect()
        self._ctrl = DroneController(
            connection=self._conn,
            takeoff_height=self._takeoff_height,
            controller=self._controller,
        )

    def disconnect(self) -> None:
        if self._conn is not None:
            self._conn.disconnect()
            self._conn = None
            self._ctrl = None

    def takeoff(self, height: float | None = None) -> None:
        self._ctrl.takeoff(height)

    def goto(self, waypoint: list[float] | Position, yaw: float = 0.0) -> None:
        if isinstance(waypoint, Position):
            pos = waypoint
        else:
            pos = Position(x=waypoint[0], y=waypoint[1], z=waypoint[2])
        self._current_waypoint = pos
        self._ctrl.set_waypoint(pos, Rotation(yaw=yaw))

    def is_moving(self) -> bool:
        if self._current_waypoint is None:
            return False
        pos = self._ctrl.get_position()
        return not self._reached(pos, self._current_waypoint)

    def wait(self, poll: float = 0.2) -> None:
        while self.is_moving():
            time.sleep(poll)

    def land(self) -> None:
        self._ctrl.land()

    def stop(self) -> None:
        self._ctrl.stop()

    @property
    def position(self) -> Position:
        return self._ctrl.get_position()

    def _reached(self, current: Position, target: Position) -> bool:
        dx = current.x - target.x
        dy = current.y - target.y
        dz = current.z - target.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) < ARRIVAL_THRESHOLD

    def __enter__(self) -> Drone:
        return self

    def __exit__(self, *args: object) -> None:
        self.disconnect()
