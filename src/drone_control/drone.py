from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable

    from drone_control.connection import DroneConnection

import numpy as np
from drone_control.camera import WifiCamera
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
        interpolate: bool = False,
        max_dist: float = 0.1,
        camera_ip: str | None = None,
        camera_port: int = 5000,
        forward_fly: bool = False,
    ):
        self._conn: DroneConnection | None = None
        self._ctrl: DroneController | None = None
        self._current_waypoint: Position | None = None
        self._takeoff_height = takeoff_height
        self._controller = controller
        self._interpolate = interpolate
        self._max_dist = max_dist
        self._camera: WifiCamera | None = None
        self._camera_ip = camera_ip
        self._camera_port = camera_port
        self._forward_fly = forward_fly
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
        self.camera_stop()
        if self._conn is not None:
            self._conn.disconnect()
            self._conn = None
            self._ctrl = None

    def camera_feed(self, callback: Callable[[np.ndarray], None]) -> None:
        if self._camera_ip is None:
            raise RuntimeError("No camera_ip set. Pass camera_ip to Drone().")
        if self._camera is None:
            self._camera = WifiCamera(
                ip=self._camera_ip, port=self._camera_port
            )
        self._camera.start(callback)

    def camera_wait_until_ready(self, timeout: float | None = None) -> bool:
        if self._camera is None:
            return False
        return self._camera.wait_until_ready(timeout=timeout)

    def camera_stop(self) -> None:
        if self._camera is not None:
            self._camera.stop()
            self._camera = None

    def takeoff(self, height: float | None = None) -> None:
        self._ctrl.takeoff(height)

    def set_forward_fly(self, enabled: bool) -> None:
        self._forward_fly = enabled

    def goto(self, waypoint: list[float] | Position, yaw: float = 0.0) -> None:
        if isinstance(waypoint, Position):
            target = waypoint
        else:
            target = Position(x=waypoint[0], y=waypoint[1], z=waypoint[2])

        start = self._ctrl.get_position()

        if self._forward_fly:
            yaw = math.atan2(target.y - start.y, target.x - start.x)

        points = self._interpolate_path(start, target, self._max_dist)

        for point in points:
            self._current_waypoint = point
            self._ctrl.set_waypoint(point, Rotation(yaw=yaw))

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

    def _interpolate_path(
        self, start: Position, end: Position, max_dist: float
    ) -> list[Position]:
        dist = start.distance_to(end)
        if dist <= 0.0:
            return [end]

        num_segments = max(1, round(dist / max_dist))
        points = []
        for i in range(1, num_segments + 1):
            t = i / num_segments
            points.append(
                Position(
                    x=start.x + (end.x - start.x) * t,
                    y=start.y + (end.y - start.y) * t,
                    z=start.z + (end.z - start.z) * t,
                )
            )
        return points

    def _reached(self, current: Position, target: Position) -> bool:
        return current.distance_to(target) < ARRIVAL_THRESHOLD

    def __enter__(self) -> Drone:
        return self

    def __exit__(self, *args: object) -> None:
        self.disconnect()
