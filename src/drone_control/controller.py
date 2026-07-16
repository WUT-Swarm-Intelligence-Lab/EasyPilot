from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.reset_estimator import reset_estimator

from drone_control.connection import DroneConnection
from drone_control.models import Position, Rotation

logger = logging.getLogger(__name__)

DEFAULT_HEIGHT = 0.5
DEFAULT_VELOCITY = 0.5


@dataclass
class DroneController:
    connection: DroneConnection
    takeoff_height: float = DEFAULT_HEIGHT
    velocity: float = DEFAULT_VELOCITY
    controller: int | None = None
    _phc: PositionHlCommander | None = None
    _position: Position = field(default_factory=lambda: Position(x=0.0, y=0.0, z=0.0))
    _position_log: LogConfig | None = field(default=None, init=False, repr=False)

    def takeoff(self, height: float | None = None) -> None:
        h = height or self.takeoff_height
        logger.info("Taking off to %.2fm...", h)
        cf = self.connection.crazyflie
        reset_estimator(cf)
        cf.supervisor.send_arming_request(True)
        time.sleep(1.0)
        self._phc = PositionHlCommander(
            cf,
            default_height=h,
            default_velocity=self.velocity,
            controller=self.controller,
        )
        self._start_position_logging()
        self._phc.take_off()
        time.sleep(h / self.velocity + 0.5)
        logger.info("Takeoff complete.")

    def land(self) -> None:
        if self._phc is None:
            logger.warning("Not in flight, nothing to land.")
            return

        logger.info("Landing...")
        self._phc.land()
        time.sleep(1.0)
        self._stop_position_logging()
        self._phc = None
        logger.info("Landed.")

    def get_position(self) -> Position:
        return self._position

    def set_waypoint(
        self,
        position: Position,
        rotation: Rotation | None = None,
    ) -> None:
        if self._phc is None:
            raise RuntimeError("Drone is not flying. Call takeoff() first.")

        yaw = rotation.yaw if rotation else 0.0
        logger.info(
            "Setting waypoint x=%.2f, y=%.2f, z=%.2f, yaw=%.1f",
            position.x,
            position.y,
            position.z,
            yaw,
        )
        if yaw != 0.0:
            pos = self._phc.get_position()
            dx = position.x - pos[0]
            dy = position.y - pos[1]
            dz = position.z - pos[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            duration_s = max(distance / self._phc._default_velocity, 0.01)
            self._phc._hl_commander.go_to(
                position.x, position.y, position.z, yaw, duration_s
            )
        else:
            self._phc.go_to(position.x, position.y, position.z)

    def goto(
        self,
        position: Position,
        rotation: Rotation | None = None,
    ) -> None:
        self.set_waypoint(position, rotation)

    def move(self, dx: float = 0, dy: float = 0, dz: float = 0, dyaw: float = 0) -> None:
        if self._phc is None:
            raise RuntimeError("Drone is not flying. Call takeoff() first.")

        logger.info("Moving by dx=%.2f, dy=%.2f, dz=%.2f, dyaw=%.1f", dx, dy, dz, dyaw)
        self._phc.move_distance(dx, dy, dz, 0.2)
        self._phc.turn_left(dyaw)

    def hover(self, seconds: float = 1.0) -> None:
        if self._phc is None:
            return
        logger.info("Hovering for %.1fs...", seconds)
        time.sleep(seconds)

    def stop(self) -> None:
        if self._phc is not None:
            self._stop_position_logging()
            self._phc.stop()
            self._phc = None
        logger.info("Emergency stop.")

    def _start_position_logging(self) -> None:
        self._position_log = LogConfig(name="position", period_in_ms=10)
        self._position_log.add_variable("stateEstimate.x", "float")
        self._position_log.add_variable("stateEstimate.y", "float")
        self._position_log.add_variable("stateEstimate.z", "float")
        self._position_log.data_received_cb.add_callback(self._on_position_data)
        self.connection.crazyflie.log.add_config(self._position_log)
        self._position_log.start()

    def _stop_position_logging(self) -> None:
        if self._position_log is not None:
            self._position_log.stop()
            self._position_log.delete()
            self._position_log = None

    def _on_position_data(self, timestamp: int, data: dict, logblock: object) -> None:
        self._position = Position(
            x=data["stateEstimate.x"],
            y=data["stateEstimate.y"],
            z=data["stateEstimate.z"],
        )
