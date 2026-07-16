from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

from cflib.crazyflie import Crazyflie

logger = logging.getLogger(__name__)


@dataclass
class DroneConnection:
    uri: str
    _cf: Crazyflie = field(init=False, repr=False)
    _connected: bool = field(default=False, init=False, repr=False)
    _connected_event: threading.Event = field(default_factory=threading.Event, init=False, repr=False)

    def __post_init__(self) -> None:
        self._cf = Crazyflie()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def crazyflie(self) -> Crazyflie:
        return self._cf

    def connect(self, timeout: float = 10.0) -> None:
        if self._connected:
            logger.warning("Already connected to %s", self.uri)
            return

        logger.info("Connecting to %s...", self.uri)

        self._cf.connected.add_callback(self._on_connected)
        self._cf.disconnected.add_callback(self._on_disconnected)
        self._cf.connection_failed.add_callback(self._on_connection_failed)

        self._cf.open_link(self.uri)

        if not self._connected_event.wait(timeout=timeout):
            raise TimeoutError(f"Connection to {self.uri} timed out after {timeout}s")

        logger.info("Connected to %s", self.uri)

    def disconnect(self) -> None:
        if not self._connected:
            return

        logger.info("Disconnecting from %s...", self.uri)
        self._cf.close_link()
        self._connected = False
        logger.info("Disconnected from %s", self.uri)

    def _on_connected(self, uri: str) -> None:
        self._connected = True
        self._connected_event.set()
        logger.info("Link connected: %s", uri)

    def _on_disconnected(self, uri: str) -> None:
        self._connected = False
        logger.info("Link disconnected: %s", uri)

    def _on_connection_failed(self, uri: str, msg: str) -> None:
        self._connected = False
        self._connected_event.set()
        logger.error("Connection failed to %s: %s", uri, msg)

    def __enter__(self) -> DroneConnection:
        self.connect()
        return self

    def __exit__(self, *args: object) -> None:
        self.disconnect()
