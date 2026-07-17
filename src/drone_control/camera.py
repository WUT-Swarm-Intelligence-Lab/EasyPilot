from __future__ import annotations

import logging
import socket
import struct
import threading
from typing import Callable

import cv2
import numpy as np

logger = logging.getLogger(__name__)

DEFAULT_IP = "192.168.4.1"
DEFAULT_PORT = 5000

MAGIC = 0xBC
FORMAT_RAW = 0
FORMAT_JPEG = 1

RAW_WIDTH = 324
RAW_HEIGHT = 244

from drone_control.flightReady import FlightReady, requires_flight_ready
class WifiCamera(FlightReady):

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT):
        self._ip = ip
        self._port = port
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._callback: Callable[[np.ndarray], None] | None = None
        self._ready = threading.Event()
        super().__init__()

    def start(self, callback: Callable[[np.ndarray], None]) -> None:
        if self._running:
            return
        self._callback = callback
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    @property
    def is_running(self) -> bool:
        return self._running

    def wait_until_ready(self, timeout: float | None = None) -> bool:
        return self._ready.wait(timeout=timeout)

    def stop(self) -> None:
        self._running = False
        self._ready.clear()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(5.0)
        logger.info("Connecting to camera at %s:%d ...", self._ip, self._port)
        self._sock.connect((self._ip, self._port))
        logger.info("Camera connected.")
        self._sock.settimeout(10.0)
        self._ready.set()

    def _rx_bytes(self, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = self._sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Camera socket closed")
            data.extend(chunk)
        return bytes(data)

    def _read_packet_header(self) -> tuple[int, int, int]:
        raw = self._rx_bytes(4)
        length, routing, function = struct.unpack("<HBB", raw)
        return length, routing, function

    def _read_image_header(self, length: int) -> tuple[int, int, int, int, int, int]:
        raw = self._rx_bytes(length - 2)
        magic, width, height, depth, fmt, size = struct.unpack("<BHHBBI", raw)
        if magic != MAGIC:
            raise ValueError(f"Bad magic: 0x{magic:02X}, expected 0x{MAGIC:02X}")
        return magic, width, height, depth, fmt, size

    def _read_image_data(self, total_size: int) -> bytearray:
        img_stream = bytearray()
        while len(img_stream) < total_size:
            length, _dst, _src = struct.unpack("<HBB", self._rx_bytes(4))
            chunk = self._rx_bytes(length - 2)
            img_stream.extend(chunk)
        return img_stream

    def _decode_frame(self, img_data: bytearray, fmt: int, width: int, height: int) -> np.ndarray:
        if fmt == FORMAT_JPEG:
            buf = np.frombuffer(img_data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("Failed to decode JPEG frame")
            return img
        elif fmt == FORMAT_RAW:
            bayer = np.frombuffer(img_data, dtype=np.uint8).reshape((height, width))
            return bayer
        else:
            raise ValueError(f"Unknown image format: {fmt}")

    def _stream_loop(self) -> None:
        self.remove_before_flight()
        while self._running:
            try:
                if self._sock is None:
                    self._connect()

                length, _routing, _function = self._read_packet_header()
                _magic, width, height, _depth, fmt, size = self._read_image_header(length)
                img_data = self._read_image_data(size)

                frame = self._decode_frame(img_data, fmt, width, height)

                if self._callback:
                    self._callback(frame)

            except (ConnectionError, OSError, ValueError) as e:
                if self._running:
                    logger.warning("Camera stream error: %s, reconnecting...", e)
                    self._sock = None
                    self._ready.clear()

    def __enter__(self) -> WifiCamera:
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()
