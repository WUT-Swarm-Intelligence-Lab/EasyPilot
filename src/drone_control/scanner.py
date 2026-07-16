from __future__ import annotations

import logging
from dataclasses import dataclass

import cflib
from cflib.crtp import scan_interfaces

from drone_control.models import DroneInfo

logger = logging.getLogger(__name__)


@dataclass
class Scanner:
    def scan(self) -> list[DroneInfo]:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        logger.info("Scanning for Crazyflie devices...")

        found_uris = scan_interfaces()
        drones = []

        for entry in found_uris:
            uri = entry[0] if isinstance(entry, list) else entry
            if not uri:
                continue
            info = DroneInfo(uri=uri)
            drones.append(info)
            logger.info("Found drone: %s", uri)

        logger.info("Scan complete: %d drone(s) found.", len(drones))
        return drones
