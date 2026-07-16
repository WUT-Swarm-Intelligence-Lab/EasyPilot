#!/usr/bin/env python3
"""Example: scan, connect, and fly a Crazyflie through 4 waypoints."""

import logging

from drone_control import Drone

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    drone = Drone()
    drone.find(0)
    drone.takeoff(height=0.5)
    for wp in [
        [1.5, 0.0, 0.5],
        [1.5, 1.5, 0.5],
        [0.0, 1.5, 0.5],
        [0.0, 0.0, 0.5],
    ]:
        drone.goto(wp)
        drone.wait()
        print(f"  Reached {wp}")

    drone.land()


if __name__ == "__main__":
    main()
