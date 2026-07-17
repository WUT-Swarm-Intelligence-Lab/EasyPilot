#!/usr/bin/env python3
"""Example: test Drone.move() with a small square pattern."""

import logging

from drone_control import Drone

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    drone = Drone()
    drone.find(0)
    drone.takeoff(height=0.5)

    step = 1.0
    for dx, dy in [(step, 0), (0, step), (-step, 0), (0, -step)]:
        drone.move(dx, dy, 0)
        drone.wait()
        print(f"  Moved by ({dx:.1f}, {dy:.1f}), now at {drone.position}")

    drone.land()


if __name__ == "__main__":
    main()
