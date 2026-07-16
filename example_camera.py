#!/usr/bin/env python3
"""Example: fly through 4 waypoints with live camera feed."""

import logging
import time

from drone_control import Drone

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    import cv2

    def on_frame(frame) -> None:
        cv2.imshow("drone", frame)
        cv2.waitKey(1)

    drone = Drone(camera_ip="192.168.4.1")
    drone.find(0)
    drone.set_forward_fly(True)
    drone.camera_feed(on_frame)
    drone.camera_wait_until_ready()
    drone.takeoff(height=0.5)

    for wp in [
        [1.5, 0.0, 0.5],
        [1.5, 1.5, 0.5],
        [0.0, 1.5, 0.5],
        [0.0, 0.0, 0.5],
    ]:
        drone.goto(wp)
        while drone.is_moving():
            time.sleep(0.05)
        print(f"  Reached {wp}")

    drone.land()
    drone.camera_stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
