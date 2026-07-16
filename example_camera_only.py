#!/usr/bin/env python3
"""Example: display live camera feed from the drone."""

import logging
import time

import cv2

from drone_control.camera import WifiCamera

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    def on_frame(frame) -> None:
        cv2.imshow("drone", frame)
        cv2.waitKey(1)

    cam = WifiCamera(ip="192.168.4.1", port=5000)
    cam.start(on_frame)
    cam.wait_until_ready()

    try:
        while cam.is_running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
