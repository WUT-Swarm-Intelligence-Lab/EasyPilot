#!/usr/bin/env python3
"""Example: detect ArUco markers from the drone camera feed."""

import logging
import time

import cv2

from drone_control.eye import ArucoEye
from drone_control.brain import SearchArucoBrain
from drone_control.camera import WifiCamera

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    brain = SearchArucoBrain()
    eye = ArucoEye()
    brain.connect(eye)
    brain.add_spike_event("SEARCHING", lambda msg, id: print(f"Searching for ArUco... {msg}"))
    brain.add_spike_event("DONE", lambda msg, id: print(f"ArUco detected! {msg}"))

    cam = WifiCamera(ip="192.168.4.1", port=5000)
    cam.start(eye.see)
    cam.wait_until_ready()

    try:
        while cam.is_running:
            brain.listen()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
