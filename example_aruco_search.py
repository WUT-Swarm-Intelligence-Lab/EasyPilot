#!/usr/bin/env python3
"""Example: detect ArUco markers from the drone camera feed."""

import logging
import time

import cv2

from drone_control import Drone
from drone_control.eye import ArucoEye
from drone_control.brain import SearchArucoBrain
from drone_control.camera import WifiCamera

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

moving = False

def on_search(msg, id):
    global moving
    if moving:
        return
    movement = msg.get("movement")
    if movement:
        dx, dy, dz = movement
        print(f"Moving toward object: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
        moving = True
        drone.move(dx, dy, dz)
        moving = False

def main() -> None:
    global drone
    drone = Drone(camera_ip="192.168.4.1")
    drone.find(0)

    brain = SearchArucoBrain()
    eye = ArucoEye()
    brain.connect(eye)
    brain.add_spike_event("SEARCHING", on_search)
    brain.add_spike_event("DONE", lambda msg, id: [print(f"ArUco detected! {msg}"), drone.land()])

    cam = WifiCamera(ip="192.168.4.1", port=5000)
    cam.start(eye.see)
    cam.wait_until_ready()

    drone.wait_for_flight_ready()
    drone.takeoff()


    try:
        while cam.is_running:
            brain.listen()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
