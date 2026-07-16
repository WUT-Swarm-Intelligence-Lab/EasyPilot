#!/usr/bin/env python3
"""Search for an ArUco marker by chasing candidates in a centre ROI.

  1. Is there something small in the ROI?
     - Yes → fly toward it to investigate
     - No  → rotate
  2. After approaching, is it a confirmed ArUco?
     - Yes → land (victory!)
     - No  → rotate and keep looking
"""

import logging
import math
import random
import time
import threading

import cv2
import numpy as np

from drone_control import Drone

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

STEP_SIZE = 0.5
LARGE_AREA_RATIO = 0.15
ROI_SIZE = 100

X_MIN, X_MAX = -0.2, 1.0
Y_MIN, Y_MAX = -1.0, 1.0
YAW_STEP = math.pi / 4


def quad_area(pts: np.ndarray) -> float:
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(
        x[0] * y[1] - x[1] * y[0]
        + x[1] * y[2] - x[2] * y[1]
        + x[2] * y[3] - x[3] * y[2]
        + x[3] * y[0] - x[0] * y[3]
    )


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def in_roi(cx: float, cy: float, w: int, h: int) -> bool:
    half = ROI_SIZE / 2
    return (w / 2 - half) <= cx <= (w / 2 + half) and (h / 2 - half) <= cy <= (h / 2 + half)


def main() -> None:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    lock = threading.Lock()
    detection: dict = {"nx": 0.0, "ny": 0.0, "confirmed": False, "area_ratio": 0.0, "has": False}

    def on_frame(frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        gray = frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if len(frame.shape) == 2:
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            display = frame

        rx1 = int(w / 2 - ROI_SIZE / 2)
        ry1 = int(h / 2 - ROI_SIZE / 2)
        rx2 = int(w / 2 + ROI_SIZE / 2)
        ry2 = int(h / 2 + ROI_SIZE / 2)
        cv2.rectangle(display, (rx1, ry1), (rx2, ry2), (0, 0, 255), 1)

        cv2.aruco.drawDetectedMarkers(display, corners, ids)
        for quad in rejected:
            pts = quad.reshape(4, 2).astype(np.int32)
            cv2.polylines(display, [pts], True, (0, 255, 255), 1)
        cv2.imshow("drone", display)
        cv2.waitKey(1)

        with lock:
            if ids is not None and len(ids) > 0:
                for i in range(len(ids)):
                    pts = corners[i][0]
                    cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
                    if in_roi(cx, cy, w, h):
                        detection["nx"] = (cx - w / 2) / (w / 2)
                        detection["ny"] = (cy - h / 2) / (h / 2)
                        detection["confirmed"] = True
                        detection["area_ratio"] = quad_area(pts) / (w * h)
                        detection["has"] = True
                        return

            min_area = w * h * 0.02
            best_a = 0.0
            best_quad = None
            for quad in rejected:
                pts = quad.reshape(4, 2)
                cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
                if not in_roi(cx, cy, w, h):
                    continue
                a = quad_area(pts)
                if a >= min_area and a > best_a:
                    best_a = a
                    best_quad = pts

            if best_quad is not None:
                cx, cy = float(best_quad[:, 0].mean()), float(best_quad[:, 1].mean())
                detection["nx"] = (cx - w / 2) / (w / 2)
                detection["ny"] = (cy - h / 2) / (h / 2)
                detection["confirmed"] = False
                detection["area_ratio"] = best_a / (w * h)
                detection["has"] = True
            else:
                detection["has"] = False

    def read() -> dict | None:
        with lock:
            return dict(detection) if detection["has"] else None

    drone = Drone(camera_ip="192.168.4.1")
    drone.find(0)
    drone.camera_feed(on_frame)
    drone.camera_wait_until_ready()

    drone.takeoff(height=0.3)
    current_yaw = 0.0

    while True:
        time.sleep(2.0)
        det = read()

        if det is None:
            logging.info("ROI empty — rotating.")
            current_yaw = (current_yaw - YAW_STEP) % (2 * math.pi)
            drone.goto([drone.position.x, drone.position.y, drone.position.z], yaw=current_yaw)
            drone.wait()
            continue

        if det["confirmed"] and det["area_ratio"] > LARGE_AREA_RATIO:
            logging.info("ArUco marker confirmed! Landing.")
            drone.land()
            break

        if det["area_ratio"] > LARGE_AREA_RATIO:
            logging.info("Big non-marker (%.0f%%) — ignoring.", det["area_ratio"] * 100)
            current_yaw = (current_yaw - YAW_STEP) % (2 * math.pi)
            drone.goto([drone.position.x, drone.position.y, drone.position.z], yaw=current_yaw)
            drone.wait()
            continue

        logging.info("Candidate (%.0f%%) — flying toward it.", det["area_ratio"] * 100)
        new_x = clamp(drone.position.x + det["nx"] * STEP_SIZE, X_MIN, X_MAX)
        new_y = clamp(drone.position.y + det["ny"] * STEP_SIZE, Y_MIN, Y_MAX)
        drone.goto([new_x, new_y, drone.position.z])
        drone.wait()

        if det["confirmed"]:
            logging.info("ArUco marker confirmed! Landing.")
            drone.land()
            break

        logging.info("Not a marker — rotating.")

    drone.camera_stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
