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
from drone_control.nerves import Nerves

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

STEP_SIZE = 0.1
LARGE_AREA_RATIO = 0.15
ROI_SIZE = 100

X_MIN, X_MAX = -0.2, 1.0
Y_MIN, Y_MAX = -1.0, 1.0
YAW_STEP = math.pi / 4

CAMERA_FOV_X = math.radians(62)
CAMERA_FOV_Y = math.radians(48)


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


def pixel_to_movement(
    nx: float,
    ny: float,
    step: float = STEP_SIZE,
    fov_x: float = CAMERA_FOV_X,
    fov_y: float = CAMERA_FOV_Y,
) -> tuple[float, float, float]:
    """Convert normalized camera coordinates to drone world-frame movement.

    Camera frame: x=right, y=down, z=forward
    Drone world frame: x=forward, y=left, z=up

    Args:
        nx: Normalized x in [-1, +1] (right positive)
        ny: Normalized y in [-1, +1] (down positive)
        step: Distance to move in meters
        fov_x: Horizontal field of view in radians
        fov_y: Vertical field of view in radians

    Returns:
        (dx, dy, dz) movement vector in world frame
    """
    azimuth = math.atan2(nx, 1.0 / math.tan(fov_x / 2))
    elevation = math.atan2(ny, 1.0 / math.tan(fov_y / 2))

    cos_e = math.cos(elevation)

    dx = step * cos_e * math.cos(azimuth)
    dy = -step * cos_e * math.sin(azimuth)
    dz = -step * math.sin(elevation)

    return dx, dy, dz

class Eye:
    
    def __init__(self) -> None:
        pass

class ArucoEye(Nerves):

    def __init__(self) -> None:
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.params = cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        self.detection: dict = {
            "nx": 0.0, 
            "ny": 0.0, 
            "confirmed": False, 
            "area_ratio": 0.0, 
            "has": False
        }
        super().__init__()

    def in_roi(self, x: float, y: float, w: int, h: int) -> bool:
        half = ROI_SIZE / 2
        if (self.rx1) <= x <= (self.rx2) and (self.ry1) <= y <= (self.ry2):
            print(f"Checking ROI: cx={x}, cy={y}, w={w}, h={h}, half={half}")
            print(f"ROI bounds: x_min={self.rx1}, x_max={self.rx2}, y_min={self.ry1}, y_max={self.ry2}")
            print(f"Satisfying conditions: {self.rx1 <= x <= self.rx2} and {self.ry1 <= y <= self.ry2}")
        return (self.rx1) <= x <= (self.rx2) and (self.ry1) <= y <= (self.ry2)


    def analyze(self, frame: np.ndarray, corners: np.ndarray, ids : list) -> dict:
        h, w = frame.shape[:2]
        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                pts = corners[i][0]
                cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
                if self.in_roi(cx, cy, w, h):
                    self.set_detection((cx - w / 2) / (w / 2), (cy - h / 2) / (h / 2), True, quad_area(pts) / (w * h), True)
        return self.detection


    def check_region_of_interest(self, frame: np.ndarray, rejected: list, display = None) -> list:
        for quad in rejected:
            pts = quad.reshape(4, 2).astype(np.int32)
            if display is not None:
                cv2.polylines(display, [pts], True, (0, 255, 255), 1)
        
            if self.in_roi(pts[0,0], pts[0,1], abs(pts[0,0] - pts[1,0]), abs(pts[0,1] - pts[1,1])):
                cv2.circle(display, (int(pts[:, 0].mean()), int(pts[:, 1].mean())), 5, (0, 255, 0), -1)
                return [[pts[:, 0].mean(), pts[:, 1].mean()]]
        return []


    def set_detection(self, nx: float, ny: float, confirmed: bool, area_ratio: float, has: bool) -> None:
        self.detection["nx"] = nx
        self.detection["ny"] = ny
        self.detection["confirmed"] = confirmed
        self.detection["area_ratio"] = area_ratio
        self.detection["has"] = has

    def see(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        gray = frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if len(frame.shape) == 2:
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            display = frame

        self.rx1 = int(w / 2 - ROI_SIZE / 2)
        self.ry1 = int(h / 2 - ROI_SIZE / 2)
        self.rx2 = int(w / 2 + ROI_SIZE / 2)
        self.ry2 = int(h / 2 + ROI_SIZE / 2)
        cv2.rectangle(display, (self.rx1, self.ry1), (self.rx2, self.ry2), (0, 0, 255), 1)

        if ids:
            print(f"Detected IDs: {ids.flatten()}")
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            self.spike(msg = {"ooi" : None, "frame" : frame, "msg" : "DONE"})
        else:
            ooi = self.check_region_of_interest(frame, rejected, display)
            if ooi:
                cx, cy = ooi[0]
                nx = (cx - w / 2) / (w / 2)
                ny = (cy - h / 2) / (h / 2)
                dx, dy, dz = pixel_to_movement(nx, ny)
                print(f"Object of interest detected at: {ooi}, movement: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
                self.spike(msg = {"ooi" : ooi, "frame" : frame, "msg" : "SEARCHING", "movement" : (dx, dy, dz)})

        # print(f"here: corners : {rejected} ids : {ids}")
        # detection = self.analyze(frame, rejected, len(rejected))
        # # now make dot
        cv2.imshow("drone", display)
        cv2.waitKey(1)
