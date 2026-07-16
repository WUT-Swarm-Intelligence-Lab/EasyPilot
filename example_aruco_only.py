#!/usr/bin/env python3
"""Example: detect ArUco markers from the drone camera feed."""

import logging
import time

import cv2

from drone_control.camera import WifiCamera

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    ROI_SIZE = 100

    def on_frame(frame) -> None:
        h, w = frame.shape[:2]
        if len(frame.shape) == 2 or frame.shape[2] == 1:
            gray = frame
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        rx1 = int(w / 2 - ROI_SIZE / 2)
        ry1 = int(h / 2 - ROI_SIZE / 2)
        rx2 = int(w / 2 + ROI_SIZE / 2)
        ry2 = int(h / 2 + ROI_SIZE / 2)
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 0, 255), 1)

        corners, ids, _ = detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                pts = corners[i][0]
                cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
                logging.info("Marker %d at (%.0f, %.0f)", int(marker_id), cx, cy)

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
