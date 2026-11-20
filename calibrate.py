"""See what the camera sees, and check that it can find the table.

    python calibrate.py --camera 0     automatic: finds the six square markers on the work mat
    python calibrate.py --manual       by hand: click the four crosses A, B, C, D
    python calibrate.py --sim          try it on the simulated camera, no hardware needed

run.py finds the markers by itself every time it starts, so you only need this to
aim the camera (all six markers and the bin should be in view) or when the markers
cannot be read. ENTER saves calibration.json, which run.py falls back on.
R starts the clicks over, Q quits without saving.
"""
import argparse

import cv2
import numpy as np

from groundstation import config as c
from groundstation.simulator import FirmwareSim, SimWorld
from groundstation.vision import Calibration, find_markers
from run import CALIBRATION_FILE, SimCamera, Webcam

WINDOW = "Mars arm camera check"
GREEN, RED, WHITE = (0, 220, 0), (60, 60, 255), (255, 255, 255)


def _label(image, text, row=0, colour=WHITE):
    """One line of text on a dark strip along the bottom of the picture."""
    top = image.shape[0] - 22 * (3 - row) - 4
    image[top:top + 22] = image[top:top + 22] // 4
    cv2.putText(image, text, (8, top + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)


def draw_layout(image, calibration):
    """Draw where the station believes things are, so a bad calibration is obvious before it is used."""
    cv2.polylines(image, [calibration.pick_area_pixels()], True, GREEN, 2, cv2.LINE_AA)
    ring = [calibration.pixel(c.BIN_XY[0] + c.BIN_RADIUS * np.cos(a), c.BIN_XY[1] + c.BIN_RADIUS * np.sin(a))
            for a in np.linspace(0, 2 * np.pi, 40)]
    cv2.polylines(image, [np.int32(ring)], True, (40, 140, 255), 2, cv2.LINE_AA)
    cv2.drawMarker(image, tuple(map(int, calibration.pixel(0, 0))), GREEN, cv2.MARKER_TILTED_CROSS, 16, 2)


def annotate(frames):
    """(picture, calibration or None) for the automatic mode, from the latest frames."""
    image = frames[-1].copy()
    found = find_markers(image)
    for marker_id, corners in found.items():
        cv2.polylines(image, [np.int32(corners)], True, GREEN, 2, cv2.LINE_AA)
        cv2.putText(image, str(marker_id), tuple(map(int, corners[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)
    calibration = Calibration.from_frames(frames)
    total = len(c.CAL_MARKERS_MM)
    if calibration is None:
        _label(image, f"{len(found)} of {total} markers found, need 3. Check light, focus, and that the mat is in view.", 2, RED)
        return image, None
    draw_layout(image, calibration)
    good = calibration.rms_mm <= c.DRIFT_TOLERANCE_MM
    _label(image, f"{len(found)} of {total} markers. Fit error {calibration.rms_mm:.2f} mm "
                  f"({'good' if good else 'poor: is the mat flat and printed at 100%?'})", 0, GREEN if good else RED)
    if calibration.camera_mm:
        x, y, z = calibration.camera_mm
        _label(image, f"Camera is about {z:.0f} mm up, over x {x:.0f} y {y:.0f} mm", 1)
    _label(image, "Green box = pick area, orange ring = bin, X = arm base. ENTER saves.", 2)
    return image, calibration


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--manual", action="store_true", help="click the four crosses instead of using the markers")
    parser.add_argument("--sim", action="store_true", help="use the simulated camera")
    args = parser.parse_args()

    camera = SimCamera(SimWorld(FirmwareSim())) if args.sim else Webcam(args.camera)
    clicks, recent, calibration = [], [], None
    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, lambda event, x, y, *_: event == cv2.EVENT_LBUTTONDOWN
                         and args.manual and len(clicks) < 4 and clicks.append((x, y)))
    while True:
        frame = camera.read()
        if frame is None:
            continue
        if args.manual:
            image, calibration = frame.copy(), None
            for label, (u, v) in zip(c.CAL_LABELS, clicks):
                cv2.drawMarker(image, (u, v), GREEN, cv2.MARKER_CROSS, 18, 2)
                cv2.putText(image, label, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
            if len(clicks) < 4:
                x, y = c.CAL_POINTS_MM[len(clicks)]
                _label(image, f"Click cross {c.CAL_LABELS[len(clicks)]}  (x={x:.0f}, y={y:.0f} mm)", 2)
            else:
                calibration = Calibration(clicks, size=(frame.shape[1], frame.shape[0]))
                draw_layout(image, calibration)
                _label(image, "Green box should sit on the pick area. ENTER saves, R redo", 2)
        else:
            recent = (recent + [frame])[-10:]
            image, calibration = annotate(recent)
        cv2.imshow(WINDOW, image)
        key = cv2.waitKey(30 if args.sim else 1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            clicks.clear()
        if key in (13, 10) and calibration is not None:
            calibration.save(CALIBRATION_FILE)
            print("Saved", CALIBRATION_FILE)
            break


if __name__ == "__main__":
    main()
