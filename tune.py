"""Tune the colour the station treats as "the sample", live, on your own camera and lighting.

    python tune.py --camera 0      real webcam (the work mat must be in view)
    python tune.py --sim           try it on the simulated camera, no hardware needed

Left: the camera, with what the station would lock on to. Right: the pixels that
count as sample colour (white) inside the pick area.

Click the sample in the left picture and the sliders jump to a range around its
colour. Then nudge them until the sample is solid white and nothing else is.
S prints the two lines to paste into groundstation/config.py, Q quits.
"""
import argparse

import cv2
import numpy as np

from groundstation import config as c
from groundstation.simulator import FirmwareSim, SimWorld
from groundstation.vision import Calibration, find_rock, rock_position, white_balance
from run import CALIBRATION_FILE, SimCamera, Webcam, calibrate_from_markers

WINDOW = "Mars arm colour tuning"
SLIDERS = (("hue low", 0, 179), ("hue high", 3, 179), ("saturation low", 1, 255), ("brightness low", 2, 255))


def range_around(frame, mask, u, v):
    """A colour range around the pixel that was clicked: (hue low, hue high, saturation low, brightness low)."""
    hsv = cv2.cvtColor(white_balance(frame, mask), cv2.COLOR_BGR2HSV)
    patch = hsv[max(0, v - 4):v + 5, max(0, u - 4):u + 5].reshape(-1, 3)
    hue, saturation, brightness = np.median(patch, axis=0)
    return (int(max(0, hue - 10)), int(min(179, hue + 10)),
            int(max(40, saturation * 0.5)), int(max(30, brightness * 0.4)))


def view(frame, calibration):
    """The side-by-side picture, and the detection (or None), for the current settings in config."""
    mask = calibration.pick_mask(frame.shape)
    hsv = cv2.cvtColor(white_balance(frame, mask), cv2.COLOR_BGR2HSV)
    match = cv2.inRange(hsv, np.array(c.HSV_LO, np.uint8), np.array(c.HSV_HI, np.uint8)) & mask
    found = find_rock(frame, mask, calibration)
    left = frame.copy()
    cv2.polylines(left, [calibration.pick_area_pixels()], True, (0, 220, 0), 1, cv2.LINE_AA)
    status = "no sample found in the pick area"
    if found:
        x, y, w, h = found.box
        cv2.rectangle(left, (x - 4, y - 4), (x + w + 4, y + h + 4), (0, 220, 0), 2)
        tx, ty = rock_position(calibration, found)
        status = f"sample at x {tx:.0f} y {ty:.0f} mm, {found.diameter_mm:.0f} mm across"
    picture = np.hstack([left, cv2.cvtColor(match, cv2.COLOR_GRAY2BGR)])
    picture[-24:] = picture[-24:] // 4
    cv2.putText(picture, status + "    click the sample to start, S prints settings", (8, picture.shape[0] - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return picture, found


def settings_text():
    return f"HSV_LO = {tuple(c.HSV_LO)}\nHSV_HI = {tuple(c.HSV_HI)}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--sim", action="store_true", help="use the simulated camera")
    args = parser.parse_args()

    camera = SimCamera(SimWorld(FirmwareSim())) if args.sim else Webcam(args.camera)
    calibration = calibrate_from_markers(camera)
    if calibration is None:
        try:
            calibration = Calibration.load(CALIBRATION_FILE)
        except OSError:
            raise SystemExit("Could not see the mat's markers and there is no calibration.json. Run calibrate.py first.")

    cv2.namedWindow(WINDOW)
    start = (c.HSV_LO[0], c.HSV_HI[0], c.HSV_LO[1], c.HSV_LO[2])
    for (name, _, top), value in zip(SLIDERS, start):
        cv2.createTrackbar(name, WINDOW, value, top, lambda _: None)
    clicked = []
    cv2.setMouseCallback(WINDOW, lambda event, x, y, *_: event == cv2.EVENT_LBUTTONDOWN and clicked.append((x, y)))
    while True:
        frame = camera.read()
        if frame is None:
            continue
        if clicked:
            u, v = clicked.pop()
            if u < frame.shape[1]:          # a click in the camera half
                for (name, *_), value in zip(SLIDERS, range_around(frame, calibration.pick_mask(frame.shape), u, v)):
                    cv2.setTrackbarPos(name, WINDOW, value)
        low, high, saturation, brightness = (cv2.getTrackbarPos(name, WINDOW) for name, *_ in SLIDERS)
        c.HSV_LO, c.HSV_HI = (low, saturation, brightness), (max(low, high), 255, 255)
        cv2.imshow(WINDOW, view(frame, calibration)[0])
        key = cv2.waitKey(30 if args.sim else 1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            print("Paste these over the same two lines in groundstation/config.py:\n" + settings_text())
    print(settings_text())


if __name__ == "__main__":
    main()
