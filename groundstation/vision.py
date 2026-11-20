"""Camera side: pixel <-> table calibration and finding the orange sample."""
import json
import math
from dataclasses import dataclass

import cv2
import numpy as np

from . import config as c


class Calibration:
    """Maps camera pixels to table millimetres.

    Built from known points on the table: the mat's ArUco markers (`from_frame`)
    or four crosses clicked by hand. It also works out roughly where the camera
    is, which is needed for things that stand up off the table: seen from the
    side, the top of the sample appears further from the camera than its base.
    """

    def __init__(self, pixels, table=c.CAL_POINTS_MM, size=(640, 480)):
        self.pixels = [tuple(map(float, p)) for p in pixels]
        self.table_points = [tuple(map(float, p)) for p in table]
        self.size = tuple(size)
        px, mm = np.float64(self.pixels), np.float64(self.table_points)
        self._to_mm, _ = cv2.findHomography(px, mm, 0)
        self._to_px = np.linalg.inv(self._to_mm)
        fitted = cv2.perspectiveTransform(px[None], self._to_mm)[0]
        self.rms_mm = float(np.sqrt(np.mean(np.sum((fitted - mm) ** 2, axis=1))))   # how well the points agree
        self.camera_mm = self._locate_camera(px, mm)

    def _locate_camera(self, px, mm):
        """(x, y, height) of the camera over the table in mm, or None if it cannot be worked out.

        The lens's field of view is assumed (config.CAMERA_HFOV_DEG), so the height
        is only as good as that guess. It is used for a correction of a millimetre
        or two, so being 15% out costs a fraction of a millimetre.
        """
        w, h = self.size
        f = w / (2 * math.tan(math.radians(c.CAMERA_HFOV_DEG) / 2))
        intrinsics = np.float64([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
        try:
            ok, rotation, shift = cv2.solvePnP(np.column_stack([mm, np.zeros(len(mm))]), px, intrinsics, None)
        except cv2.error:
            return None
        if not ok:
            return None
        x, y, z = (-cv2.Rodrigues(rotation)[0].T @ shift).ravel()
        return (float(x), float(y), float(abs(z))) if abs(z) > 50 else None

    @staticmethod
    def _apply(matrix, a, b):
        out = cv2.perspectiveTransform(np.float64([[[a, b]]]), matrix)[0, 0]
        return float(out[0]), float(out[1])

    def table(self, u, v, height=0.0, camera=None):
        """Pixel -> table position in mm of something `height` mm above the table.
        `camera` overrides where the camera is taken to be, as (x, y, height)."""
        x, y = self._apply(self._to_mm, u, v)
        camera = camera or self.camera_mm
        if height and camera:
            cx, cy, ch = camera
            x, y = cx + (x - cx) * (ch - height) / ch, cy + (y - cy) * (ch - height) / ch
        return x, y

    def pixel(self, x, y, height=0.0, camera=None):
        """Table position in mm (of something `height` mm above the table) -> pixel."""
        camera = camera or self.camera_mm
        if height and camera:
            cx, cy, ch = camera
            x, y = cx + (x - cx) * ch / (ch - height), cy + (y - cy) * ch / (ch - height)
        return self._apply(self._to_px, x, y)

    def pick_area_pixels(self, margin=0.0):
        (x0, x1), (y0, y1) = c.PICK_X, c.PICK_Y
        corners = [(x0 - margin, y0 - margin), (x0 - margin, y1 + margin),
                   (x1 + margin, y1 + margin), (x1 + margin, y0 - margin)]
        return np.int32([self.pixel(x, y) for x, y in corners])

    def pick_mask(self, shape, margin=10.0):
        """Mask that keeps only the pick area, so the bin, arm base and desk are never searched."""
        mask = np.zeros(shape[:2], np.uint8)
        cv2.fillPoly(mask, [self.pick_area_pixels(margin)], 255)
        return mask

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"pixels": self.pixels, "table_mm": self.table_points, "size": self.size,
                       "rms_mm": round(self.rms_mm, 3)}, f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        return cls(data["pixels"], data["table_mm"], data.get("size", (640, 480)))

    # --- calibrating from the printed ArUco markers ---

    @classmethod
    def from_frame(cls, frame):
        """Calibrate from the mat's ArUco markers in this frame, or None if fewer than three are visible."""
        return cls.from_frames([frame])

    @classmethod
    def from_frames(cls, frames):
        """Calibrate from several frames of a still camera: each marker corner is the median of where it was
        seen, which takes out sensor noise. None if fewer than three markers were seen in at least half the frames."""
        seen = {}
        for frame in frames:
            for marker_id, corners in find_markers(frame).items():
                seen.setdefault(marker_id, []).append(corners)
        pixels, table = [], []
        for marker_id, sightings in seen.items():
            if 2 * len(sightings) < len(frames):
                continue
            pixels += _marker_points(np.median(sightings, axis=0))
            table += _marker_table_points(marker_id)
        if len(pixels) < 3 * 5:
            return None
        return cls(pixels, table, (frames[0].shape[1], frames[0].shape[0]))

    def check(self, frame):
        """(markers seen, median error in mm) of this calibration against the markers in `frame`.

        A camera that has been knocked, or a mat that has slid, shows up here
        long before it causes a missed pick. The error is None if fewer than three markers are visible.
        """
        found = find_markers(frame)
        if len(found) < 3:
            return len(found), None
        errors = []
        for marker_id, corners in found.items():
            u, v = _marker_points(corners)[4]
            errors.append(math.dist(self.table(u, v), c.CAL_MARKERS_MM[marker_id]))
        return len(found), float(np.median(errors))


# --- ArUco markers ---

_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_PARAMETERS = cv2.aruco.DetectorParameters()
_PARAMETERS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
_DETECTOR = cv2.aruco.ArucoDetector(_DICTIONARY, _PARAMETERS)


def marker_image(marker_id, side_px):
    """The black-and-white picture of one marker (no quiet zone: print it on the white mat)."""
    return cv2.aruco.generateImageMarker(_DICTIONARY, marker_id, side_px)


def _detect(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    return {} if ids is None else {int(i): quad.reshape(4, 2) for i, quad in zip(ids.ravel(), corners)}


def find_markers(frame):
    """{marker id: 4x2 corner pixels} for every mat marker in the frame."""
    return {i: quad for i, quad in _detect(frame).items() if 0 <= i < len(c.CAL_MARKERS_MM)}


def find_arm_tag(frame):
    """Pixel position of the centre of the tag on the arm, or None if it is not in view."""
    quad = _detect(frame).get(c.ARM_TAG_ID)
    return None if quad is None else _marker_points(quad)[4]


def _marker_points(corners):
    """The four corners plus the centre (where the diagonals cross), in pixels."""
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = corners
    d = (x1 - x3) * (y2 - y4) - (y1 - y3) * (x2 - x4)
    t = ((x1 - x2) * (y2 - y4) - (y1 - y2) * (x2 - x4)) / d if d else 0.5
    return [*map(tuple, corners), (x1 + t * (x3 - x1), y1 + t * (y3 - y1))]


def _marker_table_points(marker_id):
    """Where the same five points are on the table. Printed upright: marker top = table +x, marker left = table +y."""
    cx, cy = c.CAL_MARKERS_MM[marker_id]
    h = c.MARKER_SIZE_MM / 2
    return [(cx + h, cy + h), (cx + h, cy - h), (cx - h, cy - h), (cx - h, cy + h), (cx, cy)]


@dataclass
class Detection:
    u: float            # centroid, pixels
    v: float
    box: tuple          # x, y, w, h in pixels
    area: float
    diameter_mm: float = None   # equivalent diameter on the table, when a calibration was given


def mm2_per_px2(calibration, u, v):
    """Table area (mm^2) covered by one pixel at (u, v)."""
    x0, y0 = calibration.table(u, v)
    x1, y1 = calibration.table(u + 1, v)
    x2, y2 = calibration.table(u, v + 1)
    return abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))


def white_balance(frame, mask=None):
    """Make the white mat neutral grey, so a warm bulb or daylight does not shift the orange.

    Only the colour balance is corrected: the brightest of the three channels keeps its
    value, the others are raised to match. Brightness is left alone on purpose. Boosting
    a shadowed mat up to full white would push the lit part of the sample into clipping,
    which turns orange yellow. The mat is found as the unsaturated pixels inside `mask`;
    the sample and the arm are saturated, so they do not pull the estimate.
    """
    pixels = frame[mask > 0] if mask is not None else frame.reshape(-1, 3)
    saturation = cv2.cvtColor(pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)[:, 0, 1]
    paper = pixels[saturation < 120]
    if len(paper) < 200:
        return frame
    level = np.maximum(np.median(paper, axis=0), 1.0)
    gain = np.clip(level.max() / level, 1.0, 3.0)
    return np.clip(frame * gain.astype(np.float32), 0, 255).astype(np.uint8)


def find_rock(frame, mask=None, calibration=None):
    """The orange sample in the frame (inside `mask` if given), or None.

    With a calibration, blobs are also judged by their real size and shape on the
    table, so an orange sticker, a sliver clipped by the edge of the pick area or
    a speck is not mistaken for the sample.
    """
    ox = oy = 0
    if mask is not None:                       # only look at the part of the image the mask covers
        ox, oy, w, h = cv2.boundingRect(mask)
        frame, mask = frame[oy:oy + h, ox:ox + w], mask[oy:oy + h, ox:ox + w]
    hsv = cv2.cvtColor(white_balance(frame, mask), cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, np.array(c.HSV_LO, np.uint8), np.array(c.HSV_HI, np.uint8))
    if mask is not None:
        orange = cv2.bitwise_and(orange, mask)
    orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # drop speckle
    contours, _ = cv2.findContours(orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_score = None, None
    for blob in contours:
        # The sample is convex, so it is measured by its convex hull. A shiny highlight that
        # reads as white bites a hole or a notch out of the orange; the hull fills it back in.
        hull = cv2.convexHull(blob)
        area = cv2.contourArea(hull)
        perimeter = cv2.arcLength(hull, True)
        if area < c.MIN_AREA_PX or cv2.contourArea(blob) < c.MIN_SOLIDITY * area:
            continue                           # too small, or too hollow to be one solid object
        m = cv2.moments(hull)
        u, v = m["m10"] / m["m00"] + ox, m["m01"] / m["m00"] + oy
        diameter = None
        if calibration is not None:
            diameter = 2 * np.sqrt(area * mm2_per_px2(calibration, u, v) / np.pi)
            if not c.ROCK_DIAMETER_MM[0] <= diameter <= c.ROCK_DIAMETER_MM[1]:
                continue
            if 4 * np.pi * area / perimeter ** 2 < c.MIN_CIRCULARITY:
                continue
            score = abs(diameter - c.ROCK_NOMINAL_MM)
        else:
            score = -area                      # no size information: the biggest blob wins
        if best_score is None or score < best_score:
            x, y, w, h = cv2.boundingRect(blob)
            best, best_score = Detection(u, v, (x + ox, y + oy, w, h), area, diameter), score
    return best


def in_pick_area(x, y):
    return c.PICK_X[0] <= x <= c.PICK_X[1] and c.PICK_Y[0] <= y <= c.PICK_Y[1]


def rock_position(calibration, detection):
    """Table position (mm) of a detected sample, corrected for it standing up off the table."""
    return calibration.table(detection.u, detection.v, c.ROCK_CENTRE_HEIGHT_MM)
