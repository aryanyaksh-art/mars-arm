"""Draws the ground-station console.

The look is a painted-steel flight console, not a web dashboard: a pale
grey-green panel with engraved DIN legends, dark readout windows, and a row of
rectangular status lamps like a caution-and-warning panel. Colour is only ever
used to say something: green is good, amber is "in progress / pay attention",
red is a fault. Along the bottom, the light-time strip shows commands and video
actually travelling between the ground and the arm when the Mars delay is on.

Layout (1280 x 720):

    +--------------------------------------------------------------+
    | MARS ARM  sample collection ground station        T+ 00:01:23|
    +---------------------------------------+----------------------+
    |                                       | [LINK][VIDEO][CALIB] |
    |            camera, 800 x 600          | [AUTO][LOCK ][DELAY] |
    |                                       | pick sequence        |
    |                                       | joints               |
    |                                       | target, counters     |
    +---------------------------------------+ log                  |
    | GROUND  ---- light-time strip ----  ARM                      |
    +---------------------------------------+----------------------+
"""
import functools
import os

import cv2
import numpy as np

from . import config as c

WIDTH, HEIGHT = 1280, 720
HEADER_H = 40
VIEW_W, VIEW_H = 800, 600
STRIP_Y = HEADER_H + VIEW_H            # the light-time strip starts here
PANEL_X = VIEW_W
PAD = 20

# Colours are (blue, green, red).
CONSOLE = (195, 204, 200)       # pale grey-green painted steel
SHADE = (176, 186, 181)         # the same paint in shadow: unlit lamps, scale beds
ENGRAVE = (31, 39, 34)          # legends
FAINT = (112, 122, 116)         # secondary legends
WINDOW = (26, 23, 20)           # readout glass
PHOSPHOR = (217, 230, 232)      # figures behind the glass
GO = (88, 158, 47)
CAUTION = (26, 162, 224)
FAULT = (43, 64, 208)
RUST = (42, 83, 181)            # Mars; used only for the sample bin and the arm end of the strip
WARNINGS = ("ERR", "MISS", "CAMERA", "ARM ", "NOT SENT", "# ABORT", "# link: port lost")   # log lines shown in red

SEQUENCE = (("APPROACH", "APPR"), ("DESCEND", "DESC"), ("GRIP", "GRIP"), ("LIFT", "LIFT"),
            ("TO_BIN", "BIN"), ("RELEASE", "REL"), ("HOME", "HOME"))


def _face(filename, builtin):
    path = os.path.join(os.environ.get("WINDIR", "C:/Windows"), "Fonts", filename)
    try:
        return cv2.FontFace(path) if os.path.exists(path) else cv2.FontFace(builtin)
    except cv2.error:
        return cv2.FontFace(builtin)


DIN = _face("bahnschrift.ttf", "sans")      # legends: the DIN 1451 lettering of instrument panels
MONO = _face("consola.ttf", "mono")         # figures: every digit the same width, so numbers do not jitter
_SCRATCH = np.zeros((2, 2, 3), np.uint8)


@functools.lru_cache(maxsize=4096)
def _width(text, mono, size, weight, track):
    end, _ = cv2.putText(_SCRATCH, text, (0, 0), (0, 0, 0), MONO if mono else DIN, size, weight)
    return end[0] + track * max(0, len(text) - 1)


def _text(img, text, x, y, size=13, color=ENGRAVE, mono=False, weight=400, track=0.0, align="left"):
    """Draw text with its baseline at y. `track` adds space between letters, as engraved legends have."""
    text = str(text)
    if align != "left":
        w = _width(text, mono, size, weight, track)
        x -= w if align == "right" else w / 2
    face = MONO if mono else DIN
    if not track:
        cv2.putText(img, text, (int(round(x)), int(y)), color, face, size, weight)
        return
    for ch in text:
        cv2.putText(img, ch, (int(round(x)), int(y)), color, face, size, weight)
        x += _width(ch, mono, size, weight, 0) + track


def _legend(img, text, x, y, color=FAINT, size=11, align="left"):
    _text(img, text, x, y, size, color, weight=600, track=1.2, align=align)


def _window(img, x, y, w, h):
    cv2.rectangle(img, (x, y), (x + w - 1, y + h - 1), WINDOW, -1)


def _lamp(img, x, y, w, h, text, lit=None):
    """A rectangular status lamp. `lit` is its colour when on, None when off."""
    if lit is None:
        cv2.rectangle(img, (x, y), (x + w - 1, y + h - 1), SHADE, -1)
        _text(img, text, x + w / 2, y + h / 2 + 5, 13, FAINT, weight=600, track=1.0, align="center")
    else:
        cv2.rectangle(img, (x, y), (x + w - 1, y + h - 1), lit, -1)
        ink = ENGRAVE if lit in (CAUTION, PHOSPHOR) else (255, 255, 255)
        _text(img, text, x + w / 2, y + h / 2 + 5, 13, ink, weight=700, track=1.0, align="center")


def _clock(seconds):
    seconds = max(0.0, seconds)
    return f"{int(seconds // 3600):02d}:{int(seconds // 60 % 60):02d}:{int(seconds % 60):02d}"


def _problem(station):
    """Why the station will not send a pick right now, or None."""
    if not station.camera_ok:
        return "no camera frames"
    if not station.cal_ok:
        return "camera has moved and the markers cannot be read"
    if not station.link_ok:
        return "arm is not answering"
    return None


# --- camera view ---

def _brackets(img, x, y, w, h, color, arm=14, thick=2):
    """Corner brackets around a box: the lock-on marker."""
    for cx, cy, dx, dy in ((x, y, 1, 1), (x + w, y, -1, 1), (x, y + h, 1, -1), (x + w, y + h, -1, -1)):
        cv2.line(img, (cx, cy), (cx + dx * arm, cy), color, thick, cv2.LINE_AA)
        cv2.line(img, (cx, cy), (cx, cy + dy * arm), color, thick, cv2.LINE_AA)


def _tag(img, text, x, y, fill, ink=(255, 255, 255)):
    """A small flat label on the video."""
    w = int(_width(text, False, 12, 700, 1.0)) + 14
    cv2.rectangle(img, (x, y), (x + w, y + 20), fill, -1)
    _text(img, text, x + 7, y + 15, 12, ink, weight=700, track=1.0)
    return w


def _camera_view(station):
    if station.view is None:
        view = np.full((VIEW_H, VIEW_W, 3), WINDOW, np.uint8)
        waiting = f"FIRST FRAME ARRIVES IN {max(0.0, station.delay - station.now):.1f} s" if station.mars else "WAITING FOR CAMERA"
        _legend(view, waiting, VIEW_W / 2, VIEW_H / 2, PHOSPHOR, 14, "center")
        return view
    scale = VIEW_W / station.view.shape[1]  # the camera is asked for 4:3, which fills the view exactly
    view = cv2.resize(station.view, (VIEW_W, VIEW_H), interpolation=cv2.INTER_LINEAR)
    cal = station.calibration
    px = lambda x, y: tuple(int(round(v * scale)) for v in cal.pixel(x, y))

    # Pick area, with a millimetre scale along two edges so positions can be read off the picture.
    (x0, x1), (y0, y1) = c.PICK_X, c.PICK_Y
    green = (60, 130, 30)
    cv2.polylines(view, [np.int32([px(x0, y0), px(x0, y1), px(x1, y1), px(x1, y0)])], True, green, 1, cv2.LINE_AA)
    for x in range(int(x0), int(x1) + 1, 10):         # x scale down the right-hand edge, where the mat is clear
        a, b = px(x, y0), px(x, y0 - 4)
        cv2.line(view, a, b, green, 1, cv2.LINE_AA)
        _text(view, x, b[0] + 4, b[1] + 4, 10, green, mono=True)
    for y in range(int(y0), int(y1), 20):             # y scale along the far edge
        a, b = px(x1, y), px(x1 + 4, y)
        cv2.line(view, a, b, green, 1, cv2.LINE_AA)
        _text(view, y, b[0], b[1] - 4, 10, green, mono=True, align="center")
    bx, by = px(*c.BIN_XY)
    radius = abs(px(c.BIN_XY[0], c.BIN_XY[1] + c.BIN_RADIUS)[0] - bx)
    cv2.circle(view, (bx, by), radius, RUST, 1, cv2.LINE_AA)
    _legend(view, "BIN", bx, by - radius - 6, RUST, 11, "center")

    det = station.detection
    if det:
        x, y, w, h = (int(v * scale) for v in det.box)
        pad = 8
        if station.target is None:
            color, label = FAULT, "OUT OF PICK AREA"
        elif station.locked:
            color, label = (60, 130, 30), "LOCK"
        else:
            color, label = (0, 120, 200), f"ACQUIRING {station.lock}/{c.LOCK_FRAMES}"
        _brackets(view, x - pad, y - pad, w + 2 * pad, h + 2 * pad, color)
        cv2.drawMarker(view, (int(det.u * scale), int(det.v * scale)), color, cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
        tag_w = _tag(view, label, x - pad, y + h + pad + 5, color)
        if station.target:
            _text(view, f"x {station.target[0]:.0f}  y {station.target[1]:.0f} mm", x - pad + tag_w + 8,
                  y + h + pad + 20, 13, color, mono=True, weight=700)

    if station.mars:
        _tag(view, f"DELAYED {station.delay:.1f} s", 10, 10, CAUTION, ENGRAVE)
    else:
        _tag(view, "LIVE", 10, 10, GO)
    problem = _problem(station)
    if problem:
        cv2.rectangle(view, (0, VIEW_H - 30), (VIEW_W, VIEW_H), FAULT, -1)
        _text(view, "PICKS STOPPED   " + problem, 12, VIEW_H - 10, 14, (255, 255, 255), weight=700, track=0.6)
    return view


# --- light-time strip ---

def _strip(img, station):
    """Commands going up and video coming down, placed by how far through the delay they are."""
    left, right, up, down = 118, VIEW_W - 100, STRIP_Y + 34, STRIP_Y + 58
    cv2.line(img, (0, STRIP_Y), (VIEW_W, STRIP_Y), ENGRAVE, 1)
    _text(img, "GROUND", 16, STRIP_Y + 44, 15, ENGRAVE, weight=700, track=1.5)
    _legend(img, "THIS LAPTOP", 16, STRIP_Y + 60)
    _text(img, "ARM", VIEW_W - 16, STRIP_Y + 44, 15, RUST, weight=700, track=1.5, align="right")
    _legend(img, "ARDUINO", VIEW_W - 16, STRIP_Y + 60, align="right")
    for y in (up, down):
        cv2.line(img, (left, y), (right, y), FAINT, 1)
    cv2.fillPoly(img, [np.int32([(right, up), (right - 7, up - 4), (right - 7, up + 4)])], FAINT)
    cv2.fillPoly(img, [np.int32([(left, down), (left + 7, down - 4), (left + 7, down + 4)])], FAINT)
    if not station.mars:
        _legend(img, "DIRECT LINK   NO DELAY   PRESS M FOR MARS LIGHT TIME", (left + right) / 2, STRIP_Y + 17, align="center")
        return
    _legend(img, f"ONE-WAY LIGHT TIME {station.delay:.1f} s", (left + right) / 2, STRIP_Y + 17, ENGRAVE, align="center")
    span = right - left
    frames, lines = station.downlink()
    for i, sent in enumerate(frames):                   # video frames on their way down
        if i % 2 == 0:
            x = int(right - span * min(1.0, (station.now - sent) / station.delay))
            cv2.line(img, (x, down - 3), (x, down + 3), ENGRAVE, 1)
    for sent in lines:                                  # telemetry and replies
        x = int(right - span * min(1.0, (station.now - sent) / station.delay))
        cv2.rectangle(img, (x - 2, down - 6), (x + 2, down + 6), CAUTION, -1)
    for arrives, text in station.uplink:                # commands on their way up
        x = int(left + span * (1 - min(1.0, max(0.0, (arrives - station.now) / station.delay))))
        w = int(_width(text, True, 12, 700, 0)) + 10
        cv2.rectangle(img, (x - w, up - 9), (x, up + 9), ENGRAVE, -1)
        _text(img, text, x - w + 5, up + 4, 12, PHOSPHOR, mono=True, weight=700)


# --- right-hand panel ---

@functools.lru_cache(maxsize=1)
def _blank():
    """Everything that never changes, drawn once."""
    img = np.full((HEIGHT, WIDTH, 3), CONSOLE, np.uint8)
    _text(img, "MARS ARM", 16, 27, 20, ENGRAVE, weight=700, track=3.0)
    _legend(img, "SAMPLE COLLECTION GROUND STATION", 176, 26)
    _legend(img, "MISSION TIME", WIDTH - 150, 26, align="right")
    _window(img, WIDTH - 140, 8, 124, 26)
    cv2.line(img, (0, HEADER_H - 1), (WIDTH, HEADER_H - 1), ENGRAVE, 1)
    cv2.line(img, (PANEL_X, HEADER_H), (PANEL_X, HEIGHT), ENGRAVE, 1)

    x = PANEL_X + PAD
    _legend(img, "PICK SEQUENCE", x, 158)
    _legend(img, "JOINTS", x, 222)
    for i, name in enumerate(("BASE", "SHOULDER", "ELBOW", "GRIPPER")):
        y = 246 + i * 30
        _text(img, name, x, y + 4, 12, ENGRAVE, weight=600, track=0.8)
        bed = (x + 92, x + 92 + 270)
        cv2.rectangle(img, (bed[0], y - 4), (bed[1], y + 4), SHADE, -1)
        for degrees in range(0, 181, 30):
            tx = bed[0] + int(270 * degrees / 180)
            cv2.line(img, (tx, y - (7 if degrees % 90 == 0 else 4)), (tx, y + 4), ENGRAVE, 1)
        _window(img, x + 376, y - 11, 64, 22)
    _legend(img, "0", x + 92, 356, size=10, align="center")
    _legend(img, "90", x + 92 + 135, 356, size=10, align="center")
    _legend(img, "180 DEG", x + 92 + 270, 356, size=10, align="center")

    _legend(img, "TARGET", x, 386)
    _window(img, x, 394, 250, 40)
    _legend(img, "COLLECTED", x + 268, 386)
    _window(img, x + 268, 394, 78, 40)
    _legend(img, "MISSED", x + 362, 386)
    _window(img, x + 362, 394, 78, 40)

    _legend(img, "LOG", x, 462)
    _window(img, x, 470, 440, 202)
    _legend(img, "SPACE PICK  A AUTO  M DELAY  X ABORT  H HOME  T TRIM ARM  Q QUIT", x, 692, size=10)
    return img


def _panel(img, station):
    x = PANEL_X + PAD
    w, h, gap = 142, 34, 7
    lamps = (
        ("LINK", GO) if station.link_ok else ("NO LINK", FAULT),
        ("VIDEO", GO) if station.camera_ok else ("NO VIDEO", FAULT),
        ("CALIB", GO) if station.cal_ok else ("CAL LOST", FAULT),
        ("AUTO", PHOSPHOR if station.auto else None),
        ("LOCK", GO if station.locked else CAUTION if station.lock else None),
        (f"DELAY {station.delay:.0f} s", CAUTION) if station.mars else ("DELAY", None),
    )
    for i, (text, lit) in enumerate(lamps):
        _lamp(img, x + (i % 3) * (w + gap), 56 + (i // 3) * (h + gap), w, h, text, lit)

    # Pick sequence: the steps the firmware goes through, in order.
    names = [name for name, _ in SEQUENCE]
    at = names.index(station.phase) if station.phase in names else -1
    seg = 440 // len(SEQUENCE)
    for i, (_, short) in enumerate(SEQUENCE):
        sx = x + i * seg
        if i == at:
            cv2.rectangle(img, (sx, 166), (sx + seg - 4, 190), CAUTION, -1)
            _text(img, short, sx + (seg - 4) / 2, 183, 12, ENGRAVE, weight=700, track=0.8, align="center")
        elif i < at:
            cv2.rectangle(img, (sx, 166), (sx + seg - 4, 190), ENGRAVE, -1)
            _text(img, short, sx + (seg - 4) / 2, 183, 12, CONSOLE, weight=600, track=0.8, align="center")
        else:
            cv2.rectangle(img, (sx, 166), (sx + seg - 4, 190), SHADE, -1)
            _text(img, short, sx + (seg - 4) / 2, 183, 12, FAINT, weight=600, track=0.8, align="center")
    if station.routine_status:
        _legend(img, station.routine_status, x + 440, 158, ENGRAVE, align="right")
    elif station.phase == "MOVE":
        _legend(img, "MANUAL MOVE", x + 440, 158, CAUTION, align="right")
    elif station.pending and at < 0:
        _legend(img, "COMMAND SENT", x + 440, 158, ENGRAVE, align="right")

    for i, angle in enumerate(station.joints):
        y = 246 + i * 30
        tx = x + 92 + int(270 * max(0.0, min(180.0, angle)) / 180)
        cv2.fillPoly(img, [np.int32([(tx, y + 3), (tx - 6, y - 9), (tx + 6, y - 9)])], ENGRAVE, cv2.LINE_AA)
        _text(img, f"{angle:5.1f}", x + 434, y + 5, 14, PHOSPHOR, mono=True, align="right")

    if station.target:
        _text(img, f"X{station.target[0]:4.0f}  Y{station.target[1]:4.0f}", x + 10, 422, 22,
              PHOSPHOR if station.locked else CAUTION, mono=True, weight=700)
        _text(img, "mm", x + 240, 422, 12, FAINT, mono=True, align="right")
    else:
        _text(img, "NO SAMPLE IN VIEW", x + 10, 420, 13, FAINT, weight=600, track=1.0)
    for i in range(c.LOCK_FRAMES):                       # lock builds up one tick per steady frame
        tick = x + i * 21
        cv2.rectangle(img, (tick, 438), (tick + 17, 441), GO if i < station.lock else SHADE, -1)
    _text(img, station.collected, x + 268 + 68, 424, 24, PHOSPHOR, mono=True, weight=700, align="right")
    _text(img, station.misses, x + 362 + 68, 424, 24, FAULT if station.misses else FAINT, mono=True, weight=700, align="right")

    status = "camera " + station.cal_status if station.camera_ok else "camera: no frames"
    _text(img, status[:44], x + 440, 462, 11, FAINT if station.camera_ok and station.cal_ok else FAULT,
          mono=True, align="right")
    fits = int((440 - 66 - 10) / _width("0", True, 12, 400, 0))     # characters that fit in the log window
    for i, (t, text) in enumerate(station.log):
        y = 492 + i * 20
        _text(img, f"{t:6.1f}", x + 10, y, 12, FAINT, mono=True)
        _text(img, text[:fits], x + 66, y, 12, (90, 120, 255) if text.startswith(WARNINGS) else PHOSPHOR, mono=True)
    if station.link.name.startswith("SIM"):
        _legend(img, "SIM  R ROCK  C CAMERA FAULT  K KNOCK  E BAD SERVOS", x, 708, size=10)
    else:
        _legend(img, station.link.name, x, 708, size=10)


def draw(station):
    img = _blank().copy()
    _text(img, "T+ " + _clock(station.now), WIDTH - 78, 26, 14, PHOSPHOR, mono=True, align="center")
    img[HEADER_H:STRIP_Y, :VIEW_W] = _camera_view(station)
    _strip(img, station)
    _panel(img, station)
    return img
