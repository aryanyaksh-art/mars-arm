"""Stand-ins for the hardware, so the ground station runs with no arm and no camera.

FirmwareSim speaks the same serial protocol as mars_arm.ino and moves its servos
at the same speed. SimWorld is the overhead webcam: it draws the table, the arm
and the rock, and lets the gripper actually carry the rock.
"""
import math
import re
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from . import config as c
from .kinematics import forearm_axes, forward, solve_ik, tag_position
from .vision import marker_image

LINE_MAX = 32               # the sketch's command buffer, including the terminator
STEP_MS = 12                # one interpolation step
STEP_DEG = 1.0
SEGMENT_MM = 10.0
SETTLE_MS = 200             # pause after the gripper moves
TELEMETRY_MS = 250
HEARTBEAT_MS = 1000
LOOP_US = 100               # time one idle pass of loop() is taken to cost (as in firmware_host/host.cpp)
BYTE_US = 87                # one serial byte at 115200 baud

_NUMBER = re.compile(r"\s*([+-]?(?:\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?|inf(?:inity)?|nan))", re.I)


def _print_float(value, digits):
    """What Arduino's Serial.print(value, digits) prints, worked in 32-bit floats as the board does.
    It rounds half away from zero."""
    f = np.float32
    value = f(value)
    sign, value = ("-", -value) if value < 0 else ("", value)
    rounding = f(0.5)
    for _ in range(digits):
        rounding = rounding / f(10)
    value = value + rounding
    whole = int(value)
    out = sign + str(whole)
    if digits:
        out += "."
        rest = value - f(whole)
        for _ in range(digits):
            rest = rest * f(10)
            out += str(int(rest))
            rest = rest - f(int(rest))
    return out


class FirmwareSim:
    """Python imitation of mars_arm.ino: same commands, same replies, same timing.

    Each method below is a translation of the function of the same name in the
    sketch. Where the sketch calls delay(), the method yields the time to wait,
    so the whole thing can be stepped in simulated time. tests/test_firmware.py
    checks it against the real sketch, line for line.
    """

    def __init__(self, eeprom=None):
        """`eeprom` is a dict standing for the board's EEPROM: pass the same one to a second
        FirmwareSim and it remembers the trim the first one stored."""
        self.cur = [*c.HOME, c.GRIP_OPEN]   # base, shoulder, elbow, gripper: the angles the sketch works in
        self.eeprom = {} if eeprom is None else eeprom
        self.trim_offset = list(self.eeprom.get("offset", (0.0, 0.0, 0.0)))    # degrees
        self.trim_gain = list(self.eeprom.get("gain", (0.0, 0.0, 0.0)))        # percent
        self.phase = "IDLE"
        self.delay_ms = 0
        self._now = 0                       # microseconds
        self._clock = 0.0                   # seconds of time granted so far
        self._wire = deque()                # (arrival time, line): bytes still on their way to the board
        self._last_arrival = 0
        self._queue = [None] * 4            # (due ms, text): commands waiting out their Mars delay
        self._out = []
        self._task = None                   # generator running the current command
        self._resume = 0                    # when the current command's delay() ends
        self._next_pass = 0                 # when loop() runs next while idle
        self._cmd = ""
        self._tip = None                    # last commanded jaw position, if known
        self._last_telemetry = 0
        self._emit("# Mars arm ready. Try: P 120 20   (or press the green button)")
        self._print_trim()
        self._telemetry(force=True)

    @property
    def servo(self):
        """The angles actually commanded to the four servos: `cur` with the trim applied (writeServo)."""
        out = [min(180.0, max(0.0, 90 + (a - 90 - o) / (1 + g / 100)))
               for a, o, g in zip(self.cur, self.trim_offset, self.trim_gain)]
        return out + [min(180.0, max(0.0, self.cur[3]))]

    # --- serial side ---

    @property
    def now_ms(self):
        return self._now / 1000

    def write(self, text):
        text = text.strip()
        if text:
            self._last_arrival = max(self._last_arrival, self._now) + BYTE_US * (len(text) + 1)
            self._wire.append((self._last_arrival, text))

    def read_lines(self):
        lines, self._out = self._out, []
        return lines

    def advance(self, seconds):
        """Run the firmware for this much time."""
        self._clock += seconds
        end = round(self._clock * 1e6)
        while True:
            if self._task is not None:
                if self._resume > end:
                    break
                self._now = self._resume
                self._run_task()
            else:
                at = self._next_idle_event()
                if at > end:
                    break
                self._now = at
                self._loop()
        self._now = end

    # --- the sketch, function by function ---

    def _millis(self):
        return self._now // 1000

    def _emit(self, line):
        self._out.append(line)

    def _telemetry(self, force=False):
        if not force and self._millis() - self._last_telemetry < TELEMETRY_MS:
            return
        self._last_telemetry = self._millis()
        self._emit("T," + ",".join(_print_float(a, 1) for a in self.cur) + "," + self.phase)

    def _print_trim(self):
        self._emit("# trim " + " ".join(_print_float(v, 2) for v in self.trim_offset)
                   + " gain " + " ".join(_print_float(v, 2) for v in self.trim_gain))

    def _enqueue(self, text):
        if text[0] == "M":                  # ground-station setting, applies immediately
            self.delay_ms = int(re.match(r"\s*\+?(\d*)", text[1:]).group(1) or 0)
            self._emit(f"OK comms delay ms={self.delay_ms}")
            return
        for i, slot in enumerate(self._queue):
            if slot is None:
                self._queue[i] = (self._millis() + self.delay_ms, text)
                if self.delay_ms:
                    self._emit(f"# uplink '{text}' arrives in ms={self.delay_ms}")
                return
        self._emit("ERR queue full")

    def _poll_serial(self):
        while self._wire and self._wire[0][0] <= self._now:
            text = self._wire.popleft()[1]
            if len(text) > LINE_MAX - 1:
                self._emit("ERR line too long")
            else:
                self._enqueue(text)

    def _next_arrived(self):
        best = None
        for i, slot in enumerate(self._queue):
            if slot and slot[0] <= self._millis() and (best is None or slot[0] < self._queue[best][0]):
                best = i
        return best

    def _abort_arrived(self):
        self._poll_serial()
        for i, slot in enumerate(self._queue):
            if slot and slot[1][0] == "X" and slot[0] <= self._millis():
                self._queue[i] = None
                return True
        return False

    def _move_to(self, target):
        start = list(self.cur)
        steps = max(1, int(max(abs(t - s) for t, s in zip(target, start)) / STEP_DEG))
        for step in range(1, steps + 1):
            self.cur = [s + (t - s) * step / steps for s, t in zip(start, target)]
            yield STEP_MS * 1000
            self._telemetry()
            if self._abort_arrived():
                self._emit("# ABORT")
                self._tip = None
                return False
        return True

    def _line_to(self, x, y, z):
        start, segments = (x, y, z), 1
        if self._tip:
            start = self._tip
            segments = max(1, math.ceil(math.dist(start, (x, y, z)) / SEGMENT_MM))
        for s in range(1, segments + 1):
            f = s / segments
            angles = solve_ik(*(a + (b - a) * f for a, b in zip(start, (x, y, z))))
            if angles is None:
                self._emit("ERR unreachable")
                return False
            if not (yield from self._move_to([*angles, self.cur[3]])):
                return False
        self._tip = (x, y, z)
        return True

    def _swing_to(self, x, y, z):
        self._tip = None
        return (yield from self._line_to(x, y, z))

    def _grip(self, angle):
        ok = yield from self._move_to([*self.cur[:3], angle])
        if ok:
            yield SETTLE_MS * 1000          # let the jaws settle
        return ok

    def _go_home(self):
        self.phase = "HOME"
        self._tip = None
        return (yield from self._move_to([*c.HOME, c.GRIP_OPEN]))

    def _pick(self, x, y):
        if any(solve_ik(x, y, z) is None for z in (c.Z_GRIP, c.Z_HOVER, c.Z_CARRY)):
            self._emit("ERR target out of reach")
            return False
        self._emit(f"# pick at {_print_float(x, 0)} {_print_float(y, 0)}")
        ok = True
        self.phase = "APPROACH"
        ok = ok and (yield from self._grip(c.GRIP_OPEN)) and (yield from self._swing_to(x, y, c.Z_HOVER))
        self.phase = "DESCEND"
        ok = ok and (yield from self._line_to(x, y, c.Z_GRIP))
        self.phase = "GRIP"
        ok = ok and (yield from self._grip(c.GRIP_CLOSED))
        self.phase = "LIFT"
        ok = ok and (yield from self._line_to(x, y, c.Z_CARRY))
        self.phase = "TO_BIN"
        ok = ok and (yield from self._swing_to(*c.BIN_XY, c.Z_CARRY))
        self.phase = "RELEASE"
        ok = ok and (yield from self._grip(c.GRIP_OPEN))
        ok = ok and (yield from self._go_home())
        return ok

    @staticmethod
    def _parse_floats(text):
        """Numbers at the start of `text`, up to four, stopping at anything that is not a finite number."""
        values, at = [], 0
        while len(values) < 4:
            found = _NUMBER.match(text, at)
            if not found or not math.isfinite(float(found.group(1))):
                break
            values.append(float(found.group(1)))
            at = found.end()
        return values

    def _execute(self, text):
        v = self._parse_floats(text[1:])
        kind, ok = text[0], False
        if kind == "P" and len(v) == 2:
            ok = yield from self._pick(*v)
        elif kind == "G" and len(v) == 3:
            self.phase = "MOVE"
            ok = yield from self._line_to(*v)
        elif kind == "J" and len(v) == 4:
            self.phase, self._tip = "MOVE", None
            ok = yield from self._move_to([min(180.0, max(0.0, a)) for a in v])
        elif kind in "OC":
            self.phase = "MOVE"
            ok = yield from self._grip(c.GRIP_OPEN if kind == "O" else c.GRIP_CLOSED)
        elif kind == "H":
            ok = yield from self._go_home()
        elif kind == "X":
            ok = True                       # nothing was moving
        elif kind in "ZK":
            if len(v) == 3:
                limit = 20.0 if kind == "Z" else 15.0
                trim = [float(np.float32(min(limit, max(-limit, value)))) for value in v]
                if kind == "Z":
                    self.trim_offset = trim
                else:
                    self.trim_gain = trim
                self.eeprom["offset"], self.eeprom["gain"] = tuple(self.trim_offset), tuple(self.trim_gain)
            if len(v) in (0, 3):
                self._print_trim()
                ok = True
        elif kind == "?":
            self._emit("# P x y | G x y z | J b s e g | O | C | H | X | M ms | Z b s e | K b s e")
            self._print_trim()
            ok = True
        return ok

    def _loop(self):
        """One pass of the sketch's loop() while nothing is moving."""
        self._poll_serial()
        i = self._next_arrived()
        if i is not None:
            self._cmd, self._queue[i] = self._queue[i][1], None
            self._task = self._execute(self._cmd)
            self._run_task()
        if self._task is None:
            if self._millis() - self._last_telemetry >= HEARTBEAT_MS:
                self._telemetry(force=True)
            self._next_pass = self._now + LOOP_US

    # --- stepping ---

    def _run_task(self):
        """Run the current command up to its next delay(), or to its end."""
        try:
            self._resume = self._now + next(self._task)
        except StopIteration as done:
            self._task = None
            self.phase = "IDLE"
            self._telemetry(force=True)
            self._emit(("OK " if done.value else "ERR ") + self._cmd)
            self._next_pass = self._now + LOOP_US

    def _next_idle_event(self):
        """When the idle loop next has something to do: a line arriving, a queued command coming due, or the heartbeat."""
        times = [(self._last_telemetry + HEARTBEAT_MS) * 1000]
        if self._wire:
            times.append(-(-self._wire[0][0] // LOOP_US) * LOOP_US)      # the first pass after its last byte lands
        times += [slot[0] * 1000 for slot in self._queue if slot]
        return max(self._next_pass, min(times))


@dataclass(frozen=True)
class Conditions:
    """How badly the simulated webcam sees the world. The defaults are a clean, evenly lit view."""
    exposure: float = 1.0               # overall brightness gain
    cast: tuple = (1.0, 1.0, 1.0)       # colour gains (B, G, R): warm bulbs have R > B
    gradient: float = 0.0               # light falling off across the image, 0 = even
    vignette: float = 0.0               # darker corners
    shadow: float = 0.0                 # 0..1 darkness of a soft shadow lying over the pick area
    shadow_at: tuple = (115.0, 10.0)    # where the shadow sits, table mm
    glare: float = 0.0                  # 0..1 white specular spot on the sample
    blur: float = 0.0                   # gaussian sigma in pixels (out of focus)
    noise: float = 4.0                  # sensor noise, grey levels
    distortion: float = 0.0             # barrel distortion; webcams are typically -0.05 to 0.15
    jpeg: int = 0                       # JPEG quality (0 = off); webcams send compressed frames
    decoys: tuple = ()                  # extra objects on the table: (x mm, y mm, radius mm, (b, g, r))


_ORANGE_THINGS = ((60, 130, 215), (70, 150, 225))     # a tan desk item and a pale orange sticker (b, g, r)

# Ways a real webcam differs from the clean default. tests/ and robustness.py run every one of them.
PRESETS = {
    "clean": Conditions(),
    "dim room": Conditions(exposure=0.45),
    "overexposed": Conditions(exposure=1.45),
    "warm bulb": Conditions(cast=(0.7, 0.92, 1.1)),
    "cold daylight": Conditions(cast=(1.15, 1.0, 0.82)),
    "one-sided light": Conditions(gradient=0.55),
    "vignette": Conditions(vignette=0.45),
    "soft shadow": Conditions(shadow=0.5),
    "glare on sample": Conditions(glare=0.9),
    "out of focus": Conditions(blur=2.0),
    "noisy sensor": Conditions(noise=14),
    "lens distortion": Conditions(distortion=0.12),
    "jpeg artifacts": Conditions(jpeg=25),
    # Two outside the pick area, a speck inside it, and a big sticker half off its edge.
    "orange decoys": Conditions(decoys=((20, 50, 12, _ORANGE_THINGS[0]), (25, -62, 18, _ORANGE_THINGS[1]),
                                        (105, 30, 5, _ORANGE_THINGS[0]), (170, -15, 20, _ORANGE_THINGS[1]))),
    "everything bad": Conditions(exposure=0.6, cast=(0.8, 0.95, 1.08), gradient=0.35, vignette=0.3,
                                 shadow=0.35, glare=0.5, blur=1.5, noise=10, distortion=0.06, jpeg=40),
}


class SimWorld:
    """Simulated webcam looking down at the workcell from the camera boom.

    It is a real pinhole camera in 3D, not a flat picture: the sample and the arm
    stand up off the table, so from an off-centre camera they appear shifted
    (parallax), exactly the error a real overhead camera makes.
    """

    WIDTH, HEIGHT = 640, 480
    CAMERA_AT = (120.0, 0.0, 350.0)     # where the boom in cad/build_parts.py holds the webcam, mm
    AIM_AT = (80.0, -45.0)              # the table point in the middle of the picture
    FOCAL_PX = 540.0                    # about a 61 degree wide lens
    ROLL_DEG = 2.0                      # nobody mounts a webcam perfectly square
    GRAB_TOLERANCE_MM = 6.0             # how far off-centre the open jaws can be and still close on the sample
    PAPER_X = (c.MAT_TOP_X - c.MAT_PAGE_MM[1], c.MAT_TOP_X)      # the printed work mat
    PAPER_Y = (c.MAT_LEFT_Y - c.MAT_PAGE_MM[0], c.MAT_LEFT_Y)
    # The sample (cad/build_parts.py make_rock): a 13 mm faceted ball with its bottom 5 mm cut off.
    ROCK_RINGS = ((0.0, 10.2), (8.0, 13.0), (15.0, 11.0), (19.0, 7.0))     # height above the table, radius

    def __init__(self, firmware, rock_xy=(120.0, 20.0), seed=1, conditions=None):
        self.firmware = firmware
        self.rock_xy = rock_xy
        self.rock_state = "table"           # table | held | bin
        self.slippery = False               # True: the jaws never manage to hold the sample
        self.servo_offset = (0.0, 0.0, 0.0)  # degrees each real servo is off by (base, shoulder, elbow)
        self.servo_gain = (0.0, 0.0, 0.0)    # how much more (+) or less (-) each one turns than asked, as a fraction
        self.tag_on_arm = True              # the calibration tag is stuck on the forearm
        self.conditions = conditions or Conditions()
        self.shift = (0.0, 0.0)             # camera knocked sideways after calibration, pixels
        self._rng = np.random.default_rng(seed)

        # Camera pose: table +x is up the picture, +y to the left, as you would sit behind the arm.
        eye = np.float64(self.CAMERA_AT)
        forward = np.float64([*self.AIM_AT, 0.0]) - eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [1.0, 0.0, 0.0])
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        roll = math.radians(self.ROLL_DEG)
        right, down = (math.cos(roll) * right + math.sin(roll) * down,
                       -math.sin(roll) * right + math.cos(roll) * down)
        self._eye, self._rotation = eye, np.array([right, down, forward])
        self._intrinsics = np.float64([[self.FOCAL_PX, 0, self.WIDTH / 2], [0, self.FOCAL_PX, self.HEIGHT / 2], [0, 0, 1]])
        offset = -self._rotation @ eye
        self._camera = self._intrinsics @ np.column_stack([self._rotation[:, 0], self._rotation[:, 1], offset])  # z = 0

        grain = cv2.GaussianBlur(self._rng.normal(0, 18, (self.HEIGHT, self.WIDTH)).astype(np.float32), (0, 0), 6)
        self._desk = np.clip(np.float32([45, 75, 125]) + grain[..., None], 0, 255).astype(np.uint8)
        angles = np.linspace(0, 2 * np.pi, 11)[:-1]
        shape = np.array([(r * math.cos(a), r * math.sin(a)) for a, r in zip(angles, self._rng.uniform(0.85, 1.12, 10))])
        nxt = np.roll(shape, -1, axis=0)
        cross = shape[:, 0] * nxt[:, 1] - nxt[:, 0] * shape[:, 1]
        centroid = ((shape + nxt) * cross[:, None]).sum(axis=0) / (3 * cross.sum())   # polygon area centroid
        self._rock_shape = shape - centroid                 # unit outline, so rock_xy is exactly its middle

    # --- camera ---

    def _project(self, x, y, z=0.0):
        """Table mm -> pixel in an ideal (undistorted) camera. Drawing happens in this space."""
        p = self._intrinsics @ (self._rotation @ (np.float64([x, y, z]) - self._eye))
        return float(p[0] / p[2]), float(p[1] / p[2])

    def _scale(self, x, y, z=0.0):
        """Pixels per mm for something at this point."""
        return self.FOCAL_PX / float(self._rotation[2] @ (np.float64([x, y, z]) - self._eye))

    def _distort(self, u, v):
        k = self.conditions.distortion
        cx, cy, f = self.WIDTH / 2, self.HEIGHT / 2, float(self.WIDTH)
        scale = 1 + k * (((u - cx) / f) ** 2 + ((v - cy) / f) ** 2)
        return cx + (u - cx) * scale, cy + (v - cy) * scale

    def pixel(self, x, y, z=0.0):
        """Table mm -> where that point really lands in the camera image (lens distortion included)."""
        return self._distort(*self._project(x, y, z))

    def calibration_pixels(self):
        """Where the mat's four hand-calibration crosses appear in the image."""
        return [self.pixel(x, y) for x, y in c.CAL_POINTS_MM]

    # --- the world ---

    def respawn_rock(self):
        self.rock_xy = (float(self._rng.uniform(c.PICK_X[0] + 5, c.PICK_X[1] - 5)),
                        float(self._rng.uniform(c.PICK_Y[0] + 5, c.PICK_Y[1] - 5)))
        self.rock_state = "table"

    def joint_angles(self):
        """Where the three joints really are: what each servo was told, bent by that servo's own error."""
        return [90 + (told - 90) * (1 + gain) + offset
                for told, offset, gain in zip(self.firmware.servo, self.servo_offset, self.servo_gain)]

    def update(self):
        """Let the gripper pick up and drop the rock."""
        grip = self.firmware.servo[3]
        _, (jx, jy, jz) = forward(*self.joint_angles())
        if self.rock_state == "table" and grip >= c.GRIP_CLOSED - 3 and jz < 25 and not self.slippery \
                and math.dist((jx, jy), self.rock_xy) < self.GRAB_TOLERANCE_MM:
            self.rock_state = "held"
        elif self.rock_state == "held":
            self.rock_xy = (jx, jy)
            if grip < c.GRIP_CLOSED - 20:   # jaws opened: it falls
                in_bin = math.dist(self.rock_xy, c.BIN_XY) < c.BIN_RADIUS - 10
                self.rock_state = "bin" if in_bin else "table"
                if in_bin:
                    self.rock_xy = c.BIN_XY

    # --- drawing ---

    def _poly(self, points, z=0.0):
        return np.int32([self._project(x, y, z) for x, y in points])

    def _circle(self, cx, cy, radius, z=0.0):
        return self._poly([(cx + radius * math.cos(a), cy + radius * math.sin(a))
                           for a in np.linspace(0, 2 * np.pi, 40)], z)

    def _draw_marker(self, img, marker_id):
        """One printed ArUco marker, lying on the mat with its top toward table +x."""
        cx, cy = c.CAL_MARKERS_MM[marker_id]
        h = c.MARKER_SIZE_MM / 2
        self._paste_marker(img, marker_id, [self._project(x, y) for x, y in
                                            ((cx + h, cy + h), (cx + h, cy - h), (cx - h, cy - h), (cx - h, cy + h))])

    def _draw_arm_tag(self, img, joints):
        """The calibration tag on the forearm: a white paper patch with the marker in its middle."""
        centre = np.float64(tag_position(*joints))
        along, across = (np.float64(axis) for axis in forearm_axes(*joints))
        corners = lambda half: [self._project(*(centre + half * (i * along + j * across)))
                                for i, j in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        cv2.fillConvexPoly(img, np.int32(corners(c.ARM_TAG_PATCH_MM / 2)), (238, 240, 240), cv2.LINE_AA)
        self._paste_marker(img, c.ARM_TAG_ID, corners(c.ARM_TAG_SIZE_MM / 2))

    def _paste_marker(self, img, marker_id, corners):
        """Draw an ArUco marker into the picture with its four corners at these pixels."""
        side = 48
        quad = np.float32(corners)
        edge = side - 0.5                       # pixel centres run 0..side-1, so the picture spans -0.5..side-0.5
        to_image = cv2.getPerspectiveTransform(np.float32([[-0.5, -0.5], [edge, -0.5], [edge, edge], [-0.5, edge]]), quad)
        size = (img.shape[1], img.shape[0])
        picture = cv2.warpPerspective(marker_image(marker_id, side), to_image, size, flags=cv2.INTER_AREA).astype(np.float32)
        alpha = cv2.warpPerspective(np.full((side, side), 255, np.uint8), to_image, size).astype(np.float32) / 255
        img[:] = img * (1 - alpha[..., None]) + picture[..., None] * alpha[..., None]

    def _draw_rock(self, img, base_z=0.0):
        """The sample as a solid: every ring of it is projected, so its picture shifts with height as a real one does."""
        x, y = self.rock_xy
        rings = [self._poly([(x + r * dx, y + r * dy) for dx, dy in self._rock_shape], base_z + z)
                 for z, r in self.ROCK_RINGS]
        cv2.fillConvexPoly(img, cv2.convexHull(np.vstack(rings)), (12, 90, 225), cv2.LINE_AA)        # shaded sides
        cv2.fillConvexPoly(img, cv2.convexHull(np.vstack(rings[2:])), (30, 120, 255), cv2.LINE_AA)   # lit top facets
        if self.conditions.glare:
            spot = self._circle(x - 2, y + 2, 4.5, base_z + 19.0)
            glare = np.zeros(img.shape[:2], np.float32)
            cv2.fillPoly(glare, [spot], 1.0, cv2.LINE_AA)
            glare = cv2.GaussianBlur(glare, (0, 0), 2)[..., None] * self.conditions.glare
            img[:] = np.clip(img * (1 - glare) + np.float32([235, 240, 250]) * glare, 0, 255)

    def _draw_link(self, img, a, b, width_mm, colour):
        middle = [(p + q) / 2 for p, q in zip(a, b)]
        to_px = lambda p: tuple(map(int, self._project(*p)))
        cv2.line(img, to_px(a), to_px(b), colour, max(1, round(width_mm * self._scale(*middle))), cv2.LINE_AA)

    def _static(self):
        """Things that never change between frames, built once: the table, the light and the lens."""
        k = self.conditions
        if getattr(self, "_static_for", None) is k:
            return
        self._static_for = k
        h, w = self.HEIGHT, self.WIDTH
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)

        img = self._desk.copy()
        (px0, px1), (py0, py1) = self.PAPER_X, self.PAPER_Y
        cv2.fillPoly(img, [self._poly([(px0, py0), (px0, py1), (px1, py1), (px1, py0)])], (232, 235, 235))
        (x0, x1), (y0, y1) = c.PICK_X, c.PICK_Y
        cv2.polylines(img, [self._poly([(x0, y0), (x0, y1), (x1, y1), (x1, y0)])], True, (170, 170, 170), 1)
        for x, y in c.CAL_POINTS_MM:
            cv2.drawMarker(img, tuple(map(int, self._project(x, y))), (40, 40, 40), cv2.MARKER_CROSS, 10, 1)
        for marker_id in range(len(c.CAL_MARKERS_MM)):
            self._draw_marker(img, marker_id)
        for x, y, radius, colour in k.decoys:
            cv2.fillPoly(img, [self._circle(x, y, radius)], colour, cv2.LINE_AA)
        cv2.fillPoly(img, [self._circle(*c.BIN_XY, c.BIN_RADIUS)], (175, 175, 175), cv2.LINE_AA)
        cv2.fillPoly(img, [self._circle(*c.BIN_XY, c.BIN_RADIUS - 5)], (120, 120, 120), cv2.LINE_AA)
        self._table = img

        # Exposure, colour cast, uneven light and shadows: everything that happens before the lens.
        light = np.ones((h, w, 3), np.float32) * k.exposure * np.float32(k.cast)
        if k.gradient:
            light *= (1 - k.gradient * np.linspace(0, 1, w, dtype=np.float32))[None, :, None]
        if k.vignette:
            r2 = ((xx - w / 2) ** 2 + (yy - h / 2) ** 2) / ((w / 2) ** 2 + (h / 2) ** 2)
            light *= (1 - k.vignette * r2)[..., None]
        if k.shadow:
            mask = np.zeros((h, w), np.float32)
            cv2.fillPoly(mask, [self._circle(*k.shadow_at, 45.0)], 1.0)
            light *= (1 - k.shadow * cv2.GaussianBlur(mask, (0, 0), 14))[..., None]
        self._light = light

        self._flat_light = bool(np.all(light == 1.0))
        self._noise = [(self._rng.standard_normal((h, w, 3), dtype=np.float32) * k.noise) for _ in range(6)]

        self._lens = None
        if k.distortion:
            src_u, src_v = xx.copy(), yy.copy()
            for _ in range(8):                        # invert the distortion by fixed-point iteration
                scale = 1 + k.distortion * (((src_u - w / 2) / w) ** 2 + ((src_v - h / 2) / w) ** 2)
                src_u, src_v = w / 2 + (xx - w / 2) / scale, h / 2 + (yy - h / 2) / scale
            self._lens = (src_u, src_v)

    def _lens_and_sensor(self, img):
        k = self.conditions
        h, w = img.shape[:2]
        if self._lens:
            img = cv2.remap(img, *self._lens, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        if any(self.shift):
            img = cv2.warpAffine(img, np.float32([[1, 0, self.shift[0]], [0, 1, self.shift[1]]]), (w, h),
                                 borderMode=cv2.BORDER_REPLICATE)
        if k.blur:
            img = cv2.GaussianBlur(img, (0, 0), k.blur)
        # Sensor noise: fresh random numbers for every frame are the slowest part of a render, so a
        # few noise fields are made once and each frame gets one of them, flipped at random.
        field = self._noise[self._rng.integers(len(self._noise))]
        flip = int(self._rng.integers(4))
        if flip:
            field = cv2.flip(field, flip - 2)
        img = cv2.add(img, field, dtype=cv2.CV_8U)
        if k.jpeg:
            img = cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, k.jpeg])[1], cv2.IMREAD_COLOR)
        return img

    def render(self):
        self._static()
        img = self._table.copy()
        if self.rock_state != "held":
            self._draw_rock(img)

        blue, light_blue, dark = (190, 95, 35), (215, 125, 60), (45, 40, 40)
        cv2.fillPoly(img, [self._poly([(-55, -35), (-55, 35), (45, 35), (45, -35)], 3.0)], blue, cv2.LINE_AA)
        cv2.fillPoly(img, [self._circle(0, 0, 30, 22.0)], light_blue, cv2.LINE_AA)
        cv2.fillPoly(img, [self._circle(0, 0, 29, c.BASE_H)], light_blue, cv2.LINE_AA)
        joints = self.joint_angles()
        elbow, jaws = forward(*joints)
        self._draw_link(img, (0.0, 0.0, c.BASE_H), elbow, 17.0, blue)
        self._draw_link(img, elbow, jaws, 12.0, light_blue)
        if self.tag_on_arm:
            self._draw_arm_tag(img, joints)
        if self.rock_state == "held":
            self._draw_rock(img, base_z=jaws[2] - 8.0)
        cv2.circle(img, tuple(map(int, self._project(*jaws))), 4, dark, -1, cv2.LINE_AA)

        if not self._flat_light:
            img = cv2.multiply(img, self._light, dtype=cv2.CV_8U)      # clips at white, as an overexposed sensor does
        return self._lens_and_sensor(img)
