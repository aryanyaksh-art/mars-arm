"""The ground station: watches the camera, decides when to pick, talks to the arm.

In Mars mode everything coming down from the arm (video and telemetry) is held
back by the signal delay before the station sees it, and the firmware holds
every command for the same delay before acting on it.

The station does not trust anything it cannot check. A pick only counts once
the camera shows the sample gone, the camera is re-checked against the mat's
markers, and it stops sending picks if the camera, the calibration or the link
to the arm goes bad.
"""
import math
import queue
import threading
import time
from collections import deque

import numpy as np

from . import armcal
from . import config as c
from .vision import Calibration, find_arm_tag, find_rock, in_pick_area, rock_position

READY = "# Mars arm ready"      # the firmware's start-up banner


class _Stop(Exception):
    """Raised inside a routine to give up, with the reason."""


class SerialLink:
    """The real arm on a COM port. If the cable is pulled it keeps trying to reopen the port."""

    def __init__(self, port, ready_timeout=5.0, opener=None, retry_s=1.0):
        if opener is None:
            import serial  # imported here so the simulator runs without pyserial installed
            opener = lambda: serial.Serial(port, c.BAUD, timeout=0.05)
        self.name = port
        self.connected = True
        self._open, self._retry_s = opener, retry_s
        self._serial = opener()             # a wrong port name fails here, straight away
        self._lines = queue.Queue()
        self._ready = threading.Event()
        self._guard = threading.Lock()
        self._closing = False
        threading.Thread(target=self._reader, daemon=True).start()
        self._ready.wait(ready_timeout)     # opening the port resets the Uno; wait for its banner

    def _reader(self):
        while not self._closing:
            port = self._serial
            if port is None:
                try:
                    port = self._open()
                except Exception:
                    time.sleep(self._retry_s)
                    continue
                self._serial, self.connected = port, True
                self._lines.put("# link: port reopened")
            try:
                line = port.readline().decode("ascii", "replace").strip()
            except Exception:
                self._drop(port)
                continue
            if line:
                if line.startswith(READY):
                    self._ready.set()
                self._lines.put(line)

    def _drop(self, port):
        with self._guard:
            if self._serial is not port:
                return
            self._serial, self.connected = None, False
        try:
            port.close()
        except Exception:
            pass
        if not self._closing:
            self._lines.put("# link: port lost")

    def write(self, text):
        """Send one command. False if the port is down."""
        port = self._serial
        if port is None:
            return False
        try:
            port.write((text + "\n").encode("ascii"))
            return True
        except Exception:
            self._drop(port)
            return False

    def read_lines(self):
        lines = []
        while not self._lines.empty():
            lines.append(self._lines.get())
        return lines

    def close(self):
        self._closing = True
        port = self._serial
        if port is not None:
            try:
                port.close()
            except Exception:
                pass


class SimLink:
    """A simulated arm: simulator.FirmwareSim, or the real sketch via hostfw.HostFirmware."""

    name = "SIMULATOR"
    connected = True

    def __init__(self, firmware):
        self._firmware = firmware
        self.read_lines = firmware.read_lines

    def write(self, text):
        self._firmware.write(text)
        return True

    def close(self):
        pass


class GroundStation:
    def __init__(self, link, calibration):
        self.link = link
        self.calibration = calibration
        self.auto = False
        self.mars = False
        self.joints = [*c.HOME, c.GRIP_OPEN]
        self.phase = "IDLE"
        self.view = None            # the camera frame the operator is allowed to see right now
        self.detection = None       # sample found in `view`, in pixels
        self.target = None          # the same in table mm, if it is inside the pick area
        self.lock = 0               # consecutive frames the target has held still
        self.pending = None         # pick command sent and not yet reported finished
        self.collected = 0          # picks confirmed by the camera: the sample really left the pick area
        self.misses = 0             # failed picks in a row
        self.camera_ok = False      # frames are arriving
        self.link_ok = True         # the arm has been heard from lately
        self.cal_ok = True          # the calibration agrees with the mat's markers (or could not be checked)
        self.watch_markers = True   # keep checking the camera against the mat markers
        self.trims = None           # (offsets in degrees, gains in percent) the arm says it is using
        self.routine = None         # the arm calibration, while it is running
        self.routine_status = ""    # what it is doing, for the dashboard
        self.arm_fit = None         # result of the last arm calibration
        self.arm_check_mm = None    # how far off the arm still was when that result was checked
        self.cal_status = "not checked yet"
        self.log = deque(maxlen=9)
        self.uplink = []            # (arrival_time, text) for commands still travelling
        self.now = 0.0
        self._lock_anchor = None
        self._pending_since = 0.0
        self._aborted = False       # the pending pick was cancelled on purpose
        self._verify = 0            # frames left before a finished pick is judged
        self._awaiting = None       # command the routine is waiting for the arm to finish
        self._refused = False
        self._trims_before = None   # what to put back if the calibration is abandoned
        self._next_check = 0.0
        self._last_heard = None     # when the arm last sent anything
        self._view_time = None      # when the frame in `view` was taken
        self._camera_seen = False
        self._mask = None
        self._frames = deque()      # downlink in transit: (time, frame)
        self._lines = deque()       # downlink in transit: (time, line)

    @property
    def delay(self):
        return c.MARS_DELAY_S if self.mars else 0.0

    @property
    def locked(self):
        return self.target is not None and self.lock >= c.LOCK_FRAMES

    @property
    def ready(self):
        """Safe to send a pick: the camera, its calibration and the link are all good."""
        return self.camera_ok and self.cal_ok and self.link_ok

    def downlink(self):
        """(times video frames were taken, times lines were sent) for everything still on its way down."""
        return [t for t, _ in self._frames], [t for t, _ in self._lines]

    # --- operator actions ---

    def send(self, text, quiet=False):
        """Send a command to the arm. False if it could not be sent."""
        if self.link.write(text) is False:
            self._note(f"NOT SENT, link down: {text}")
            return False
        if self.delay:
            self.uplink.append((self.now + self.delay, text))
        if not quiet:
            self._note(f"> {text}")
        return True

    def pick(self):
        """Send a pick for the locked target. False if there is nothing to pick, the arm is busy,
        or the station cannot trust what it sees."""
        if not self.locked or self.pending or self._verify or self.phase != "IDLE" or not self.ready or self.routine:
            return False
        before = self.calibration
        self._check_camera(self.view, now=True)     # the camera may have been bumped since the last look
        if self.calibration is not before or not self.ready:
            return False                            # recalibrated: lock on again with the new numbers first
        command = f"P {self.target[0]:.0f} {self.target[1]:.0f}"
        if not self.send(command):
            return False
        self.pending, self._pending_since, self._aborted = command, self.now, False
        return True

    def abort(self):
        self.auto = False           # otherwise autonomy would start the same pick again
        self._aborted = True
        self.send("X")
        if self.routine:
            self._end_routine("arm calibration cancelled")

    # --- arm calibration ---

    def calibrate_arm(self):
        """Start measuring the servos' errors with the camera (see armcal.py). False if it cannot start now."""
        if self.routine or self.pending or self._verify or self.phase != "IDLE" or not self.ready:
            return False
        if self.mars:
            self._note("turn the Mars delay off to calibrate the arm")
            return False
        self.auto = False
        self.routine = self._calibrate_arm()
        return True

    def _end_routine(self, why=None, from_inside=False):
        """Stop the calibration. If it had not finished, put the arm's old trim back."""
        if self.routine and not from_inside:
            self.routine.close()
        self.routine, self.routine_status, self._awaiting = None, "", None
        if why:
            self._note(why)
        if self._trims_before:
            offsets, gains = self._trims_before
            self.link.write("Z %.2f %.2f %.2f" % offsets)
            self.link.write("K %.2f %.2f %.2f" % gains)
            self._trims_before = None

    def _command(self, text, timeout=15.0):
        """Send a command and wait until the arm reports it finished."""
        if not self.send(text, quiet=True):
            raise _Stop("the link to the arm is down")
        self._awaiting, self._refused = text, False
        deadline = self.now + timeout
        while self._awaiting:
            if self.now > deadline:
                raise _Stop(f"the arm did not answer '{text}'")
            yield
        if self._refused:
            raise _Stop(f"the arm refused '{text}'")

    def _sight_tag(self, settle=0.4, frames=8, patience=2.0):
        """Let the arm stop shaking, then return where the tag is in the picture (None if it cannot be seen)."""
        until = self.now + settle
        while self.now < until:
            yield
        seen, last, deadline = [], None, self.now + patience
        while len(seen) < frames and self.now < deadline:
            if self._view_time != last:                 # a frame we have not looked at yet
                last = self._view_time
                tag = find_arm_tag(self.view)
                if tag is not None:
                    seen.append(tag)
            yield
        return tuple(float(v) for v in np.median(seen, axis=0)) if len(seen) >= frames // 2 else None

    def _calibrate_arm(self):
        self._trims_before = self.trims
        self._note("arm calibration started")
        try:
            yield from self._command("Z 0 0 0")         # measure the servos as they really are
            yield from self._command("K 0 0 0")
            sightings = []
            for i, pose in enumerate(armcal.POSES):
                self.routine_status = f"MEASURING ARM {i + 1}/{len(armcal.POSES)}"
                yield from self._command("J %d %d %d %d" % (*pose, c.GRIP_OPEN))
                sightings.append((yield from self._sight_tag()))
            result = armcal.fit(self.calibration, armcal.POSES, sightings)
            if result is None:
                seen = sum(s is not None for s in sightings)
                raise _Stop(f"saw the tag on the arm in only {seen} of {len(sightings)} poses")
            if not result.believable:
                raise _Stop(f"the sightings do not fit an arm (off by {result.rms_px:.1f} px)")
            yield from self._command("Z %.2f %.2f %.2f" % result.offsets)
            yield from self._command("K %.2f %.2f %.2f" % tuple(100 * g for g in result.gains))
            self._trims_before = None                   # the new trim is in: nothing to put back
            self.arm_fit, self.calibration.camera_mm = result, result.camera_mm
            misses = []
            for i, pose in enumerate(armcal.CHECK_POSES):   # now test it on poses it was not fitted to
                self.routine_status = f"CHECKING ARM {i + 1}/{len(armcal.CHECK_POSES)}"
                yield from self._command("J %d %d %d %d" % (*pose, c.GRIP_OPEN))
                tag = yield from self._sight_tag()
                if tag is not None:
                    misses.append(armcal.miss_mm(self.calibration, pose, tag))
            yield from self._command("H")
            self.arm_check_mm = max(misses) if misses else None
            if not misses:
                self._note("ARM CALIBRATED but the check could not see the tag")
            elif self.arm_check_mm > c.ARM_CHECK_MM:
                self._note(f"ARM CALIBRATION POOR: still {self.arm_check_mm:.1f} mm off")
            else:
                self._note(f"arm calibrated: within {self.arm_check_mm:.1f} mm")
        except _Stop as problem:
            self._end_routine(f"ARM CALIBRATION FAILED: {problem}", from_inside=True)
            self.send("H", quiet=True)
        finally:
            self.routine_status = ""

    def home(self):
        self.send("H")

    def toggle_auto(self):
        self.auto = not self.auto
        self._note("autonomy " + ("ON" if self.auto else "OFF"))

    def toggle_mars(self):
        self.mars = not self.mars
        self.link.write(f"M {int(self.delay * 1000)}")
        self._note(f"comms delay {self.delay:.0f} s" if self.mars else "comms delay off")

    # --- per-frame update ---

    def update(self, frame, now):
        """Feed in the newest camera frame (None if the camera gave nothing). Call once per frame."""
        self.now = now
        heard = self.link.read_lines()
        if heard or self._last_heard is None:
            self._last_heard = now
        self._lines.extend((now, line) for line in heard)
        if frame is not None:
            self._frames.append((now, frame))
        visible = now - self.delay
        while self._lines and self._lines[0][0] <= visible:
            self._handle_line(self._lines.popleft()[1])
        while self._frames and self._frames[0][0] <= visible:
            self._view_time, self.view = self._frames.popleft()
        self.uplink = [u for u in self.uplink if u[0] > now]

        self._set("link_ok", now - self._last_heard <= c.LINK_SILENCE_S, "arm link back", "ARM SILENT: no telemetry")
        fresh = self._view_time is not None and visible - self._view_time <= c.CAMERA_STALE_S
        if fresh and not self._camera_seen:
            self.camera_ok = self._camera_seen = True       # the first frame: nothing to report
        self._set("camera_ok", fresh, "camera feed back", "CAMERA LOST: no frames", quiet=not self._camera_seen)
        if self.pending and now - self._pending_since > c.PICK_TIMEOUT_S + 2 * self.delay:
            self._note("ARM DID NOT ANSWER: pick given up")
            self.pending, self.auto = None, False       # something is wrong with the link; do not keep sending

        if not self.camera_ok:
            self.detection = self.target = self._lock_anchor = None
            self.lock = 0
            if self.routine:
                self._end_routine("ARM CALIBRATION FAILED: camera lost")
            return
        self._check_camera(self.view)
        self._look(self.view)
        if self.routine:
            routine = self.routine
            try:
                next(routine)
            except StopIteration:
                if self.routine is routine:
                    self.routine = None
        elif self.auto:
            self.pick()

    def _set(self, name, value, good, bad, quiet=False):
        if getattr(self, name) != value:
            setattr(self, name, value)
            if not quiet:
                self._note(good if value else bad)

    def _check_camera(self, frame, now=False):
        """Re-read the mat markers every so often (or right `now`); if the camera or mat has moved, recalibrate."""
        if not self.watch_markers or self.pending or self.routine or self.phase != "IDLE" \
                or (self.now < self._next_check and not now):
            return
        self._next_check = self.now + c.CHECK_EVERY_S
        seen, error = self.calibration.check(frame)
        if error is None:                   # markers covered or unreadable: cannot tell, so keep the last verdict
            self.cal_status = f"only {seen} of {len(c.CAL_MARKERS_MM)} markers visible"
            return
        if error <= c.DRIFT_TOLERANCE_MM:
            self.cal_ok, self.cal_status = True, f"ok, {error:.1f} mm off the markers"
            return
        better = Calibration.from_frame(frame)
        if better is not None and better.rms_mm <= c.DRIFT_TOLERANCE_MM:
            self.calibration, self._mask = better, None
            self.lock, self._lock_anchor = 0, None
            self.cal_ok, self.cal_status = True, f"moved {error:.1f} mm: recalibrated"
            self._note(f"camera moved {error:.1f} mm: recalibrated")
        else:
            if self.cal_ok:
                self._note(f"CAMERA MOVED {error:.1f} mm: picks stopped")
            self.cal_ok, self.cal_status = False, f"MOVED {error:.1f} mm, cannot recalibrate"

    def _look(self, frame):
        if self._mask is None or self._mask.shape != frame.shape[:2]:
            self._mask = self.calibration.pick_mask(frame.shape)
        self.detection = find_rock(frame, self._mask, self.calibration)
        self.target = None
        if self.detection:
            x, y = rock_position(self.calibration, self.detection)
            if in_pick_area(x, y):
                self.target = (x, y)
        if self.target is None:
            self.lock, self._lock_anchor = 0, None
        elif self._lock_anchor and math.dist(self.target, self._lock_anchor) <= c.LOCK_TOLERANCE_MM:
            self.lock += 1
        else:
            self.lock, self._lock_anchor = 1, self.target
        if self._verify:
            self._verify -= 1
            if not self._verify:
                self._judge_pick()

    def _judge_pick(self):
        """The arm said it finished: believe it only if the sample is no longer in the pick area."""
        if self.target is None:
            self.collected, self.misses = self.collected + 1, 0
            return
        self._miss(f"MISS: sample still at x {self.target[0]:.0f} y {self.target[1]:.0f}")

    def _miss(self, text):
        self.misses += 1
        self._note(text)
        if self.misses >= c.MAX_MISSES and self.auto:
            self.auto = False
            self._note(f"{self.misses} failed picks in a row: autonomy off")

    def _handle_line(self, line):
        if line.startswith("T,"):
            parts = line.split(",")
            if len(parts) == 6:
                try:
                    self.joints = [float(p) for p in parts[1:5]]
                    self.phase = parts[5]
                except ValueError:
                    pass
            return
        if self._awaiting and line in ("OK " + self._awaiting, "ERR " + self._awaiting):
            self._refused, self._awaiting = line.startswith("ERR"), None
            return
        if line.startswith("# trim "):      # the arm reporting the servo trim it is using
            try:
                numbers = [float(v) for v in line.replace("gain", "").split()[2:]]
                self.trims = (tuple(numbers[:3]), tuple(numbers[3:6]))
            except ValueError:
                pass
            if self.routine:
                return
        self._note(line)
        if line.startswith(READY):          # the board has (re)started and forgotten everything
            if self.routine:
                self._trims_before = None   # its trim is whatever its EEPROM held; nothing to put back
                self._end_routine("ARM CALIBRATION FAILED: the arm restarted")
            if self.pending:
                self._note("ARM RESTARTED: pick lost, autonomy off")
                self.pending, self.auto = None, False
            if self.mars:
                self.link.write(f"M {int(self.delay * 1000)}")
        elif self.pending and line in ("OK " + self.pending, "ERR " + self.pending):
            if line.startswith("OK"):
                self._verify = c.VERIFY_FRAMES
            elif not self._aborted:
                self._miss("arm refused the pick")
            self.pending = None

    def _note(self, text):
        self.log.append((self.now, text))
