"""Runs the real sketch (mars_arm/mars_arm.ino), compiled for the PC, in place of FirmwareSim.

FirmwareSim is a Python imitation of the firmware. This is the firmware: the same
.ino file, built with a small stand-in for the Arduino core (firmware_host/) and
run as a separate program that only moves forward in time when told to. It has
the same interface as FirmwareSim, so the simulated camera and the whole ground
station can be tested against it.

Needs a C++ compiler: g++ on Linux/macOS, or g++ inside WSL on Windows.
"""
import hashlib
import os
import shutil
import subprocess
import sys

from . import config as c

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCES = [os.path.join(ROOT, "firmware_host", name) for name in ("host.cpp", "Arduino.h", "Servo.h", "EEPROM.h")] \
    + [os.path.join(ROOT, "mars_arm", "mars_arm.ino")]
FLAGS = "-std=c++17 -O1 -Wall -fsingle-precision-constant"     # single precision: AVR doubles are 32-bit
WINDOWS = sys.platform == "win32"
_binary = None


def _run(command, **kwargs):
    return subprocess.run(command, capture_output=True, text=True, **kwargs)


def _wsl_path(path):
    drive, rest = os.path.splitdrive(os.path.abspath(path))
    return f"/mnt/{drive[0].lower()}{rest.replace(os.sep, '/')}"


def _version():
    """A short fingerprint of the sources, so each version of the sketch is compiled once and never
    rebuilt underneath a copy that is running (tests run several at a time)."""
    digest = hashlib.sha1()
    for path in SOURCES:
        with open(path, "rb") as f:
            digest.update(f.read())
    return digest.hexdigest()[:12]


def build():
    """Compile the sketch for this PC. Returns the command that runs it, or raises RuntimeError."""
    global _binary
    if _binary:
        return _binary
    name = "firmware_host-" + _version()
    if WINDOWS:
        if not shutil.which("wsl.exe"):
            raise RuntimeError("no C++ compiler: WSL is not installed")
        script = (f'out="$HOME/.cache/mars-arm/{name}"; mkdir -p "$HOME/.cache/mars-arm" && cd "{_wsl_path(ROOT)}" && '
                  f'{{ [ -x "$out" ] || {{ g++ {FLAGS} -I firmware_host -o "$out.$$" firmware_host/host.cpp '
                  f'&& mv "$out.$$" "$out"; }}; }} && echo "$out"')
        done = _run(["wsl.exe", "-e", "sh", "-c", script])
        if done.returncode != 0:
            raise RuntimeError("could not build the firmware in WSL:\n" + (done.stderr or done.stdout).replace("\0", ""))
        _binary = ["wsl.exe", "-e", done.stdout.strip().splitlines()[-1]]
    else:
        compiler = shutil.which("g++") or shutil.which("c++")
        if not compiler:
            raise RuntimeError("no C++ compiler found")
        out = os.path.join(ROOT, "firmware_host", "build", name)
        if not os.path.exists(out):
            os.makedirs(os.path.dirname(out), exist_ok=True)
            partial = f"{out}.{os.getpid()}"
            done = _run([compiler, *FLAGS.split(), "-I", os.path.join(ROOT, "firmware_host"), "-o", partial, SOURCES[0]])
            if done.returncode != 0:
                raise RuntimeError("could not build the firmware:\n" + done.stderr)
            os.replace(partial, out)
        _binary = [out]
    return _binary


class HostFirmware:
    """The real firmware, stepped in virtual time. Drop-in for simulator.FirmwareSim."""

    def __init__(self, eeprom=None):
        """`eeprom` is a file that stands for the board's EEPROM, so a second HostFirmware given
        the same file remembers what the first one stored. Without it the EEPROM starts erased."""
        command = build()
        if eeprom:
            command = [*command, _wsl_path(eeprom) if WINDOWS else str(eeprom)]
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.servo = [*c.HOME, c.GRIP_OPEN]     # the angles the sketch is commanding its four servos to
        self.phase = "IDLE"
        self.now_ms = 0.0
        self.led = False
        self.rx_dropped = 0             # bytes the board lost because the sketch was not reading
        self.button = False             # the PICK button on pin 2
        self.pots = [512, 512]          # the Wokwi sliders on A0 and A1
        self._to_board = bytearray()
        self._partial = ""
        self._out = []
        self._granted_us = 0
        self._clock = 0.0
        self._read_status()             # the board's start-up banner

    # --- same interface as FirmwareSim ---

    def write(self, text):
        self._to_board += (text.strip() + "\n").encode("ascii")

    def read_lines(self):
        lines, self._out = self._out, []
        return lines

    def advance(self, seconds):
        self._clock += seconds
        grant = round(self._clock * 1e6) - self._granted_us
        self._granted_us += grant
        sent, self._to_board = self._to_board, bytearray()
        self._process.stdin.write(f"A {grant} {int(self.button)} {self.pots[0]} {self.pots[1]} {sent.hex() or '-'}\n")
        self._process.stdin.flush()
        self._read_status()

    def close(self):
        if self._process.poll() is None:
            self._process.stdin.close()
            self._process.wait(timeout=5)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    # --- internals ---

    def _read_status(self):
        reply = self._process.stdout.readline().split()
        if len(reply) != 9 or reply[0] != "S":
            raise RuntimeError(f"firmware stopped unexpectedly: {reply!r}")
        self.now_ms = int(reply[1]) / 1000
        self.servo = [(int(us) - 544) * 180 / (2400 - 544) for us in reply[2:6]]
        self.led = reply[6] == "1"
        self.rx_dropped = int(reply[7])
        if reply[8] != "-":
            text = self._partial + bytes.fromhex(reply[8]).decode("ascii", "replace")
            *lines, self._partial = text.split("\n")
            for line in (l.strip() for l in lines):
                if line.startswith("T,") and line.count(",") == 5:
                    self.phase = line.rsplit(",", 1)[1]
                if line:
                    self._out.append(line)
