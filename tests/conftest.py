"""Shared test helpers: the two firmwares (Python imitation and the real sketch) and the full rig."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))      # so tests can import the project

from groundstation import hostfw
from groundstation.simulator import FirmwareSim, SimWorld
from groundstation.station import GroundStation, SimLink
from groundstation.vision import Calibration

FPS = 30


def real_firmware_or_skip():
    try:
        hostfw.build()
    except RuntimeError as problem:
        pytest.skip(f"real firmware not available: {problem}")
    return hostfw.HostFirmware()


@pytest.fixture(params=["imitation", "real"])
def firmware(request):
    """Every test that takes this runs twice: against FirmwareSim and against the compiled sketch."""
    if request.param == "imitation":
        yield FirmwareSim()
    else:
        fw = real_firmware_or_skip()
        yield fw
        fw.close()


class Rig:
    """Simulated camera + ground station around a firmware, stepped in simulated time."""

    def __init__(self, firmware, rock_xy=(120.0, 20.0), conditions=None, seed=1):
        self.firmware = firmware
        self.world = SimWorld(firmware, rock_xy, seed=seed, conditions=conditions)
        self.station = GroundStation(SimLink(firmware), Calibration.from_frame(self.world.render()))
        self.now = 0.0
        self.serial_log = []
        self.camera_on = True
        read = firmware.read_lines
        firmware.read_lines = lambda: self._tap(read())
        self.station.link.read_lines = firmware.read_lines

    def _tap(self, lines):
        self.serial_log += [(self.now, line) for line in lines]
        return lines

    def run(self, seconds, until=None):
        for _ in range(int(seconds * FPS)):
            self.now += 1 / FPS
            self.firmware.advance(1 / FPS)
            self.world.update()
            self.station.update(self.world.render() if self.camera_on else None, self.now)
            if until and until():
                return True
        return until is None

    def lines(self, prefix):
        return [(t, line) for t, line in self.serial_log if line.startswith(prefix)]


@pytest.fixture
def rig(firmware):
    return Rig(firmware)
