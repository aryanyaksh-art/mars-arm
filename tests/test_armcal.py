"""Calibrating the arm's servos with the camera.  Run with:  python -m pytest tests"""
import math

import numpy as np
import pytest

from conftest import Rig, real_firmware_or_skip
from groundstation import armcal
from groundstation import config as c
from groundstation.hostfw import HostFirmware
from groundstation.kinematics import forward, solve_ik, tag_position
from groundstation.simulator import FirmwareSim, SimWorld
from groundstation.vision import Calibration, find_arm_tag

ERRORS = ((6.0, -7.0, 5.0), (0.04, -0.03, 0.05))       # degrees, fractions: about what cheap servos have


def logged(rig, text):
    return any(text in entry for _, entry in rig.station.log)


def bad_servos(rig, errors=ERRORS):
    rig.world.servo_offset, rig.world.servo_gain = errors
    return rig


def calibrate(rig):
    assert rig.run(5, until=rig.station.calibrate_arm)      # starts as soon as the station has its first frame
    assert rig.run(150, until=lambda: rig.station.routine is None)


# --- the maths ---

def test_tag_is_seen_where_the_kinematics_say_it_is():
    firmware = FirmwareSim()
    world = SimWorld(firmware)
    for pose in armcal.POSES + armcal.CHECK_POSES:
        firmware.cur = [*pose, c.GRIP_OPEN]
        seen = find_arm_tag(world.render())
        assert seen is not None, f"tag not visible at {pose}"
        assert math.dist(seen, world.pixel(*tag_position(*pose))) < 0.5


@pytest.mark.parametrize("seed", range(6))
def test_fit_recovers_servo_errors_and_where_the_camera_is(seed):
    rng = np.random.default_rng(seed)
    offsets, gains = rng.uniform(-8, 8, 3), rng.uniform(-0.06, 0.06, 3)
    world = SimWorld(FirmwareSim())
    calibration = Calibration.from_frame(world.render())
    sightings = [np.add(world.pixel(*tag_position(*armcal.real_angles(pose, offsets, gains))), rng.normal(0, 0.3, 2))
                 for pose in armcal.POSES]
    result = armcal.fit(calibration, armcal.POSES, sightings)
    assert result.believable and result.rms_px < 0.6
    assert np.abs(np.subtract(result.offsets, offsets)).max() < 1.5
    assert np.abs(np.subtract(result.gains, gains)).max() < 0.025
    assert math.dist(result.camera_mm, world.CAMERA_AT) < 12
    # What matters: after correcting with the fit, the jaws go where they are sent.
    for target in ((95, -35, c.Z_GRIP), (135, 55, c.Z_GRIP), (115, 10, c.Z_HOVER)):
        want = solve_ik(*target)
        told = [90 + (a - 90 - o) / (1 + g) for a, o, g in zip(want, result.offsets, result.gains)]
        _, jaws = forward(*armcal.real_angles(told, offsets, gains))
        assert math.dist(jaws, target) < 3.0


def test_fit_refuses_to_guess_from_too_few_sightings():
    calibration = Calibration.from_frame(SimWorld(FirmwareSim()).render())
    sightings = [(300.0, 200.0)] * 5 + [None] * (len(armcal.POSES) - 5)
    assert armcal.fit(calibration, armcal.POSES, sightings) is None


def test_fit_is_not_believed_when_the_sightings_make_no_sense():
    calibration = Calibration.from_frame(SimWorld(FirmwareSim()).render())
    rng = np.random.default_rng(0)
    nonsense = [tuple(rng.uniform(100, 500, 2)) for _ in armcal.POSES]
    assert not armcal.fit(calibration, armcal.POSES, nonsense).believable


# --- the whole routine, on both firmwares ---

def test_uncalibrated_servos_make_the_arm_miss(rig):
    bad_servos(rig)
    rig.station.toggle_auto()
    assert rig.run(60, until=lambda: rig.station.misses >= 1)
    assert rig.station.collected == 0


def test_calibration_measures_the_servos_and_the_arm_then_picks(rig):
    bad_servos(rig)
    calibrate(rig)
    offsets, gains = rig.station.trims                   # what the arm reports it is now using
    assert offsets == pytest.approx(ERRORS[0], abs=1.0)
    assert [g / 100 for g in gains] == pytest.approx(ERRORS[1], abs=0.015)
    assert rig.station.arm_check_mm < 1.5 and logged(rig, "arm calibrated")
    assert rig.station.phase == "IDLE"
    rig.station.toggle_auto()
    for done, spot in enumerate(((120.0, 20.0), (92.0, -38.0), (138.0, 58.0)), start=1):
        rig.world.rock_xy, rig.world.rock_state = spot, "table"
        assert rig.run(40, until=lambda: rig.station.collected == done), f"missed the sample at {spot}"
    assert rig.station.misses == 0


def test_calibrating_good_servos_changes_almost_nothing(rig):
    calibrate(rig)
    offsets, gains = rig.station.trims
    assert max(map(abs, offsets)) < 1.0 and max(map(abs, gains)) < 1.5
    assert rig.station.arm_check_mm < 1.5


def test_calibration_gives_up_and_restores_the_old_trim_when_the_tag_is_missing(rig):
    rig.station.send("Z 1 2 3")
    rig.station.send("K 0.5 -0.5 1")
    rig.run(1)
    rig.world.tag_on_arm = False
    calibrate(rig)
    rig.run(3)
    assert logged(rig, "ARM CALIBRATION FAILED") and rig.station.arm_fit is None
    assert rig.station.trims == ((1.0, 2.0, 3.0), (0.5, -0.5, 1.0))
    assert rig.station.phase == "IDLE"


def test_abort_cancels_calibration_and_restores_the_old_trim(rig):
    rig.station.send("Z 1 2 3")
    rig.run(1)
    assert rig.station.calibrate_arm()
    rig.run(8)
    assert rig.station.routine_status.startswith("MEASURING ARM")
    rig.station.abort()
    rig.run(3)
    assert rig.station.routine is None and logged(rig, "cancelled")
    assert rig.station.trims[0] == (1.0, 2.0, 3.0)


def test_calibration_will_not_start_when_it_should_not(rig):
    rig.run(1)
    rig.station.toggle_mars()
    assert not rig.station.calibrate_arm()               # not with the comms delay on
    rig.station.toggle_mars()
    rig.run(1)
    assert rig.station.pick()
    assert not rig.station.calibrate_arm()               # not while a pick is under way
    assert rig.run(30, until=lambda: rig.station.collected == 1)
    assert rig.station.calibrate_arm()
    assert not rig.station.pick()                        # and no picks while it runs


def test_no_picks_are_sent_while_calibrating(rig):
    bad_servos(rig)
    rig.run(0.5)
    rig.station.toggle_auto()
    assert rig.station.calibrate_arm()                   # this switches autonomy off
    rig.run(20)
    assert not rig.lines("# pick at") and not rig.station.auto


# --- the trim is kept by the board ---

def test_trim_survives_a_power_cycle_in_the_imitation():
    eeprom = {}
    first = FirmwareSim(eeprom)
    first.write("Z 5 -7.5 3")
    first.write("K 4 -3 2")
    first.advance(1)
    second = FirmwareSim(eeprom)
    assert "# trim 5.00 -7.50 3.00 gain 4.00 -3.00 2.00" in second.read_lines()
    assert second.servo[:3] == pytest.approx(first.servo[:3])


def test_trim_survives_a_power_cycle_on_the_real_firmware(tmp_path):
    real_firmware_or_skip().close()
    eeprom = tmp_path / "eeprom.bin"
    first = HostFirmware(eeprom=eeprom)
    first.write("Z 5 -7.5 3")
    first.write("K 4 -3 2")
    first.advance(1)
    first.close()
    second = HostFirmware(eeprom=eeprom)
    assert "# trim 5.00 -7.50 3.00 gain 4.00 -3.00 2.00" in second.read_lines()
    second.close()


def test_real_firmware_ignores_a_corrupted_trim(tmp_path):
    real_firmware_or_skip().close()
    eeprom = tmp_path / "eeprom.bin"
    garbage = bytearray(b"\xff" * 1024)
    garbage[0] = 0x5A                                    # says "a trim is stored", followed by rubbish
    garbage[1:25] = bytes(range(200, 224))
    eeprom.write_bytes(garbage)
    board = HostFirmware(eeprom=eeprom)
    assert "# trim 0.00 0.00 0.00 gain 0.00 0.00 0.00" in board.read_lines()
    assert board.servo[:3] == pytest.approx(c.HOME, abs=0.2)
    board.close()
