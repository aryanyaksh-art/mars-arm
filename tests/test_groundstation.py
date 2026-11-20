"""Whole-system tests: simulated camera + ground station + firmware.  Run with:  python -m pytest tests

Every test that takes `rig` runs twice: once against the Python imitation of the
firmware and once against the real sketch compiled for the PC.
"""
import time
from collections import deque

import pytest

from conftest import Rig
from groundstation import config as c
from groundstation.kinematics import forward, solve_ik
from groundstation.simulator import PRESETS
from groundstation.station import READY, SerialLink
from groundstation.vision import Calibration


def logged(rig, text):
    return any(text in entry for _, entry in rig.station.log)


# --- kinematics ---

# Servo angles the firmware reported in Wokwi (telemetry lines) for these targets.
@pytest.mark.parametrize("target, wokwi_angles", [
    ((120, 20, 14), (107.3, 27.4, 98.2)),       # gripping at P 120 20
    ((50, -140, 75), (26.0, 36.0, 131.6)),      # over the bin
])
def test_ik_matches_the_firmware_running_in_wokwi(target, wokwi_angles):
    assert solve_ik(*target) == pytest.approx(wokwi_angles, abs=0.06)


@pytest.mark.parametrize("target", [(90, -40, 14), (140, 60, 14), (115, 10, 45), (50, -140, 75)])
def test_forward_kinematics_undoes_ik(target):
    _, jaws = forward(*solve_ik(*target))
    assert jaws == pytest.approx(target, abs=1e-6)


def test_targets_out_of_reach_are_rejected():
    assert solve_ik(200, 0, 14) is None     # too far
    assert solve_ik(20, 0, 14) is None      # folded tighter than the elbow limit


# --- missions ---

def test_autonomous_mission_puts_the_rock_in_the_bin(rig):
    rig.station.toggle_auto()
    assert rig.run(30, until=lambda: rig.station.collected == 1)
    assert rig.world.rock_state == "bin" and rig.station.phase == "IDLE" and not rig.station.pending
    rig.run(3)                               # nothing left to pick: it must not start again
    assert len(rig.lines("# pick at")) == 1 and rig.station.misses == 0


def test_manual_mode_waits_for_the_operator(rig):
    rig.run(3)
    assert rig.station.locked and not rig.lines("# pick at")
    assert rig.station.pick()
    assert rig.run(30, until=lambda: rig.world.rock_state == "bin")


def test_mars_delay_holds_commands_and_video_for_three_seconds(rig):
    rig.station.toggle_mars()
    rig.run(1)
    assert rig.station.view is None          # first frame has not arrived yet
    rig.station.toggle_auto()
    assert rig.run(60, until=lambda: rig.station.collected == 1)
    sent = next(t for t, text in rig.station.log if text.startswith("> P"))
    started = rig.lines("# pick at")[0][0]
    assert started - sent == pytest.approx(c.MARS_DELAY_S, abs=0.1)
    assert rig.world.rock_state == "bin"
    assert len(rig.lines("# pick at")) == 1  # delayed video must not trigger a second pick


def test_abort_stops_the_arm_and_turns_autonomy_off(rig):
    rig.station.toggle_auto()
    assert rig.run(30, until=lambda: rig.firmware.phase == "LIFT")
    rig.station.abort()
    rig.run(2)
    assert rig.firmware.phase == "IDLE" and rig.world.rock_state == "held"
    assert rig.lines("# ABORT") and rig.lines("ERR P")
    assert not rig.station.auto and rig.station.pending is None and rig.station.misses == 0


def test_rock_outside_the_pick_area_is_never_picked(firmware):
    rig = Rig(firmware, rock_xy=(120.0, 85.0))
    rig.station.toggle_auto()
    rig.run(5)
    assert not rig.lines("# pick at") and rig.station.target is None


def test_five_samples_in_a_row_through_a_bad_camera(firmware):
    rig = Rig(firmware, rock_xy=(95.0, 55.0), conditions=PRESETS["everything bad"], seed=7)
    rig.station.toggle_auto()
    for done in range(1, 6):
        assert rig.run(40, until=lambda: rig.station.collected == done), f"sample {done} at {rig.world.rock_xy} not collected"
        assert rig.world.rock_state == "bin"
        rig.world.respawn_rock()
    assert rig.station.misses == 0 and len(rig.lines("# pick at")) == 5


@pytest.mark.parametrize("corner", [(90, -40), (90, 60), (140, -40), (140, 60)])
def test_samples_at_the_very_corners_of_the_pick_area_are_collected(firmware, corner):
    rig = Rig(firmware, rock_xy=(float(corner[0]), float(corner[1])))
    rig.station.toggle_auto()
    # A sample exactly on the edge may read a hair outside; nudge it 1 mm in, as a person placing it would.
    rig.world.rock_xy = (corner[0] + (1 if corner[0] == 90 else -1), corner[1] + (1 if corner[1] == -40 else -1))
    assert rig.run(40, until=lambda: rig.station.collected == 1)
    assert rig.world.rock_state == "bin"


# --- things going wrong ---

def test_a_pick_that_did_not_work_is_noticed_and_autonomy_gives_up_after_three(rig):
    rig.world.slippery = True                # the jaws close but the sample stays put
    rig.station.toggle_auto()
    assert rig.run(90, until=lambda: not rig.station.auto)
    assert rig.station.misses == c.MAX_MISSES and rig.station.collected == 0
    assert len(rig.lines("# pick at")) == c.MAX_MISSES and logged(rig, "autonomy off")
    rig.run(5)
    assert len(rig.lines("# pick at")) == c.MAX_MISSES      # and it stays off


def test_a_knocked_camera_is_noticed_and_recalibrated_before_picking(rig):
    rig.run(1)
    before = rig.station.calibration
    rig.world.shift = (9.0, -6.0)            # someone bumps the camera: everything moves ~6 mm in the picture
    rig.run(c.CHECK_EVERY_S + 1)
    assert rig.station.calibration is not before and logged(rig, "recalibrated") and rig.station.cal_ok
    rig.station.toggle_auto()
    assert rig.run(30, until=lambda: rig.station.collected == 1)
    assert rig.world.rock_state == "bin"


def test_without_the_marker_check_a_knocked_camera_makes_the_arm_miss(firmware):
    """The same bump with the safety net off: shows the check above is what saves the pick."""
    rig = Rig(firmware)
    rig.station.watch_markers = False
    rig.run(1)
    rig.world.shift = (24.0, -16.0)
    rig.station.toggle_auto()
    assert rig.run(90, until=lambda: not rig.station.auto)
    assert rig.station.collected == 0 and rig.station.misses == c.MAX_MISSES


def test_no_picks_while_the_calibration_is_known_to_be_wrong(rig, monkeypatch):
    rig.run(1)
    monkeypatch.setattr(Calibration, "from_frame", classmethod(lambda cls, frame: None))   # recalibration fails
    rig.world.shift = (9.0, -6.0)
    rig.station.toggle_auto()
    rig.run(c.CHECK_EVERY_S + 4)
    assert not rig.station.cal_ok and not rig.lines("# pick at") and logged(rig, "CAMERA MOVED")
    monkeypatch.undo()
    assert rig.run(30, until=lambda: rig.station.collected == 1)     # recalibration works again: carries on


def test_no_picks_while_the_camera_feed_is_down(rig):
    rig.station.toggle_auto()
    rig.camera_on = False
    rig.run(4)
    assert not rig.station.camera_ok and not rig.lines("# pick at")
    rig.camera_on = True
    assert rig.run(30, until=lambda: rig.station.collected == 1)


def test_camera_dying_mid_pick_does_not_count_the_pick_until_it_is_seen(rig):
    rig.station.toggle_auto()
    assert rig.run(30, until=lambda: rig.firmware.phase == "LIFT")
    rig.camera_on = False
    rig.run(8)                               # the arm finishes on its own
    assert rig.world.rock_state == "bin" and rig.station.collected == 0 and logged(rig, "CAMERA LOST")
    rig.camera_on = True
    assert rig.run(5, until=lambda: rig.station.collected == 1)


def test_no_picks_while_the_arm_is_silent(rig):
    rig.run(1)
    real_read = rig.station.link.read_lines
    rig.station.link.read_lines = lambda: (real_read(), [])[1]       # the arm's replies stop arriving
    rig.run(c.LINK_SILENCE_S + 0.5)
    assert not rig.station.link_ok and logged(rig, "ARM SILENT")
    rig.station.toggle_auto()
    rig.run(3)
    assert not rig.station.pending and not rig.lines("# pick at")
    rig.station.link.read_lines = real_read
    assert rig.run(30, until=lambda: rig.station.collected == 1)


def test_a_command_that_cannot_be_sent_is_not_treated_as_sent(rig):
    rig.run(2)
    rig.station.link.write = lambda text: False
    assert not rig.station.pick()
    assert rig.station.pending is None and logged(rig, "NOT SENT")


def test_an_arm_that_never_answers_is_given_up_on(rig):
    rig.station.link.write = lambda text: True          # commands vanish on the way
    rig.station.toggle_auto()
    assert rig.run(c.PICK_TIMEOUT_S + 10, until=lambda: logged(rig, "DID NOT ANSWER"))
    assert rig.station.pending is None and not rig.station.auto


def test_arm_restarting_mid_pick_clears_the_pick_and_restores_the_mars_delay(rig):
    rig.station.toggle_mars()
    rig.station.toggle_auto()
    assert rig.run(30, until=lambda: rig.station.pending)
    settings_sent = len(rig.lines("OK comms delay ms=3000"))
    real_read = rig.station.link.read_lines
    rig.station.link.read_lines = lambda: [READY + " (power glitch)"]
    rig.run(0.04)
    rig.station.link.read_lines = real_read
    rig.run(c.MARS_DELAY_S + 1)
    assert rig.station.pending is None and not rig.station.auto and logged(rig, "ARM RESTARTED")
    assert len(rig.lines("OK comms delay ms=3000")) == settings_sent + 1     # the delay setting was sent again


# --- the serial link, with a fake port ---

class FakePort:
    def __init__(self, lines=()):
        self.incoming, self.written, self.dead, self.closed = deque(lines), [], False, False

    def readline(self):
        if self.dead:
            raise OSError("device removed")
        time.sleep(0.002)
        return self.incoming.popleft() if self.incoming else b""

    def write(self, data):
        if self.dead:
            raise OSError("device removed")
        self.written.append(data)

    def close(self):
        self.closed = True


def wait_for(condition, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.005)
    return False


def test_serial_link_survives_the_cable_being_pulled():
    banner = (READY + "\r\n").encode()
    ports = [FakePort([banner, b"T,90.0,90.0,70.0,60.0,IDLE\r\n"]), FakePort([banner])]
    opened = iter(ports)
    link = SerialLink("COMX", ready_timeout=2.0, opener=lambda: next(opened), retry_s=0.01)
    seen = []
    assert wait_for(lambda: seen.extend(link.read_lines()) or len(seen) >= 2)
    assert seen[0].startswith(READY) and link.connected

    assert link.write("P 120 20") and ports[0].written == [b"P 120 20\n"]
    ports[0].dead = True                     # cable pulled
    assert wait_for(lambda: seen.extend(link.read_lines()) or "# link: port reopened" in seen)
    assert "# link: port lost" in seen and ports[0].closed
    assert wait_for(lambda: seen.extend(link.read_lines()) or sum(l.startswith(READY) for l in seen) == 2)
    assert link.connected and link.write("H") and ports[1].written == [b"H\n"]
    link.close()


def test_serial_link_reports_a_write_to_a_dead_port():
    port = FakePort([(READY + "\r\n").encode()])
    link = SerialLink("COMX", ready_timeout=2.0, opener=lambda: port, retry_s=10)
    port.dead = True
    assert wait_for(lambda: not link.connected)
    assert link.write("H") is False
    link.close()


def test_serial_link_fails_straight_away_on_a_port_that_does_not_exist():
    def no_such_port():
        raise OSError("could not open port 'COM99'")
    with pytest.raises(OSError):
        SerialLink("COM99", opener=no_such_port)
