"""Vision and calibration under bad cameras.  Run with:  python -m pytest tests"""
import math

import numpy as np
import pytest

from groundstation import config as c
from groundstation.simulator import FirmwareSim, PRESETS, Conditions, SimWorld
from groundstation.vision import Calibration, find_markers, find_rock, rock_position, white_balance

POSITIONS = [(95, -35), (115, 10), (135, 55), (100, 50), (130, -30)]


def calibrated(conditions):
    """Calibrate from the markers in a frame taken under these conditions (clean view if unreadable)."""
    return (Calibration.from_frame(SimWorld(FirmwareSim(), conditions=conditions).render())
            or Calibration.from_frame(SimWorld(FirmwareSim()).render()))


@pytest.mark.parametrize("name", PRESETS)
def test_sample_found_within_1mm_in_every_condition(name):
    calibration = calibrated(PRESETS[name])
    for position in POSITIONS:
        frame = SimWorld(FirmwareSim(), position, conditions=PRESETS[name]).render()
        found = find_rock(frame, calibration.pick_mask(frame.shape), calibration)
        assert found is not None, f"{name}: sample at {position} not found"
        assert math.dist(rock_position(calibration, found), position) < 1.0


@pytest.mark.parametrize("name", PRESETS)
def test_empty_table_gives_no_detection_in_every_condition(name):
    calibration = calibrated(PRESETS[name])
    world = SimWorld(FirmwareSim(), conditions=PRESETS[name])
    world.rock_state, world.rock_xy = "bin", c.BIN_XY
    frame = world.render()
    assert find_rock(frame, calibration.pick_mask(frame.shape), calibration) is None


def test_size_filter_rejects_things_that_are_not_the_sample():
    calibration = calibrated(Conditions())
    for radius in (4, 25):        # a small speck and a big sticker, in place of the sample
        world = SimWorld(FirmwareSim(), (120, 20), conditions=Conditions(decoys=((120, 20, radius, (30, 120, 255)),)))
        world.rock_state, world.rock_xy = "bin", c.BIN_XY
        frame = world.render()
        assert find_rock(frame, calibration.pick_mask(frame.shape), calibration) is None


def test_white_balance_removes_a_warm_cast_from_the_mat():
    world = SimWorld(FirmwareSim(), conditions=PRESETS["warm bulb"])
    frame = world.render()
    mask = Calibration.from_frame(frame).pick_mask(frame.shape)
    before = frame[mask > 0].mean(axis=0)
    after = white_balance(frame, mask)[mask > 0].mean(axis=0)
    assert before.max() - before.min() > 40          # the cast is real in the raw frame
    assert after.max() - after.min() < 15 + 0.15 * after.max() / 10


# --- calibration ---

def test_markers_are_all_found_in_a_clean_view():
    assert sorted(find_markers(SimWorld(FirmwareSim()).render())) == list(range(len(c.CAL_MARKERS_MM)))


@pytest.mark.parametrize("distortion", [0.0, 0.12, -0.08])
def test_marker_calibration_is_sub_millimetre_across_the_pick_area(distortion):
    """Also through a lens with barrel or pincushion distortion, which the calibration does not model:
    the markers surround the pick area, so the error it causes there stays small."""
    world = SimWorld(FirmwareSim(), conditions=Conditions(distortion=distortion))
    calibration = Calibration.from_frame(world.render())
    errors = [math.dist(calibration.table(*world.pixel(x, y)), (x, y))
              for x in range(90, 141, 10) for y in range(-40, 61, 20)]
    assert max(errors) < 0.8


def test_calibration_finds_roughly_where_the_camera_is():
    world = SimWorld(FirmwareSim())
    x, y, z = Calibration.from_frame(world.render()).camera_mm
    assert math.dist((x, y), world.CAMERA_AT[:2]) < 30      # limited by not knowing the lens's field of view
    assert z == pytest.approx(world.CAMERA_AT[2], rel=0.15)


def test_height_correction_removes_the_parallax_error():
    """Seen from the side, the sample appears shifted away from the camera. Without the correction the
    error is over a millimetre at the far edge of the pick area; with it, under half that."""
    world = SimWorld(FirmwareSim(), (95.0, 58.0))
    calibration = Calibration.from_frame(SimWorld(FirmwareSim()).render())
    frame = world.render()
    found = find_rock(frame, calibration.pick_mask(frame.shape), calibration)
    uncorrected = math.dist(calibration.table(found.u, found.v), world.rock_xy)
    corrected = math.dist(rock_position(calibration, found), world.rock_xy)
    assert uncorrected > 1.0 and corrected < 0.5


def test_pixel_and_table_are_inverses_at_any_height():
    calibration = Calibration.from_frame(SimWorld(FirmwareSim()).render())
    for height in (0.0, 8.0, 60.0):
        assert calibration.table(*calibration.pixel(120.0, 35.0, height), height) == pytest.approx((120.0, 35.0), abs=1e-6)


def test_calibrating_from_several_frames_beats_sensor_noise():
    world = SimWorld(FirmwareSim(), conditions=PRESETS["noisy sensor"])
    truth = [(x, y) for x in range(90, 141, 10) for y in range(-40, 61, 20)]
    worst = lambda cal: max(math.dist(cal.table(*world.pixel(x, y)), (x, y)) for x, y in truth)
    single = max(worst(Calibration.from_frame(world.render())) for _ in range(8))
    assert worst(Calibration.from_frames([world.render() for _ in range(15)])) < single


def test_calibration_round_trips_through_a_file(tmp_path):
    calibration = Calibration.from_frame(SimWorld(FirmwareSim()).render())
    path = tmp_path / "calibration.json"
    calibration.save(path)
    loaded = Calibration.load(path)
    assert loaded.table(300, 200, 8.0) == pytest.approx(calibration.table(300, 200, 8.0))
    assert loaded.camera_mm == pytest.approx(calibration.camera_mm)


def test_hand_calibration_from_four_clicked_crosses_agrees_with_the_markers():
    world = SimWorld(FirmwareSim())
    by_hand, by_markers = Calibration(world.calibration_pixels()), Calibration.from_frame(world.render())
    for x, y in ((90, -40), (140, 60), (115, 10)):
        assert math.dist(by_hand.table(*world.pixel(x, y)), by_markers.table(*world.pixel(x, y))) < 0.5


def test_too_few_markers_gives_no_calibration():
    assert Calibration.from_frame(np.full((480, 640, 3), 200, np.uint8)) is None


def test_check_notices_a_camera_that_has_been_knocked():
    world = SimWorld(FirmwareSim())
    calibration = Calibration.from_frame(world.render())
    seen, error = calibration.check(world.render())
    assert seen == 6 and error < 0.5
    world.shift = (6.0, -4.0)                       # camera nudged
    _, error = calibration.check(world.render())
    assert error > 2.0
