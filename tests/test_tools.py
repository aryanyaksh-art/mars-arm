"""The command-line tools, exercised without a window, camera or arm."""
import math
import re
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

import calibrate
import make_mat
import tune
from groundstation import config as c
from groundstation.dashboard import draw
from groundstation.simulator import FirmwareSim, PRESETS, SimWorld
from groundstation.station import GroundStation, SimLink
from groundstation.vision import Calibration, find_arm_tag, find_markers, find_rock
from run import SimCamera, calibrate_from_markers

ROOT = Path(__file__).resolve().parent.parent


def test_the_printed_mat_carries_markers_the_camera_can_read(tmp_path, monkeypatch):
    """Draw the SVG's marker squares as a picture of the sheet, then check the vision finds all six
    markers and puts the table where the mat says it is."""
    monkeypatch.chdir(tmp_path)
    make_mat.main()
    svg = (tmp_path / "work_mat.svg").read_text()
    scale = 4                                # pixels per mm
    width, height = c.MAT_PAGE_MM
    sheet = np.full((round(height * scale), round(width * scale), 3), 255, np.uint8)
    squares = re.findall(r'<rect x="([\d.]+)" y="([\d.]+)" width="([\d.]+)" height="([\d.]+)"/>',
                         svg[svg.index('<g fill="black"'):])
    assert squares
    for x, y, w, h in (map(float, square) for square in squares):
        cv2.rectangle(sheet, (round(x * scale), round(y * scale)),
                      (round((x + w) * scale) - 1, round((y + h) * scale) - 1), (0, 0, 0), -1)
    assert sorted(find_markers(sheet)) == list(range(len(c.CAL_MARKERS_MM)))
    assert find_arm_tag(sheet) is not None                   # the cut-out tag for the arm is on the sheet too
    calibration = Calibration.from_frame(sheet)
    for page_x, page_y in ((100, 100), (150, 60), (30, 180)):          # mm from the sheet's top-left corner
        expected = (c.MAT_TOP_X - page_y, c.MAT_LEFT_Y - page_x)
        assert math.dist(calibration.table(page_x * scale, page_y * scale), expected) < 0.3


def test_everything_on_the_mat_is_inside_the_printable_area():
    width, height = c.MAT_PAGE_MM
    margin = 6.0                             # most printers cannot print closer to the edge than this
    half = c.MARKER_SIZE_MM / 2
    points = [(x + dx, y + dy) for x, y in c.CAL_MARKERS_MM for dx in (-half, half) for dy in (-half, half)]
    points += [(x + dx, y + dy) for x, y in c.CAL_POINTS_MM for dx in (-4, 4) for dy in (-4, 4)]
    points += [(c.PICK_X[i], c.PICK_Y[j]) for i in (0, 1) for j in (0, 1)]
    half = c.ARM_TAG_PATCH_MM / 2
    points += [(make_mat.ARM_TAG_AT[0] + dx, make_mat.ARM_TAG_AT[1] + dy) for dx in (-half, half) for dy in (-half, half)]
    for x, y in points:
        page_x, page_y = make_mat.page(x, y)
        assert margin <= page_x <= width - margin and margin <= page_y <= height - margin, (x, y)


def test_markers_do_not_overlap_the_pick_area_or_each_other():
    half = c.MARKER_SIZE_MM / 2
    for i, (x, y) in enumerate(c.CAL_MARKERS_MM):
        inside_x = c.PICK_X[0] < x + half and x - half < c.PICK_X[1]
        inside_y = c.PICK_Y[0] < y + half and y - half < c.PICK_Y[1]
        assert not (inside_x and inside_y), f"marker {i} overlaps the pick area"
        for other in c.CAL_MARKERS_MM[i + 1:]:
            assert max(abs(x - other[0]), abs(y - other[1])) >= c.MARKER_SIZE_MM + 10      # quiet zone between them


def test_calibration_view_finds_the_table_on_the_simulated_camera():
    camera = SimCamera(SimWorld(FirmwareSim()))
    picture, calibration = calibrate.annotate([camera.read() for _ in range(10)])
    assert calibration is not None and calibration.rms_mm < c.DRIFT_TOLERANCE_MM
    assert picture.shape == (480, 640, 3)


def test_calibration_view_says_so_when_it_cannot_see_the_mat():
    picture, calibration = calibrate.annotate([np.full((480, 640, 3), 90, np.uint8)])
    assert calibration is None and picture.shape == (480, 640, 3)


@pytest.mark.parametrize("name", ["clean", "dim room", "warm bulb", "everything bad"])
def test_clicking_the_sample_in_the_tuning_tool_gives_settings_that_find_it(name, monkeypatch):
    world = SimWorld(FirmwareSim(), (110.0, 25.0), conditions=PRESETS[name])
    calibration = calibrate_from_markers(SimCamera(world)) or calibrate_from_markers(SimCamera(SimWorld(FirmwareSim())))
    frame = world.render()
    mask = calibration.pick_mask(frame.shape)
    u, v = map(int, world.pixel(*world.rock_xy, 8.0))
    low, high, saturation, brightness = tune.range_around(frame, mask, u, v)
    monkeypatch.setattr(c, "HSV_LO", (low, saturation, brightness))
    monkeypatch.setattr(c, "HSV_HI", (high, 255, 255))
    picture, found = tune.view(frame, calibration)
    assert found is not None and math.dist((found.u, found.v), (u, v)) < 4
    assert picture.shape == (480, 1280, 3)
    world.rock_state, world.rock_xy = "bin", c.BIN_XY                  # and nothing when the sample is gone
    empty = world.render()
    assert find_rock(empty, mask, calibration) is None


def test_ground_station_runs_a_whole_mission_from_the_command_line(tmp_path):
    shot = tmp_path / "dashboard.png"
    done = subprocess.run([sys.executable, "run.py", "--headless", "12", "--auto", "--conditions", "dim room",
                           "--save", str(shot)], cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert done.returncode == 0, done.stderr
    assert "OK P 120 20" in done.stdout
    assert cv2.imread(str(shot)).shape == (720, 1280, 3)


def test_dashboard_draws_in_every_state_the_station_can_be_in():
    firmware = FirmwareSim()
    world = SimWorld(firmware)
    station = GroundStation(SimLink(firmware), Calibration.from_frame(world.render()))
    pictures = [draw(station)]                                   # before the first frame
    station.toggle_mars()
    station.toggle_auto()
    now = 0.0
    for step in range(9 * 30):                                   # through a delayed pick
        now += 1 / 30
        firmware.advance(1 / 30)
        world.update()
        station.update(world.render(), now)
        if step % 45 == 0:
            pictures.append(draw(station))
    station.cal_ok, station.link_ok, station.misses = False, False, 2
    pictures.append(draw(station))                               # faults showing
    for _ in range(60):                                          # camera gone
        now += 1 / 30
        station.update(None, now)
    pictures.append(draw(station))
    assert all(p.shape == (720, 1280, 3) and p.dtype == np.uint8 for p in pictures)
    assert len({p.tobytes() for p in pictures}) == len(pictures)  # every state looks different


def test_ground_station_can_record_a_demo_clip(tmp_path):
    clip = tmp_path / "demo.mp4"
    done = subprocess.run([sys.executable, "run.py", "--headless", "2", "--auto", "--record", str(clip)],
                          cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert done.returncode == 0, done.stderr
    video = cv2.VideoCapture(str(clip))
    assert int(video.get(cv2.CAP_PROP_FRAME_COUNT)) >= 55 and int(video.get(cv2.CAP_PROP_FRAME_WIDTH)) == 1280


def test_robustness_report_shows_no_problems():
    done = subprocess.run([sys.executable, "robustness.py"], cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert done.returncode == 0, done.stderr
    assert done.stdout.count(" ok") == len(PRESETS) and "problem" not in done.stdout
