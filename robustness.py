"""Prints how well the vision copes with each simulated camera problem.

    python robustness.py               every preset, 9 rock positions each
    python robustness.py --save DIR    also writes one example frame per preset

For each preset it reports how often the rock was found and how far off the
position was, in mm. The gripper opens 28 mm and the rock is about 26 mm, so
an error above about 4 mm means a missed pick.
"""
import argparse
import math
import os

import cv2
import numpy as np

from groundstation import config as c
from groundstation.simulator import FirmwareSim, PRESETS, SimWorld
from groundstation.vision import Calibration, find_rock, rock_position

POSITIONS = [(x, y) for x in (95, 115, 135) for y in (-35, 10, 55)]


def measure(conditions):
    """(found, total, mean error mm, worst error mm, false alarm) with the camera calibrating itself
    from the mat's markers under the same conditions."""
    calibration = Calibration.from_frame(SimWorld(FirmwareSim(), conditions=conditions).render())
    if calibration is None:     # too few markers readable in these conditions: fall back to a clean-view calibration
        calibration = Calibration.from_frame(SimWorld(FirmwareSim()).render())
    errors, found = [], 0
    for position in POSITIONS:
        world = SimWorld(FirmwareSim(), position, conditions=conditions)
        frame = world.render()
        hit = find_rock(frame, calibration.pick_mask(frame.shape), calibration)
        if hit:
            found += 1
            errors.append(math.dist(rock_position(calibration, hit), position))
    empty = SimWorld(FirmwareSim(), conditions=conditions)      # nothing to pick: must not see anything
    empty.rock_state, empty.rock_xy = "bin", c.BIN_XY
    frame = empty.render()
    false_alarm = find_rock(frame, calibration.pick_mask(frame.shape), calibration) is not None
    return found, len(POSITIONS), (float(np.mean(errors)) if errors else float("nan")), \
        (max(errors) if errors else float("nan")), false_alarm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", help="folder for one example frame per preset")
    args = parser.parse_args()
    if args.save:
        os.makedirs(args.save, exist_ok=True)
    print(f"{'condition':18} {'found':>7} {'mean mm':>8} {'worst mm':>9}  empty table")
    for name, conditions in PRESETS.items():
        found, total, mean, worst, false_alarm = measure(conditions)
        flag = "" if found == total and worst < 4 and not false_alarm else "   <-- problem"
        print(f"{name:18} {found:>3}/{total:<3} {mean:8.2f} {worst:9.2f}  {'FALSE ALARM' if false_alarm else 'ok'}{flag}")
        if args.save:
            frame = SimWorld(FirmwareSim(), conditions=conditions).render()
            cv2.imwrite(os.path.join(args.save, name.replace(" ", "_") + ".png"), frame)


if __name__ == "__main__":
    main()
