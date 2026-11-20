"""Works out how far off each servo is, from where the camera sees the tag on the forearm.

No hobby servo goes exactly where it is told. Its horn only fits on the shaft in
steps of about 17 degrees, so every joint is assembled a few degrees out, and its
travel is a few percent more or less than the 180 degrees it should be. A joint
that is told to go to `a` really goes to

    90 + (a - 90) * (1 + gain) + offset

Left alone, that puts the jaws 10 to 20 mm from where the arm thinks they are.

To measure it, the arm is moved through a set of poses with the forearm held up
where the camera can see the tag on it. For each pose we know the angles that
were commanded and the pixel where the tag appeared. `fit` finds the three
offsets and three gains that best explain all the sightings.

It solves for the camera's position at the same time. The tag is 10 cm up in the
air, and where something that high appears in the picture depends strongly on
exactly where the camera is, which the flat markers on the mat only pin down
roughly. The sightings of the tag at different heights pin it down properly.
"""
import math
from dataclasses import dataclass

import numpy as np

from . import config as c
from .kinematics import tag_position

# Poses to watch the tag in: (base, shoulder, elbow) servo angles. The forearm is level or
# nearly so in all of them (shoulder + elbow is close to 180), so the tag faces the camera,
# and all are high enough that even a badly assembled arm stays well clear of the table.
# With the arm built 9 degrees out at every joint, the lowest any of them brings the jaws is 35 mm.
POSES = [(base, shoulder, elbow) for base in (55, 90, 125)
         for shoulder, elbow in ((40, 140), (60, 120), (85, 95), (60, 95), (100, 100), (45, 115),
                                 (30, 150), (35, 130), (110, 75))]
CHECK_POSES = [(70, 50, 125), (110, 75, 105), (90, 55, 110)]    # different ones, to test the result on
MIN_POSES = 12              # refuse to fit from fewer sightings than this
MAX_OFFSET_DEG, MAX_GAIN = 20.0, 0.15       # what the firmware will accept; a fit beyond these is not believed


def real_angles(commanded, offsets, gains):
    """Where joints really go when told to go to `commanded`."""
    return [90 + (a - 90) * (1 + g) + o for a, o, g in zip(commanded, offsets, gains)]


def predict(calibration, pose, offsets=(0, 0, 0), gains=(0, 0, 0), camera=None):
    """Pixel where the tag should appear with the arm commanded to `pose`."""
    x, y, z = tag_position(*real_angles(pose, offsets, gains))
    return calibration.pixel(x, y, z, camera)


@dataclass
class ArmFit:
    offsets: tuple          # degrees: base, shoulder, elbow
    gains: tuple            # fractions
    camera_mm: tuple        # where the camera turned out to be: x, y, height
    rms_px: float           # how well the fit explains the sightings; about 0.3 is as good as the camera gets
    poses: int

    @property
    def believable(self):
        return (self.rms_px < 1.5 and max(map(abs, self.offsets)) <= MAX_OFFSET_DEG
                and max(map(abs, self.gains)) <= MAX_GAIN and 150 < self.camera_mm[2] < 1500)


def fit(calibration, poses, pixels):
    """Best offsets, gains and camera position for these sightings. None if there are too few."""
    seen = [(pose, pixel) for pose, pixel in zip(poses, pixels) if pixel is not None]
    if len(seen) < MIN_POSES:
        return None
    start = calibration.camera_mm or (np.mean([p[0] for p in c.CAL_MARKERS_MM]), 0.0, 350.0)
    p = np.concatenate([np.zeros(6), start])            # offsets, gains, camera
    limit = np.array([10.0] * 3 + [0.1] * 3 + [60.0] * 3)  # most any value may change in one step

    def residuals(p):
        return np.concatenate([np.subtract(predict(calibration, pose, p[0:3], p[3:6], tuple(p[6:9])), pixel)
                               for pose, pixel in seen])

    for _ in range(60):                                 # Gauss-Newton with a numerical Jacobian
        r = residuals(p)
        jacobian = np.empty((len(r), len(p)))
        for i in range(len(p)):
            nudge = np.zeros(len(p))
            nudge[i] = 1e-4
            jacobian[:, i] = (residuals(p + nudge) - r) / 1e-4
        step = np.clip(np.linalg.lstsq(jacobian, -r, rcond=None)[0], -limit, limit)
        p = p + step
        if np.abs(step / limit).max() < 1e-7:
            break
    r = residuals(p)
    return ArmFit(tuple(map(float, p[0:3])), tuple(map(float, p[3:6])), tuple(map(float, p[6:9])),
                  float(np.sqrt(np.mean(r ** 2))), len(seen))


def miss_mm(calibration, pose, pixel, camera=None):
    """How far (mm, at the tag's height) the tag was seen from where a perfectly trimmed arm would put it."""
    x, y, z = tag_position(*pose)
    return math.dist(calibration.table(*pixel, z, camera), (x, y))
