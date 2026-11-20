"""Numbers shared with the firmware (mars_arm/mars_arm.ino) and the printed parts (cad/build_parts.py).

If you change one of these in the sketch, change it here too. Lengths are mm,
angles are degrees, table frame is x forward, y left, z up from the base axis.
"""

# Arm geometry.
L1, L2 = 80.0, 85.0          # shoulder-to-elbow, elbow-to-jaws
BASE_H = 46.2                # table to shoulder axis
GRIP_Y = -16.5               # sideways offset of the jaws from the arm's plane
UPPER_LIMITS = (-10.0, 120.0)
INNER_LIMITS = (45.0, 150.0)

# Poses and heights used by the pick sequence.
HOME = (90.0, 90.0, 70.0)    # base, shoulder, elbow servo angles
GRIP_OPEN, GRIP_CLOSED = 60.0, 100.0
Z_HOVER, Z_GRIP, Z_CARRY = 45.0, 14.0, 75.0

# Workcell layout.
BIN_XY = (50.0, -140.0)
BIN_RADIUS = 45.0
PICK_X = (90.0, 140.0)       # the arm only picks inside this rectangle
PICK_Y = (-40.0, 60.0)
# The printed work mat (make_mat.py): a US Letter sheet, landscape, long side along table y.
MAT_PAGE_MM = (279.4, 215.9)
MAT_TOP_X, MAT_LEFT_Y = 180.9, 90.0     # table coordinates of the sheet's top-left corner
# ArUco markers on the mat, centred at these table positions (marker id = list index).
# Six of them, three along each side of the pick area, so the camera can find the table on its own.
CAL_MARKERS_MM = [(155.0, 72.0), (155.0, -80.0), (75.0, -80.0), (75.0, 72.0), (115.0, 72.0), (115.0, -80.0)]
MARKER_SIZE_MM = 20.0
# Crosses for calibrating by hand (calibrate.py --manual), in the order you click them.
CAL_POINTS_MM = [(155.0, 48.0), (155.0, -56.0), (75.0, -56.0), (75.0, 48.0)]
CAL_LABELS = "ABCD"

# The tag stuck on the arm, which the camera watches to calibrate the servos (make_mat.py prints it).
# It goes on the flat top of the wall that holds the gripper servo, pushed up against the forearm
# plate's side of that wall. Position of its centre, from cad/build_parts.py:
ARM_TAG_ID = 6
ARM_TAG_SIZE_MM = 13.0       # the black square; the paper patch around it is 17 mm
ARM_TAG_PATCH_MM = 17.0
TAG_ALONG_MM = 50.0          # from the elbow axis along the forearm (PIVOT_U)
TAG_ABOVE_MM = 20.3          # above the forearm's axis (PALM_W0 + LINK_T)
TAG_SIDE_MM = -6.0           # sideways, same sign as GRIP_Y

# Vision: HSV range for the orange sample (OpenCV hue runs 0-179). It is applied after
# the colours are balanced so the white mat reads as neutral, so a warm bulb or daylight
# does not shift it. The low brightness limit lets it work in a dim room or under a shadow.
HSV_LO = (4, 120, 60)
HSV_HI = (24, 255, 255)
MIN_AREA_PX = 150
ROCK_CENTRE_HEIGHT_MM = 8.0       # height of the sample's widest part: what its outline in the picture is centred on
CAMERA_HFOV_DEG = 65.0            # the webcam's horizontal field of view; most are 55 to 78
ROCK_DIAMETER_MM = (12.0, 40.0)   # blobs outside this size are not the sample (stickers, edges, glare specks)
ROCK_NOMINAL_MM = 26.0            # when several blobs fit, the one closest to this wins
MIN_CIRCULARITY = 0.55            # 1.0 is a circle; thin streaks and clipped slivers score low
MIN_SOLIDITY = 0.6                # orange area / area of its convex hull; a ring or a scatter of bits scores low
LOCK_FRAMES = 12             # consecutive steady detections before the target counts as locked
LOCK_TOLERANCE_MM = 4.0

# Station behaviour.
CHECK_EVERY_S = 2.0          # how often the station re-reads the mat markers to see if the camera moved
DRIFT_TOLERANCE_MM = 1.5     # recalibrate when the markers say the calibration is off by more than this
PICK_TIMEOUT_S = 20.0        # a pick that has not finished after this (plus comms delay) is given up on
LINK_SILENCE_S = 2.5         # the firmware sends telemetry at least once a second; quieter than this means trouble
CAMERA_STALE_S = 1.0         # no new frame for this long means the camera has stopped
VERIFY_FRAMES = 15           # frames to look after a pick, to confirm the sample really left the pick area
MAX_MISSES = 3               # failed picks in a row before autonomy switches itself off
ARM_CHECK_MM = 3.0           # after calibrating the arm, it must put the tag within this of where it should be

MARS_DELAY_S = 3.0           # one-way signal delay in Mars mode
BAUD = 115200
