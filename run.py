"""Mars arm ground station.

    python run.py                            simulator: no arm or camera needed
    python run.py --conditions "dim room"    simulator with a camera problem (python robustness.py lists them)
    python run.py --real-firmware            simulator running the actual Arduino sketch (needs g++, or WSL on Windows)
    python run.py --port COM5                real arm on COM5, webcam 0
    python run.py --port COM5 --camera 1
    python run.py --headless 20 --auto --mars --record demo.mp4     make a demo clip without opening a window

Keys: SPACE pick the locked target, A autonomy on/off, M Mars comms delay,
X abort, H home, T calibrate the arm's servos with the camera, Q quit.
Simulator only: R new rock, C next camera problem, K knock the camera,
E give the servos the errors real ones have.
"""
import argparse
import os
import sys
import time

import cv2

from groundstation.dashboard import draw
from groundstation.simulator import PRESETS, FirmwareSim, SimWorld
from groundstation.station import GroundStation, SerialLink, SimLink
from groundstation.vision import Calibration

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
SERVO_ERRORS = ((6.0, -7.0, 5.0), (0.04, -0.03, 0.05))     # typical: horns a tooth out, travel a few percent off
FPS = 30


class Webcam:
    def __init__(self, index):
        self._camera = cv2.VideoCapture(index, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self._camera.isOpened():
            sys.exit(f"Could not open camera {index}.")

    def read(self):
        """The newest frame, or None if the camera gave nothing."""
        ok, frame = self._camera.read()
        return frame if ok else None


class SimCamera:
    """The simulated webcam, with the same read() as the real one."""

    def __init__(self, world):
        self.world = world

    def read(self):
        return self.world.render()


def calibrate_from_markers(camera, frames=15):
    """Calibration from the mat's markers as the camera sees them right now, or None."""
    shots = [frame for frame in (camera.read() for _ in range(frames)) if frame is not None]
    return Calibration.from_frames(shots) if shots else None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port of the Arduino, e.g. COM5; omit to run the simulator")
    parser.add_argument("--camera", type=int, default=0, help="webcam index (real arm only)")
    parser.add_argument("--conditions", default="clean", choices=list(PRESETS), metavar="NAME",
                        help="simulator only: camera problem to simulate, e.g. \"dim room\"")
    parser.add_argument("--real-firmware", action="store_true",
                        help="simulator only: run the real sketch, compiled for this PC, instead of the Python imitation")
    parser.add_argument("--headless", type=float, metavar="SECONDS",
                        help="simulator only: run this long with no window, as fast as possible")
    parser.add_argument("--servo-errors", action="store_true",
                        help="simulator only: servos assembled a few degrees out, as real ones are")
    parser.add_argument("--trim", action="store_true", help="calibrate the arm's servos with the camera at start-up")
    parser.add_argument("--auto", action="store_true", help="start with autonomy on")
    parser.add_argument("--mars", action="store_true", help="start with the Mars comms delay on")
    parser.add_argument("--save", metavar="PNG", help="write the last dashboard frame to this file on exit")
    parser.add_argument("--record", metavar="MP4", help="record the dashboard to this video file")
    args = parser.parse_args()

    firmware = world = None
    if args.port:
        camera = Webcam(args.camera)
        calibration, source = calibrate_from_markers(camera), "markers"
        if calibration is None and os.path.exists(CALIBRATION_FILE):
            calibration, source = Calibration.load(CALIBRATION_FILE), "calibration.json"
            print("Could not see the mat's markers; using the saved calibration.json instead.")
        if calibration is None:
            sys.exit("Could not see at least three of the mat's markers, and there is no calibration.json.\n"
                     "Run  python calibrate.py --camera %d  to see what the camera sees." % args.camera)
        try:
            link = SerialLink(args.port)
        except Exception as problem:
            sys.exit(f"Could not open {args.port}: {problem}")
    else:
        if args.real_firmware:
            from groundstation.hostfw import HostFirmware
            try:
                firmware = HostFirmware()
            except RuntimeError as problem:
                sys.exit(str(problem))
        else:
            firmware = FirmwareSim()
        world = SimWorld(firmware, conditions=PRESETS[args.conditions])
        if args.servo_errors:
            world.servo_offset, world.servo_gain = SERVO_ERRORS
        camera = SimCamera(world)
        # Like a real setup: calibrate in good light, then live with whatever the room does.
        calibration, source = calibrate_from_markers(camera) or calibrate_from_markers(SimCamera(SimWorld(FirmwareSim()))), "markers"
        link = SimLink(firmware)
        if args.real_firmware:
            link.name = "SIM, REAL SKETCH"

    station = GroundStation(link, calibration)
    station.cal_status = f"from {source}"
    if args.auto:
        station.toggle_auto()
    if args.mars:
        station.toggle_mars()

    trim_wanted, auto_after_trim = args.trim, args.trim and args.auto
    if auto_after_trim:
        station.auto = False                    # picks start once the arm has been calibrated
    headless = args.headless is not None and firmware is not None
    video = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (1280, 720)) if args.record else None
    conditions = list(PRESETS)
    start = time.monotonic()
    now, image = 0.0, None
    while True:
        previous, now = now, (now + 1 / FPS if headless else time.monotonic() - start)
        if firmware:
            firmware.advance(now - previous)
            world.update()
        station.update(camera.read(), now)
        if trim_wanted and station.calibrate_arm():
            trim_wanted = False
        if auto_after_trim and not trim_wanted and not station.routine:
            auto_after_trim, station.auto = False, station.arm_fit is not None
        image = draw(station)
        if video:
            video.write(image)
        if headless:
            if now >= args.headless:
                break
            continue

        cv2.imshow("Mars arm ground station", image)
        key = cv2.waitKey(1 if args.port else max(1, int(1000 / FPS))) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            station.pick()
        elif key == ord("a"):
            station.toggle_auto()
        elif key == ord("m"):
            station.toggle_mars()
        elif key == ord("x"):
            station.abort()
        elif key == ord("h"):
            station.home()
        elif key == ord("t"):
            station.calibrate_arm()
        elif world and key == ord("e"):
            world.servo_offset, world.servo_gain = SERVO_ERRORS
            station.log.append((now, "sim: servos now have real-world errors"))
        elif world and key == ord("r"):
            world.respawn_rock()
        elif world and key == ord("c"):
            name = conditions[(conditions.index(args.conditions) + 1) % len(conditions)]
            args.conditions, world.conditions = name, PRESETS[name]
            station.log.append((now, f"sim camera: {name}"))
        elif world and key == ord("k"):
            world.shift = (world.shift[0] + 8.0, world.shift[1] - 5.0)
            station.log.append((now, "sim: camera knocked"))

    if video:
        video.release()
    if args.save and image is not None:
        cv2.imwrite(args.save, image)
    for _, text in station.log:
        print(text)
    link.close()


if __name__ == "__main__":
    main()
