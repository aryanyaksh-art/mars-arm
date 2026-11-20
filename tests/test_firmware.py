"""The Python imitation of the firmware (FirmwareSim) must behave like the real sketch.

The real sketch is mars_arm/mars_arm.ino compiled for the PC (groundstation/hostfw.py).
These tests send the same commands to both and compare everything they print.
They are skipped if no C++ compiler is available.
"""
import random

import pytest

from conftest import real_firmware_or_skip
from groundstation.simulator import FirmwareSim

DT = 0.012

SCRIPTS = {
    "pick": [(0.1, "P 120 20")],
    "pick at each corner": [(0.1, "P 90 -40"), (5.0, "P 140 60"), (10.0, "P 90 60"), (15.0, "P 140 -40")],
    "every move command": [(0.1, "G 110 0 60"), (1.5, "J 45 80 100 90"), (3.5, "O"), (4.5, "C"), (5.5, "H"), (7.5, "?")],
    "bad commands": [(0.1, "P 300 0"), (0.3, "Z"), (0.5, "P 1"), (0.7, "G 1 2"), (0.9, "X"), (1.1, "J 1 2 3"),
                     (1.3, "G 200 0 10"), (1.5, "P abc def"), (1.7, "P nan nan"), (1.9, "J nan 90 90 90"),
                     (2.1, "G inf 0 0"), (2.3, "J 123.45 123.45 123.45 123.45 123"), (2.5, "P 120 20 junk"),
                     (7.5, "M abc"), (7.7, "O 5"), (8.7, "J 400 -50 90 60")],
    "abort mid-pick": [(0.1, "P 120 20"), (1.2, "X"), (3.0, "H")],
    "abort while gripping": [(0.1, "P 120 20"), (1.45, "X"), (3.0, "H")],
    "mars delay": [(0.1, "M 3000"), (0.2, "P 100 -30"), (4.0, "X"), (9.0, "M 0"), (9.1, "H")],
    "queue overflow": [(0.1, "M 2000"), (0.2, "H"), (0.25, "O"), (0.3, "C"), (0.35, "H"), (0.4, "O"), (0.45, "C")],
    "commands sent during a pick": [(0.1, "P 120 20"), (0.5, "H"), (0.5, "O"), (0.5, "C")],
    "idle heartbeat": [],
    "servo trim": [(0.1, "Z 5 -7.5 3"), (0.5, "K 4 -3 25"), (1.0, "J 60 60 100 60"), (3.0, "Z"), (3.5, "Z 1 2"),
                   (4.0, "K"), (4.5, "?"), (5.0, "P 120 20"), (12.0, "Z 30 -30 0.005"), (12.5, "K -99 1.005 2.675"),
                   (13.0, "H"), (15.0, "Z 0 0 0"), (15.5, "K 0 0 0")],
}


def trace(firmware, script, seconds):
    """Everything the firmware prints, with times, when sent this script, and where it drove its servos."""
    out, servos, t, waiting = [(0.0, line) for line in firmware.read_lines()], [], 0.0, sorted(script)
    while t < seconds:
        while waiting and waiting[0][0] <= t + 1e-9:
            firmware.write(waiting.pop(0)[1])
        firmware.advance(DT)
        t += DT
        out += [(t, line) for line in firmware.read_lines()]
        servos.append(list(firmware.servo))
    return out, servos


def compare(script, seconds=20.0, time_tolerance=0.0005, angle_tolerance=0.11):
    real = real_firmware_or_skip()
    (expected, real_servos), (got, servos) = trace(real, script, seconds), trace(FirmwareSim(), script, seconds)
    assert real.rx_dropped == 0
    real.close()
    # The servo outputs themselves, every 12 ms. The real sketch rounds to whole microseconds (0.1 degree).
    worst = max(abs(a - b) for want, have in zip(real_servos, servos) for a, b in zip(want, have))
    assert worst < 0.25, f"servo outputs differ by {worst:.2f} degrees"
    for kind, same_text in (("events", True), ("telemetry", False)):
        want = [(t, l) for t, l in expected if l.startswith("T,") != same_text]
        have = [(t, l) for t, l in got if l.startswith("T,") != same_text]
        if len(have) != len(want):
            first = next((i for i, (w, h) in enumerate(zip(want, have)) if w[1] != h[1]), min(len(want), len(have)))
            pytest.fail(f"{kind}: real firmware printed {len(want)} lines, imitation {len(have)}; "
                        f"first difference at line {first}: real {want[first:first + 2]}, imitation {have[first:first + 2]}")
        for (t_want, l_want), (t_have, l_have) in zip(want, have):
            assert abs(t_have - t_want) <= time_tolerance, f"{l_have!r} at {t_have:.3f} s, real firmware at {t_want:.3f} s"
            if same_text:
                assert l_have == l_want
            else:
                a, b = l_want.split(","), l_have.split(",")
                assert a[5] == b[5], f"phase at {t_want:.3f} s: real {l_want}, imitation {l_have}"
                assert all(abs(float(x) - float(y)) <= angle_tolerance for x, y in zip(a[1:5], b[1:5])), \
                    f"at {t_want:.3f} s: real {l_want}, imitation {l_have}"
    return expected


@pytest.mark.parametrize("name", SCRIPTS)
def test_imitation_prints_what_the_real_firmware_prints(name):
    compare(SCRIPTS[name])


def test_real_firmware_answers_with_the_command_it_was_given():
    """Regression: a command arriving mid-move used to overwrite the text of the one being run,
    so an aborted pick was reported as 'ERR X' and the ground station never heard about its pick."""
    lines = [l for _, l in compare(SCRIPTS["abort mid-pick"])]
    assert "ERR P 120 20" in lines and "ERR X" not in lines


def test_real_firmware_never_sends_nonsense_to_a_servo():
    real = real_firmware_or_skip()
    for command in ("P nan nan", "J nan nan nan nan", "G inf 0 0", "J 1e9 -1e9 90 60", "P 1e30 1e30"):
        real.write(command)
        for _ in range(250):
            real.advance(DT)
            assert all(-0.5 <= angle <= 180.5 for angle in real.servo), f"{command!r} drove a servo to {real.servo}"
    real.close()


COMMANDS = ["P {x} {y}", "P {x} {y}", "G {x} {y} {z}", "J {a} {b} {c} {d}", "O", "C", "H", "X", "X", "?",
            "M 0", "M 700", "M 2000", "Z", "P 1", "P nan 3", "G 1 2 x", "Y", "Z {o} {p} {o}", "K {p} {o} {p}", "K 1"]


@pytest.mark.parametrize("seed", range(40))
def test_imitation_matches_real_firmware_on_random_command_streams(seed):
    rng = random.Random(seed)
    script = []
    for _ in range(rng.randint(5, 10)):
        text = rng.choice(COMMANDS).format(
            x=rng.randint(60, 170), y=rng.randint(-90, 90), z=rng.randint(5, 110),
            a=rng.randint(-20, 200), b=rng.randint(0, 180), c=rng.randint(0, 180), d=rng.randint(40, 120),
            o=round(rng.uniform(-9, 9), 2), p=round(rng.uniform(-6, 6), 2))
        script.append((round(rng.uniform(0.1, 12.0), 2), text))
    compare(script, seconds=24.0)
