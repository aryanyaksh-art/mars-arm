# Building the real arm

Nothing here is needed to run the project: everything works in simulation. This is for when you want to build it. Nothing below has been built yet, so treat the fit and strength notes as design intent, not as tested.

The parts, prices and links are in [PARTS.md](PARTS.md). The print files are in [`cad/stl/`](../cad/stl).

## The arm

This is a custom direct-drive arm sized for SG90 / MG90S micro servos, not a
downloaded design. (For reference: EEZYbotARM MK1 is a free printable arm that
also uses micro servos; MK2 is its bigger version that needs three large
servos. This one is smaller and has fewer parts than either.)

- Reach: about 165 mm. Pick area: x 90–140 mm, y −40–60 mm in front of the base.
- Links: 80 mm upper arm, 85 mm from elbow to the middle of the jaws.
- Gripper: one moving finger closing against the end of the forearm, holds up to 28 mm.

Each link is glued onto the horn of the servo that drives it and carries the
next servo, so there are no bearings, pins or bolts.

## Printing

Print `fit_test.stl` first. It takes about ten minutes and checks the three
things that depend on your exact servos: the body pocket, the screw holes and
the horn recess. If a servo is loose or will not go in, change the numbers at
the top of `cad/build_parts.py` and re-run it.

| File | Part | Print notes |
| --- | --- | --- |
| `fit_test.stl` | Test coupon | Flat |
| `arm_base.stl` | Drum holding the base servo, with a flange to screw or clamp down | Flange down |
| `arm_turret.stl` | Sits on the base servo's horn, cradles the shoulder servo | Disc down |
| `arm_upper.stl` | Upper arm, holds the elbow servo | Flat, horn recess up |
| `arm_forearm.stl` | Forearm, fixed jaw and the wall holding the gripper servo | Flat side down, wall up |
| `arm_finger.stl` | Moving jaw | Flat, horn recess up |
| `mast_foot.stl` | Holds the dowel upright | Flat side down |
| `camera_boom.stl` | Caps the dowel; webcam clips onto the end plate | Exported upside down |
| `sample_bin.stl` | Drop container | |
| `rock_sample.stl` | The sample; print in orange | Flat side down |

None need supports. To regenerate: `blender-launcher -b --python cad/build_parts.py`
(the Store build of Blender hides its console; the report goes to `cad/build.log`).

## Assembly

1. Drop the base servo into the base, shaft toward the front (the +x side), and screw its tabs down.
2. Before attaching anything, plug each servo in and send `H` so every servo sits at its home angle.
3. Press a horn into each recess with a drop of superglue: double-arm horn under
   the turret, single-arm horns in the upper arm, forearm and finger.
4. Push the turret onto the base servo facing forward and fasten the horn screw through the centre hole.
5. Lay the shoulder servo in the turret cradle, shaft pointing left, and screw its tabs to the two blocks.
6. With the servos still at home: upper arm goes on pointing straight up; forearm
   goes on about 70° from the upper arm, pointing forward and slightly down.
7. Screw the gripper servo into the forearm wall, then fit the finger so the jaws are open.
8. Cut the small arm tag out of the printed mat along its dashed square and
   glue it on the flat top of the wall that holds the gripper servo: the 17 mm
   square goes at the end of that face nearest the forearm plate, its edges
   lined up with the wall's edges. With the forearm level it faces straight up.

Servo horns only fit in steps of about 17 degrees, so the home pose will be a
few degrees off. That is expected: [calibrating the arm](HOW-IT-WORKS.md#calibrating-the-arm)
measures and corrects it. If a joint moves the wrong way, flip its `DIR` at the
top of the sketch.

## Wiring

The firmware drives the servos straight from Uno pins, so no PCA9685 board is
needed. Servo power must come from the 5 V adapter, never from the Uno's 5V pin
(Wokwi wires it that way only because simulated servos draw nothing).

- Servo red wires → adapter +5 V. Servo brown wires → adapter GND.
- Adapter GND → Uno GND.
- Servo orange wires → Uno pins 3 (base), 5 (shoulder), 6 (elbow), 9 (gripper).
- Set `SIM_POTS` to `0` in the sketch before uploading.

## Using it with the real arm

1. `python make_mat.py`, then print `work_mat.svg` on Letter paper at 100% scale
   (the sheet has a 100 mm line to check). Tape it down flat, base drum on the circle.
2. `python calibrate.py --camera 0` and aim the camera until it reports all six
   markers and a good fit, with the green box on the pick area. Nothing needs
   saving: `run.py` finds the markers itself each time it starts. (If the
   markers cannot be read, `python calibrate.py --manual` lets you click the
   four crosses instead, and ENTER saves that as a fallback.)
3. `python tune.py --camera 0`, put the sample in the pick area and click it.
   If the right-hand picture shows the sample solid white and nothing else,
   the colour settings are fine. Otherwise adjust the sliders and paste the
   two lines it prints into `groundstation/config.py`.
4. `python run.py --port COM5` (find the port in the Arduino IDE).
5. Press **T** once to calibrate the arm's servos. Keep hands clear: the arm
   moves through 27 poses on its own.

Print the arm in a colour other than orange.
