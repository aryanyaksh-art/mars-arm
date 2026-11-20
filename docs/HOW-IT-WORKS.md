# How it works

The details behind the console, the vision and the failure handling.

## The console

It is drawn to look like a painted-steel flight console rather than a web
dashboard: engraved DIN lettering, dark readout windows, and colour used only
to mean something (green good, amber in progress, red fault).

- **Status lamps**: LINK, VIDEO and CALIB are the three things that must be
  green before the station will send a pick. AUTO, LOCK and DELAY show the mode.
- **Pick sequence**: the seven steps the firmware goes through, lit as it reaches them.
- **Joints**: each servo's angle on a 0 to 180 degree scale.
- **Light-time strip** (bottom): with the Mars delay on, commands are shown
  travelling up to the arm and video frames and telemetry travelling back down,
  placed by how far through the 3 seconds they are.
- The camera picture carries a millimetre scale around the pick area.

It uses the Bahnschrift and Consolas fonts that come with Windows, and falls
back to OpenCV's built-in font elsewhere.

## How it sees

- **Finding the table.** The work mat carries six square ArUco markers at known
  positions. The station finds them in the picture and works out the mapping
  from pixels to table millimetres, so there is nothing to click. It also
  estimates where the camera is.
- **Finding the sample.** It looks only inside the pick area, balances the
  colours so the white mat reads as neutral (this removes warm-bulb and
  daylight casts), and takes pixels in the sample's orange range. A blob only
  counts as the sample if it is the right size on the table (12 to 40 mm),
  roughly round and solid. A shiny highlight that bites a notch out of the
  orange is filled back in.
- **Height correction.** The sample stands up off the table, so from an
  off-centre camera it appears shifted away from the camera, by more than a
  millimetre at the edge of the pick area. The station corrects for this using
  the camera position.
- A target counts as locked after 12 steady frames.

## Calibrating the arm

No hobby servo goes exactly where it is told. Its horn only fits on the shaft
in steps of about 17 degrees, so each joint is assembled a few degrees out, and
its travel is a few percent more or less than 180 degrees. Left alone that puts
the jaws 10 to 20 mm from where the arm thinks they are, which is a miss.

Press **T** and the station measures it:

1. It moves the arm through 27 poses with the forearm held up and level, and
   records where the camera sees the small tag stuck on the forearm.
2. It finds the offset and gain of each of the three joints that best explain
   all 27 sightings. It solves for the camera's exact position at the same
   time, because where something 10 cm up in the air appears in the picture
   depends strongly on that, and the flat markers on the mat only give it roughly.
3. It sends the result to the arm (`Z` and `K` commands), which stores it in
   EEPROM, so it survives power-off and never needs doing again unless a horn is refitted.
4. It checks the result on three poses it was not fitted to and reports how
   far off the arm still is. Over 3 mm and it says the calibration is poor.

It takes about a minute. In the simulator, with servos 5 to 7 degrees and 3 to
5 percent out, the arm misses every pick before calibration and afterwards
puts the jaws within about 1 mm on average (2.5 mm at worst over 40 random
sets of errors). If it cannot see the tag, or the sightings do not fit an arm,
it gives up and puts the old trim back. X cancels it. It will not run with the
Mars delay on.

What it does not model: slack in the gears and the arm sagging under its own
weight, both of which a real arm has.

Try it without hardware: `python run.py --servo-errors`, press A and watch it
miss, then press T, then A again.

## What it does when things go wrong

The station does not trust anything it cannot check.

| Problem | What happens |
| --- | --- |
| The arm says "done" but the sample is still there | Logged as a miss. After 3 misses in a row autonomy switches itself off |
| The camera or mat gets bumped | The markers are re-read every 2 s and before every pick. If they have moved more than 1.5 mm it recalibrates; if it cannot, picks stop |
| The camera stops sending frames | Picks stop. A pick already under way is only counted once the camera is back and shows the sample gone |
| The arm stops answering | The firmware sends telemetry at least once a second. After 2.5 s of silence picks stop |
| The Arduino restarts mid-pick | The pick is dropped, autonomy goes off, and the Mars delay setting is sent again |
| The USB cable is pulled | The station keeps trying to reopen the port and says so in the log |
| A pick is never answered | Given up after 20 s (plus the comms delay), autonomy off |
