# Parts to buy

Everything below is on **Amazon.ca**, in one cart. Prices are in CAD before tax
and shipping, read from the live listings on 16 November 2026 (all in stock; the
power supply showed "only 5 left"). They move, so glance at the price at checkout.
Amazon.ca usually ships free over $35, which this cart clears; check at checkout.

## The cart

**Must have: $131.29**

| Part | Qty | Price | Listing |
| --- | --- | --- | --- |
| MG90S metal-gear micro servos, 4-pack | 1 | $24.99 | [Miuzei MG90S](https://www.amazon.ca/dp/B0CP98TZJ2) |
| Arduino Uno R3 compatible board with USB cable | 1 | $16.99 | [UNO R3 ATmega328P](https://www.amazon.ca/dp/B0CXQ4XGQJ) |
| 5 V 3 A power supply with screw terminal | 1 | $18.06 | [5V 3A 15W supply](https://www.amazon.ca/dp/B0FXBY513X) |
| 400-point breadboards, 3-pack | 1 | $9.99 | [ELEGOO breadboards](https://www.amazon.ca/dp/B01EV640I6) |
| Dupont jumper wires, 120 pieces | 1 | $12.99 | [ELEGOO jumper wires](https://www.amazon.ca/dp/B01EV70C78) |
| USB webcam, 1080p | 1 | $14.29 | [1080P webcam](https://www.amazon.ca/dp/B0D8QY5MT1) |
| PLA filament 1.75 mm, grey, 1 kg (the arm: **not** orange) | 1 | $23.99 | [ELEGOO PLA grey](https://www.amazon.ca/dp/B0BM4CPVXC) |
| Super glue, 12 x 3 g | 1 | $9.99 | [Super glue 12 x 3 g](https://www.amazon.ca/dp/B0C5C9VDNP) |

**Also needed, still on Amazon: $37.57**

| Part | Qty | Price | Listing | What it is for |
| --- | --- | --- | --- | --- |
| Orange acrylic paint pens | 1 | $9.99 | [Orange paint pens](https://www.amazon.ca/dp/B0DBHS7KF4) | Colours the sample. The camera finds it by its orange |
| Glue stick, 3-pack | 1 | $6.99 | [Elmer's glue sticks](https://www.amazon.ca/dp/B00GGBI5AO) | Sticks the printed arm tag onto the arm |
| M3 screw, nut and washer kit | 1 | $10.99 | [M3 screw kit](https://www.amazon.ca/dp/B0GPWY9LFY) | The camera boom's set screw, and screwing the base down |
| 1000 uF 16 V capacitor, 5-pack | 1 | $9.60 | [Capacitors](https://www.amazon.ca/dp/B07YN6DS58) | Across the servo 5 V rails (stripe to ground) so the supply does not dip when the servos start |

**Totals**

| | Cost |
| --- | --- |
| Must have | $131.29 |
| + paint pens, glue stick, M3 kit | $159.26 |
| + capacitors | $168.86 |

The camera mast needs a 1/2 inch wooden dowel at least 350 mm long (14 inches).
Amazon only sells them in bulk (the nearest is [30 dowels of 1/2" x 15" for $25.99](https://www.amazon.ca/dp/B0D5Q34VL6)),
so buy one at any hardware store for a few dollars instead.

## Trimming it down

- **No webcam:** −$14.29, if a phone running a webcam app will do. The printed boom is made for a webcam, so a phone needs its own holder.
- **Print the sample in orange instead of painting it:** −$9.99 for the pens, but an orange spool is another [$24.99](https://www.amazon.ca/dp/B0BS6LTHLM). Painting is cheaper.
- **Skip the M3 kit:** −$10.99. Any small screw does for the boom, and tape or a clamp holds the base.
- **Skip the capacitors:** −$9.60. It is a precaution, not a requirement.
- **Canadian shops** sell the servos singly (BC Robotics, $6.00 each) and the Uno clone for less (Universal Solder, C$7.49), but they charge shipping on small orders.
- **No printer:** a library or school makerspace can print the files in `cad/stl/`.
- **Power for free:** an old 5 V charger rated 2 A or more and a USB cable you are willing to cut: red is +5 V, black is ground.

## Not on the list

- A **printer** for the mat (`work_mat.svg`, US Letter at 100%) and the arm tag. A library or school printer does.
- **Scissors** for the tag.

## Printing

About 200 g of PLA for everything, which is a fifth of a 1 kg spool
([ELEGOO PLA 1 kg, $24.99](https://www.amazon.ca/dp/B0BS6LTHLM) on Amazon.ca).
That figure is the parts' volume from the STL files at roughly 45% of solid
weight (walls plus light infill); the two heaviest are the mast foot (about
68 g) and the base (41 g).

- The sample must be **orange**. Print it in orange, or print it in anything and paint it.
- Print the arm in a colour that is **not** orange.
- No printer: a library or school makerspace can print the files in `cad/stl/`,
  and PCBWay runs a student sponsorship that covers 3D printing.

## Does the servo have enough torque?

Worked out from the STL volumes, a 13.4 g servo, and the arm stretched out
level at its full 165 mm reach while holding the sample, which is the worst case:

| Joint | Load | MG90S stall torque (4.8 V) | Share of stall |
| --- | --- | --- | --- |
| Shoulder | 0.48 to 0.71 kg-cm | 1.8 kg-cm | 27% to 39% |
| Elbow | about 0.24 kg-cm | 1.8 kg-cm | 13% |

The range is light infill to solid prints. A servo is comfortable up to about
a third of its stall torque and struggles past half, so the shoulder is
workable but has little to spare: print the forearm and finger light (15 to
20% infill), and do not swap the shoulder for a plastic-gear SG90. This is a
calculation, not a measurement; nothing has been built yet.

## Power

Four micro servos draw well under 1 A while moving normally and about 0.7 A
each if stalled, so 3 A covers the worst case and 2 A is enough in practice.
