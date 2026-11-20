"""Write work_mat.svg: a US Letter sheet to print at 100% scale and tape under the arm.

It shows where the arm base, pick area and bin go, and carries the six square
markers the camera uses to find the table by itself. The four small crosses are
only for calibrating by hand (calibrate.py --manual). The small tag in the
corner is cut out and stuck on the arm, for calibrating its servos.

    python make_mat.py
"""
from groundstation import config as c
from groundstation.vision import marker_image

PAGE_W, PAGE_H = c.MAT_PAGE_MM      # mm, landscape: the long side runs left-right on the table (y)


def page(x, y):
    """Table mm -> page mm. Table +x is up the page, table +y is to the left."""
    return c.MAT_LEFT_Y - y, c.MAT_TOP_X - x


ARM_TAG_AT = (-20.0, -172.0)        # where the cut-out tag for the arm is printed, table mm


def marker_svg(marker_id, centre=None, size=c.MARKER_SIZE_MM):
    """One ArUco marker as black squares, upright on the page (its top toward table +x)."""
    cx, cy = centre or c.CAL_MARKERS_MM[marker_id]
    half = size / 2
    left, top = page(cx + half, cy + half)
    cells = marker_image(marker_id, 6)          # 6x6 cells: the 4x4 code plus its black border
    cell = size / 6
    out = []
    for row in range(6):
        col = 0
        while col < 6:                          # one rectangle per run of black cells
            if cells[row, col]:
                col += 1
                continue
            start = col
            while col < 6 and not cells[row, col]:
                col += 1
            # a hair of overlap so no white line shows between rows
            out.append(f'<rect x="{left + start * cell:.3f}" y="{top + row * cell:.3f}" '
                       f'width="{(col - start) * cell:.3f}" height="{cell + 0.02:.3f}"/>')
    return out


def main():
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{PAGE_W}mm" height="{PAGE_H}mm" '
           f'viewBox="0 0 {PAGE_W} {PAGE_H}" font-family="Arial" font-size="3.5">',
           '<g fill="none" stroke="black" stroke-width="0.3">']

    def text(x, y, s, dx=2.0, dy=-2.0, size=None):
        px, py = page(x, y)
        attr = f' font-size="{size}"' if size else ""
        out.append(f'<text x="{px + dx:.1f}" y="{py + dy:.1f}" fill="black" stroke="none"{attr}>{s}</text>')

    # Arm base: drum circle, centre cross and the front edge of the flange.
    bx, by = page(0, 0)
    out.append(f'<circle cx="{bx}" cy="{by}" r="30"/>')
    out.append(f'<path d="M{bx - 6} {by} h12 M{bx} {by - 6} v12"/>')
    fx, fy = page(45, 35)
    out.append(f'<path d="M{fx} {fy} h70" stroke-dasharray="2 1"/>')
    text(45, 35, "front edge of base flange", dy=-1.5)
    text(0, 0, "ARM BASE", dx=-9, dy=13)

    # Pick area.
    (x0, x1), (y0, y1) = c.PICK_X, c.PICK_Y
    px, py = page(x1, y1)
    out.append(f'<rect x="{px}" y="{py}" width="{y1 - y0}" height="{x1 - x0}" stroke-dasharray="3 1.5"/>')
    text(x0, y1, f"PICK AREA  x {x0:.0f} to {x1:.0f}  y {y0:.0f} to {y1:.0f} mm", dx=1, dy=4.5)

    # Bin.
    cx, cy = page(*c.BIN_XY)
    out.append(f'<circle cx="{cx}" cy="{cy}" r="32"/>')
    text(c.BIN_XY[0], c.BIN_XY[1], "SAMPLE BIN", dx=-11, dy=1)

    # Crosses for calibrating by hand.
    for label, (x, y) in zip(c.CAL_LABELS, c.CAL_POINTS_MM):
        px, py = page(x, y)
        out.append(f'<path d="M{px - 4} {py} h8 M{px} {py - 4} v8" stroke-width="0.4"/>')
        text(x, y, label, dx=1.5, dy=-1.5, size=3)

    # Scale check.
    sx, sy = page(-22, 60)
    out.append(f'<path d="M{sx} {sy} h100 M{sx} {sy - 2} v4 M{sx + 100} {sy - 2} v4"/>')
    text(-22, 60, "This line must measure 100 mm. If not, print again at 100% / actual size.", dy=-3)
    text(-22, 60, "Keep the six square markers uncovered and flat: the camera finds the table with them.", dy=7)
    # The tag that goes on the arm: cut along the dashed square.
    half = c.ARM_TAG_PATCH_MM / 2
    px, py = page(ARM_TAG_AT[0] + half, ARM_TAG_AT[1] + half)
    out.append(f'<rect x="{px:.2f}" y="{py:.2f}" width="{2 * half}" height="{2 * half}" stroke-dasharray="1.2 0.8" stroke-width="0.2"/>')
    text(ARM_TAG_AT[0] + half, ARM_TAG_AT[1] + half, "ARM TAG: cut out along the dashes, stick on the arm (see README)", dx=-104, dy=7, size=3)
    out.append("</g>")

    # Markers: solid black, no outline.
    out.append('<g fill="black" stroke="none" shape-rendering="crispEdges">')     # no hairlines between cells
    for marker_id, (x, y) in enumerate(c.CAL_MARKERS_MM):
        out += marker_svg(marker_id)
        text(x - c.MARKER_SIZE_MM / 2, y + c.MARKER_SIZE_MM / 2, str(marker_id), dx=0, dy=3.5, size=2.5)
    out += marker_svg(c.ARM_TAG_ID, ARM_TAG_AT, c.ARM_TAG_SIZE_MM)
    out.append("</g></svg>")

    with open("work_mat.svg", "w") as f:
        f.write("\n".join(out))
    print("Wrote work_mat.svg")


if __name__ == "__main__":
    main()
