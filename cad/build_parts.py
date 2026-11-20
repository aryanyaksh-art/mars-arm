"""Generate every printed part for the Mars arm: the arm itself and the workcell.

Run:  blender-launcher -b --python cad/build_parts.py
(blender-launcher is the Microsoft Store build's command; plain `blender` works elsewhere.)

Writes print-ready STLs to cad/stl/, the assembled scene to
cad/mars_arm_workcell.blend, renders to cad/*.png and a report to cad/build.log.
All dimensions are millimetres (1 Blender unit = 1 mm).

The arm is a direct-drive design for four SG90 / MG90S micro servos:
base (yaw) -> shoulder -> elbow -> one-finger gripper. Each link is glued onto
the horn of the servo that drives it, and carries the next servo.
"""
import contextlib
import math
import os
import random
import traceback

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

OUT = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(OUT, "stl")

# ---------- hardware dimensions: measure yours and edit ----------

# SG90 / MG90S micro servo.
SV_L, SV_W = 22.8, 12.2     # body length and width
SV_BELOW = 15.9             # bottom of body to underside of the mounting tabs
SV_TAB_T = 2.5              # tab thickness
SV_TAB_SPAN = 32.5          # tab tip to tab tip
SV_CASE_ABOVE = 6.8         # underside of tabs to top of the case
SV_SHAFT_OFF = 5.4          # shaft axis to the centre of the body, along its length
SV_SCREW_SPACING = 27.8     # between the two tab screw holes
HORN_H = 14.5               # underside of tabs to the top face of a fitted horn
# Single-arm horn that comes in the servo bag.
HORN_HUB_D, HORN_W0, HORN_W1, HORN_LEN = 7.2, 5.6, 4.0, 17.5  # hub dia, width at hub, width at tip, centre to tip

FIT = 0.5                   # clearance added to the servo pockets
PILOT_D = 1.6               # pilot hole for the servo's own self-tapping screws
RECESS = 1.7                # depth of the horn-shaped recesses
POCKET_L, POCKET_W = SV_L + FIT, SV_W + FIT

DOWEL_D = 12.7              # 1/2 inch hardware-store dowel for the camera mast
DOWEL_LEN = 350.0

# ---------- arm design ----------

LINK_T = 5.0                # thickness of the flat links
L1 = 80.0                   # shoulder axis to elbow axis
L2 = 85.0                   # elbow axis to the middle of the jaws
JAW_GAP = 28.0              # widest object the gripper holds with the finger parallel
FINGER_W, FINGER_LEN = 9.0, 45.0
PIVOT_U = L2 - 35.0         # gripper servo shaft, measured along the forearm
BASE_TOP = 22.0             # height of the base block (base servo hangs in it by its tabs)
PLATE_T = 5.0               # turret plate thickness
RIM_GAP = 0.6               # air gap between the base rim and the turret

# Derived positions. "Arm frame": x forward, y left, z up, origin on the yaw axis at the table.
TURRET_Z0 = BASE_TOP + HORN_H - RECESS            # underside of the turret plate
Z_S = TURRET_Z0 + PLATE_T + SV_W / 2 + 0.3        # shoulder axis height (= BASE_H in the firmware)
Y_F = LINK_T / 2                                  # forearm face that takes the elbow horn
Y_UI = Y_F - RECESS + HORN_H                      # upper arm inner face (elbow servo tabs sit on it)
Y_UO = Y_UI + LINK_T                              # upper arm outer face
Y_M = Y_UI + RECESS - HORN_H                      # turret face the shoulder servo tabs sit on
PALM_W0 = HORN_H - RECESS + LINK_T / 2            # palm face the gripper servo tabs sit on
PIVOT_Z = LINK_T + JAW_GAP + FINGER_W / 2         # gripper shaft, measured across the forearm
GRIP_Y = Y_F - (LINK_T + JAW_GAP / 2)             # jaws' sideways offset (= GRIP_Y in the firmware)

# Workcell layout, same numbers as mars_arm.ino.
BIN_XY = (50.0, -140.0)
ROCK_XY = (120.0, 20.0)
MAST_XY = (120.0, 170.0)
Z_HOVER, Z_GRIP, Z_CARRY = 45.0, 14.0, 75.0


# ---------- mesh helpers ----------

def add_object(name, bm):
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    ob = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def box(name, xr, yr, zr):
    """Axis-aligned box from (min, max) ranges."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    size = (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0])
    center = ((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2)
    bmesh.ops.scale(bm, vec=size, verts=bm.verts)
    bmesh.ops.translate(bm, vec=center, verts=bm.verts)
    return add_object(name, bm)


def taper(name, x0, x1, w0, w1, z0, z1):
    """Prism from x0 to x1 whose y-width goes from w0 to w1."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        near = v.co.x < 0
        v.co.x = x0 if near else x1
        v.co.y *= w0 if near else w1
        v.co.z = z0 if v.co.z < 0 else z1
    return add_object(name, bm)


def cylinder(name, d_bottom, height, base, d_top=None, segments=64, direction=(0, 0, 1)):
    """Cylinder or cone whose bottom face is centred on `base`, extending along `direction`."""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segments, radius1=d_bottom / 2,
                          radius2=(d_bottom if d_top is None else d_top) / 2, depth=height)
    bmesh.ops.translate(bm, vec=(0, 0, height / 2), verts=bm.verts)
    rot = Vector((0, 0, 1)).rotation_difference(Vector(direction).normalized()).to_matrix()
    bmesh.ops.rotate(bm, cent=(0, 0, 0), matrix=rot, verts=bm.verts)
    bmesh.ops.translate(bm, vec=base, verts=bm.verts)
    return add_object(name, bm)


def bake_modifiers(ob):
    depsgraph = bpy.context.evaluated_depsgraph_get()
    mesh = bpy.data.meshes.new_from_object(ob.evaluated_get(depsgraph))
    old = ob.data
    ob.modifiers.clear()
    ob.data = mesh
    bpy.data.meshes.remove(old)
    return ob


def boolean(target, tool, operation):
    mod = target.modifiers.new("bool", "BOOLEAN")
    mod.operation = operation
    mod.solver = "EXACT"
    mod.object = tool
    bake_modifiers(target)
    bpy.data.objects.remove(tool)
    return target


def union(target, tool):
    return boolean(target, tool, "UNION")


def cut(target, tool):
    return boolean(target, tool, "DIFFERENCE")


def merge(name, objects):
    """Combine display-only objects into one mesh (no boolean, shells may overlap)."""
    bm = bmesh.new()
    for ob in objects:
        bm.from_mesh(ob.data)
        bpy.data.objects.remove(ob)
    return add_object(name, bm)


def material(name, rgb, roughness=0.6):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def paint(ob, mat):
    ob.data.materials.clear()
    ob.data.materials.append(mat)
    return ob


def report(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    open_edges = sum(1 for e in bm.edges if len(e.link_faces) != 2)
    volume_cm3 = abs(bm.calc_volume()) / 1000
    bm.free()
    d = ob.dimensions
    print(f"PART {ob.name}: {d.x:.1f} x {d.y:.1f} x {d.z:.1f} mm, {volume_cm3:.1f} cm3, "
          f"non-manifold edges: {open_edges}")
    return open_edges


def export_stl(ob, filename, flip=False):
    """Export in print orientation: optionally flipped, sitting on z=0, centred in xy."""
    mesh = ob.data.copy()
    if flip:
        mesh.transform(Matrix.Rotation(math.pi, 4, "X"))
    xs, ys, zs = zip(*(v.co for v in mesh.vertices))
    mesh.transform(Matrix.Translation((-(min(xs) + max(xs)) / 2, -(min(ys) + max(ys)) / 2, -min(zs))))
    tmp = bpy.data.objects.new("export", mesh)
    bpy.context.scene.collection.objects.link(tmp)
    for other in bpy.context.scene.objects:
        other.select_set(False)
    tmp.select_set(True)
    bpy.context.view_layer.objects.active = tmp
    bpy.ops.wm.stl_export(filepath=os.path.join(STL_DIR, filename), export_selected_objects=True)
    bpy.data.objects.remove(tmp)
    bpy.data.meshes.remove(mesh)


# ---------- servo features ----------

def horn_recess(part, x, y, face_z, on_top=True, angles=(0.0,)):
    """Horn-shaped pocket in a face, plus a hole so a screwdriver reaches the horn screw."""
    z0, z1 = (face_z - RECESS, face_z + 1) if on_top else (face_z - 1, face_z + RECESS)
    cut(part, cylinder("hub", HORN_HUB_D + 0.6, z1 - z0, (x, y, z0), segments=48))
    for angle in angles:
        slot = taper("slot", -1.3, HORN_LEN + 1, HORN_W0 + 0.6, HORN_W1 + 0.8, z0, z1)
        slot.data.transform(Matrix.Translation((x, y, 0)) @ Matrix.Rotation(angle, 4, "Z"))
        cut(part, slot)
    cut(part, cylinder("driver", 5.0, 40, (x, y, face_z - 20), segments=32))


def servo_pocket(part, shaft_x, y, z0, z1):
    """Through pocket along z for a servo whose shaft sits at shaft_x, body trailing toward -x."""
    cx = shaft_x - SV_SHAFT_OFF
    cut(part, box("pocket", (cx - POCKET_L / 2, cx + POCKET_L / 2),
                  (y - POCKET_W / 2, y + POCKET_W / 2), (z0 - 1, z1 + 1)))
    for dx in (-SV_SCREW_SPACING / 2, SV_SCREW_SPACING / 2):
        cut(part, cylinder("pilot", PILOT_D, z1 - z0 + 2, (cx + dx, y, z0 - 1), segments=16))


def servo_dummy(name):
    """Stand-in servo: tab underside on z=0, shaft along +z through the origin."""
    cx = -SV_SHAFT_OFF
    return merge(name, [
        box("b", (cx - SV_L / 2, cx + SV_L / 2), (-SV_W / 2, SV_W / 2), (-SV_BELOW, SV_CASE_ABOVE)),
        box("t", (cx - SV_TAB_SPAN / 2, cx + SV_TAB_SPAN / 2), (-SV_W / 2, SV_W / 2), (0.02, SV_TAB_T)),
        cylinder("g", 11.8, 4.0, (0, 0, SV_CASE_ABOVE), segments=32),
        cylinder("s", 4.6, 12.3 - SV_CASE_ABOVE, (0, 0, SV_CASE_ABOVE), segments=24),
    ])


# ---------- arm parts (each built in the orientation it prints in) ----------

def make_base():
    """Drum the base servo hangs in, on a flange you screw, tape or clamp down.

    The rim on top stops just under the turret, so the turret can lean on it
    instead of levering the little servo shaft sideways.
    """
    base = box("base", (-55, 45), (-35, 35), (0, 3))
    union(base, cylinder("drum", 60, BASE_TOP, (0, 0, 0), segments=96))
    rim_h = TURRET_Z0 - RIM_GAP - BASE_TOP + 0.5
    rim = cylinder("rim", 57, rim_h, (0, 0, BASE_TOP - 0.5), segments=96)
    cut(rim, cylinder("bore", 49, rim_h + 2, (0, 0, BASE_TOP - 1), segments=96))
    union(base, rim)
    servo_pocket(base, 0, 0, 0, BASE_TOP)
    cut(base, box("cable", (-42, 31), (-2.5, 2.5), (-1, 11)))  # servo lead exits under either end
    for x in (-47, 37):
        for y in (-27, 27):
            cut(base, cylinder("screw", 4.5, 6, (x, y, -1), segments=32))
    return base


def make_turret():
    """Sits on the base servo's horn and cradles the shoulder servo on its side."""
    z1 = TURRET_Z0 + PLATE_T
    turret = cylinder("turret", 58, PLATE_T, (0, 0, TURRET_Z0), segments=96)
    cut(turret, box("flat", (-40, 40), (Y_M + 6.5, 40), (TURRET_Z0 - 1, z1 + 1)))  # room for the upper arm to swing
    cx = -SV_SHAFT_OFF
    for x0, x1 in ((cx - POCKET_L / 2 - 6.5, cx - POCKET_L / 2), (cx + POCKET_L / 2, cx + POCKET_L / 2 + 6.5)):
        union(turret, box("cheek", (x0, x1), (Y_M - 10.5, Y_M), (z1 - 0.5, Z_S + POCKET_W / 2)))
    for dx in (-SV_SCREW_SPACING / 2, SV_SCREW_SPACING / 2):
        cut(turret, cylinder("pilot", PILOT_D, 10, (cx + dx, Y_M + 1, Z_S), segments=16, direction=(0, -1, 0)))
    horn_recess(turret, 0, 0, TURRET_Z0, on_top=False, angles=(0.0, math.pi))  # double-arm horn
    return turret


def make_upper_arm():
    """Shoulder horn at one end, elbow servo at the other. Prints outer face down."""
    link = cylinder("upper_arm", 21, LINK_T, (0, 0, 0))
    union(link, box("bar", (0, L1 + 13), (-10.5, 10.5), (0, LINK_T)))
    horn_recess(link, 0, 0, LINK_T)
    servo_pocket(link, L1, 0, 0, LINK_T)
    return link


def make_forearm():
    """Elbow horn at one end; the far end is the fixed jaw, with a wall holding the gripper servo."""
    link = cylinder("forearm", 16, LINK_T, (0, 0, 0))
    union(link, box("bar", (0, 62), (-8, 8), (0, LINK_T)))
    union(link, box("jaw", (60, PIVOT_U + FINGER_LEN), (-6, 6), (0, LINK_T)))
    zc = PIVOT_Z - SV_SHAFT_OFF  # centre of the gripper servo body
    union(link, box("riser", (PIVOT_U - 10, PIVOT_U + 10), (7, PALM_W0 + LINK_T), (0, LINK_T)))
    union(link, box("palm", (PIVOT_U - 10, PIVOT_U + 10), (PALM_W0, PALM_W0 + LINK_T),
                    (0, zc + SV_SCREW_SPACING / 2 + 4.5)))
    cut(link, box("pocket", (PIVOT_U - POCKET_W / 2, PIVOT_U + POCKET_W / 2),
                  (PALM_W0 - 1, PALM_W0 + LINK_T + 1), (zc - POCKET_L / 2, zc + POCKET_L / 2)))
    for dz in (-SV_SCREW_SPACING / 2, SV_SCREW_SPACING / 2):
        cut(link, cylinder("pilot", PILOT_D, LINK_T + 2, (PIVOT_U, PALM_W0 - 1, zc + dz),
                           segments=16, direction=(0, 1, 0)))
    horn_recess(link, 0, 0, 0, on_top=False)
    return link


def make_finger():
    """Moving jaw: glues onto the gripper servo's horn."""
    finger = cylinder("finger", 14, LINK_T, (0, 0, 0))
    union(finger, box("blade", (0, FINGER_LEN), (-FINGER_W / 2, FINGER_W / 2), (0, LINK_T)))
    horn_recess(finger, 0, 0, LINK_T)
    return finger


def make_fit_test():
    """Ten-minute test print: check the servo pocket, screw holes and horn recess before the real parts."""
    coupon = box("fit_test", (-23, 23), (-14, 24), (0, LINK_T))
    servo_pocket(coupon, SV_SHAFT_OFF, 10, 0, LINK_T)
    horn_recess(coupon, -10, -7, LINK_T)
    return coupon


# ---------- workcell parts ----------

def make_mast_foot():
    """Plate that tapes or clamps to the desk, with a socket for the dowel."""
    plate = box("mast_foot", (-70, 70), (-45, 45), (0, 6))
    bevel = plate.modifiers.new("bevel", "BEVEL")
    bevel.width, bevel.segments = 1.5, 2
    bake_modifiers(plate)
    union(plate, cylinder("boss", 50, 45, (0, 0, 5.9), d_top=26, segments=96))
    cut(plate, cylinder("socket", DOWEL_D + FIT, 46, (0, 0, 6), segments=96))
    for x in (-55, 55):
        cut(plate, cylinder("screw", 4.5, 10, (x, 0, -2), segments=32))
    return plate


def make_camera_boom():
    """Cap for the top of the dowel with an arm and a plate the webcam clips onto.

    The top face is flat, so it prints upside down with no supports.
    """
    boom = cylinder("camera_boom", 26, 45, (0, 0, -45), segments=96)
    union(boom, box("beam", (0, 150), (-9, 9), (-10, 0)))
    union(boom, box("plate", (142.5, 197.5), (-25, 25), (-6, 0)))
    cut(boom, cylinder("socket", DOWEL_D + FIT, 41, (0, 0, -46), segments=96))
    cut(boom, cylinder("tripod_hole", 6.5, 10, (170, 0, -8), segments=48))
    cut(boom, cylinder("set_screw", 2.8, 16, (-14, 0, -25), segments=24, direction=(1, 0, 0)))
    return boom


def make_sample_bin():
    """Tapered open cup: wide mouth so a slightly-off drop still lands inside."""
    wall, floor, height, d_bottom, d_top = 2.0, 2.0, 45.0, 64.0, 90.0
    outer_d = lambda z: d_bottom + (d_top - d_bottom) * z / height
    cup = cylinder("sample_bin", d_bottom, height, (0, 0, 0), d_top=d_top, segments=96)
    cut(cup, cylinder("cavity", outer_d(floor) - 2 * wall, height - floor + 1, (0, 0, floor),
                      d_top=outer_d(height + 1) - 2 * wall, segments=96))
    return cup


def make_rock():
    """Faceted 'mineral sample', flat underneath so it sits still. Print in orange."""
    rng = random.Random(4)
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=2, radius=13.0)
    for v in bm.verts:
        v.co *= rng.uniform(0.88, 1.12)
    bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:],
                           plane_co=(0, 0, -8), plane_no=(0, 0, 1), clear_inner=True)
    bmesh.ops.holes_fill(bm, edges=[e for e in bm.edges if e.is_boundary], sides=0)
    bmesh.ops.translate(bm, vec=(0, 0, 8), verts=bm.verts)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return add_object("rock_sample", bm)


# ---------- assembly ----------

def solve_ik(x, y, z):
    """Same maths as solveIK() in the firmware. Returns yaw, upper, inner in radians."""
    r = math.hypot(x, y)
    yaw = math.atan2(y, x) - math.asin(GRIP_Y / r)
    rp = math.sqrt(r * r - GRIP_Y * GRIP_Y)
    zw = z - Z_S
    d = math.hypot(rp, zw)
    upper = math.atan2(zw, rp) + math.acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d))
    inner = math.acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2))
    return yaw, upper, inner


def rows(*r):
    return Matrix(r).to_4x4()


def link_frame(angle):
    """Link's print frame -> arm frame: x along the link, y across it, z toward -y (away from its servo)."""
    c, s = math.cos(angle), math.sin(angle)
    return rows((c, -s, 0), (0, 0, -1), (s, c, 0))


SIDE_ON = rows((1, 0, 0), (0, 0, 1), (0, -1, 0))         # local z -> arm +y
GRIP_SERVO = rows((0, -1, 0), (0, 0, -1), (1, 0, 0))     # shaft toward -w, body toward the fixed jaw


def set_pose(arm, yaw, upper, inner, finger_open):
    """Place every moving part for the given joint angles (radians)."""
    T = Matrix.Translation
    turn = Matrix.Rotation(yaw, 4, "Z")
    arm["turret"].matrix_world = turn
    arm["sv_shoulder"].matrix_world = turn @ T((0, Y_M, Z_S)) @ SIDE_ON
    arm["upper_arm"].matrix_world = turn @ T((0, Y_UO, Z_S)) @ link_frame(upper)
    arm["sv_elbow"].matrix_world = arm["upper_arm"].matrix_world @ T((L1, 0, LINK_T))
    elbow = (L1 * math.cos(upper), Y_F, Z_S + L1 * math.sin(upper))
    arm["forearm"].matrix_world = turn @ T(elbow) @ link_frame(upper + inner - math.pi)
    arm["sv_grip"].matrix_world = arm["forearm"].matrix_world @ T((PIVOT_U, PALM_W0, PIVOT_Z)) @ GRIP_SERVO
    arm["finger"].matrix_world = (arm["forearm"].matrix_world @ T((PIVOT_U, -LINK_T / 2, PIVOT_Z))
                                  @ SIDE_ON @ Matrix.Rotation(-finger_open, 4, "Z"))


def world_bvh(ob):
    m = ob.matrix_world
    return BVHTree.FromPolygons([m @ v.co for v in ob.data.vertices],
                                [tuple(p.vertices) for p in ob.data.polygons])


# Pairs that touch by design: a servo and the part it is screwed into.
MOUNTED = {frozenset(p) for p in (("base", "sv_base"), ("turret", "sv_shoulder"),
                                  ("upper_arm", "sv_elbow"), ("forearm", "sv_grip"))}


def check_pose(label, arm, obstacles):
    """Report any two parts whose surfaces intersect, and how close the arm gets to the table."""
    bpy.context.view_layer.update()
    things = {**arm, **obstacles}
    trees = {name: world_bvh(ob) for name, ob in things.items()}
    names = list(things)
    hits = [f"{a}/{b}" for i, a in enumerate(names) for b in names[i + 1:]
            if frozenset((a, b)) not in MOUNTED and not (a in obstacles and b in obstacles)
            and trees[a].overlap(trees[b])]
    moving = ("upper_arm", "sv_elbow", "forearm", "sv_grip", "finger")
    low = min((arm[n].matrix_world @ v.co).z for n in moving for v in arm[n].data.vertices)
    print(f"POSE {label}: lowest point {low:.1f} mm above table, collisions: {', '.join(hits) or 'none'}")
    return len(hits) + (1 if low < 0 else 0)


def build_scene(arm, workcell):
    printed = material("printed blue", (0.05, 0.25, 0.75))
    accent = material("printed orange", (0.95, 0.35, 0.05))
    servo = material("servo", (0.02, 0.08, 0.35), 0.35)
    orange = material("sample orange", (1.0, 0.32, 0.02), 0.5)
    wood = material("dowel", (0.72, 0.52, 0.30))
    dark = material("dark", (0.05, 0.05, 0.06), 0.4)
    grey = material("grey", (0.55, 0.55, 0.55))
    regolith = material("regolith", (0.45, 0.20, 0.11), 0.95)
    mat_white = material("work mat", (0.9, 0.88, 0.84), 0.9)

    paint(box("table", (-250, 450), (-350, 350), (-4, 0)), regolith)
    paint(box("work_area", (85, 145), (-45, 65), (0, 0.4)), mat_white)  # pick area: x 90-140, y -40..60
    for name, ob in arm.items():
        paint(ob, servo if name.startswith("sv_") else accent if name in ("turret", "finger") else printed)

    mx, my = MAST_XY
    paint(workcell["mast_foot"], grey)
    paint(cylinder("dowel", DOWEL_D, DOWEL_LEN, (mx, my, 6), segments=32), wood)
    paint(workcell["camera_boom"], grey)
    top = 6 + DOWEL_LEN + 5
    paint(box("webcam", (mx - 35, mx + 35), (my - 184, my - 156), (top - 36, top - 6)), dark)
    paint(cylinder("lens", 16, 6, (mx, my - 170, top - 42), segments=32), grey)
    paint(workcell["sample_bin"], grey)
    paint(workcell["rock_sample"], orange)

    scene = bpy.context.scene
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.data.angle = math.radians(8)
    sun.rotation_euler = (math.radians(50), 0, math.radians(35))
    scene.collection.objects.link(sun)
    world = bpy.data.worlds.new("world")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.75, 0.62, 0.55, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.8
    scene.world = world


def render(name, location, target, lens):
    scene = bpy.context.scene
    cam = bpy.data.objects.new(name, bpy.data.cameras.new(name))
    cam.location = location
    cam.rotation_euler = (Vector(target) - Vector(location)).to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    cam.data.clip_end = 10000
    scene.collection.objects.link(cam)
    scene.camera = cam
    scene.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)


def main():
    os.makedirs(STL_DIR, exist_ok=True)
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = "MILLIMETERS"
    print(f"GEOMETRY for firmware: L1={L1} L2={L2} BASE_H={Z_S:.1f} GRIP_Y={GRIP_Y:.1f}")

    printable = {
        "arm_base.stl": make_base(), "arm_turret.stl": make_turret(),
        "arm_upper.stl": make_upper_arm(), "arm_forearm.stl": make_forearm(),
        "arm_finger.stl": make_finger(), "fit_test.stl": make_fit_test(),
        "mast_foot.stl": make_mast_foot(), "camera_boom.stl": make_camera_boom(),
        "sample_bin.stl": make_sample_bin(), "rock_sample.stl": make_rock(),
    }
    bpy.context.view_layer.update()
    bad = sum(report(ob) for ob in printable.values())
    for filename, ob in printable.items():
        export_stl(ob, filename, flip=(filename == "camera_boom.stl"))
    bpy.data.objects.remove(printable.pop("fit_test.stl"))

    by_name = {ob.name: ob for ob in printable.values()}
    arm = {n: by_name[n] for n in ("base", "turret", "upper_arm", "forearm", "finger")}
    for n in ("sv_base", "sv_shoulder", "sv_elbow", "sv_grip"):
        arm[n] = servo_dummy(n)
    arm["sv_base"].matrix_world = Matrix.Translation((0, 0, BASE_TOP))
    workcell = {n: by_name[n] for n in ("mast_foot", "camera_boom", "sample_bin", "rock_sample")}
    workcell["mast_foot"].location = (*MAST_XY, 0)
    workcell["camera_boom"].location = (*MAST_XY, 6 + DOWEL_LEN + 5)
    workcell["camera_boom"].rotation_euler = (0, 0, math.radians(-90))  # reach over the work area
    workcell["sample_bin"].location = (*BIN_XY, 0)
    workcell["rock_sample"].location = (*ROCK_XY, 0)
    obstacles = {n: workcell[n] for n in ("mast_foot", "sample_bin")}

    # Sweep the poses the firmware actually uses and look for parts running into each other.
    open_angle = math.radians(30)
    set_pose(arm, 0, math.radians(90), math.radians(70), open_angle)
    clashes = check_pose("home", arm, obstacles)
    for x, y in ((120, 20), (90, 0), (90, 60), (90, -40), (140, 60), (140, -40), (140, 0), (115, -40)):
        for z, finger in ((Z_HOVER, open_angle), (Z_GRIP, open_angle), (Z_GRIP, 0.0), (Z_CARRY, 0.0)):
            set_pose(arm, *solve_ik(x, y, z), finger)
            clashes += check_pose(f"x={x} y={y} z={z:.0f} finger={math.degrees(finger):.0f}", arm, obstacles)
    for finger in (0.0, open_angle):
        set_pose(arm, *solve_ik(*BIN_XY, Z_CARRY), finger)
        clashes += check_pose(f"over bin finger={math.degrees(finger):.0f}", arm, obstacles)

    set_pose(arm, *solve_ik(*ROCK_XY, Z_GRIP), 0.0)  # pose for the pictures: gripping the rock
    build_scene(arm, workcell)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "mars_arm_workcell.blend"))

    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 40
    scene.render.resolution_x, scene.render.resolution_y = 1400, 900
    render("preview", (560, -470, 400), (95, 10, 120), 32)
    render("arm_right", (250, -230, 150), (70, -5, 55), 40)
    render("arm_left", (-90, 250, 190), (65, 0, 55), 40)
    print(f"DONE non-manifold edges total: {bad}, pose problems: {clashes}")


# The Store build of Blender only starts through blender-launcher.exe, which
# hides the console, so the report also goes to cad/build.log.
with open(os.path.join(OUT, "build.log"), "w") as log:
    try:
        with contextlib.redirect_stdout(log):
            main()
    except Exception:
        traceback.print_exc(file=log)
        raise
