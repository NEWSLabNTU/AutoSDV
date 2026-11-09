#!/usr/bin/env python3
"""A low-poly USD body of AutoSDV car A, for Isaac Sim (render only) and RViz.

    python3 make_body.py OUT_DIR [--dae MESH.dae]

writes OUT_DIR/vehicle.newslab.autosdv_car_a/:
  vehicle.newslab.autosdv_car_a.usda  default prim /Vehicle: the car, its sensors and wheels;
  wheels.usda                         /Wheel_Front_Left, ..., each centred on its own origin,
                                      axle along y;
  entry.json                          the car's entry for swb_isaac_world's vehicles.json
                                      (paths relative to OUT_DIR).
and, with --dae, the same car as one COLLADA mesh in base_link (rear axle centre on the
ground), for autosdv_vehicle_description.

Frame (as CARLA's vehicle assets): metres, z up, x forward, y left, the origin on the ground at
mid-length. The rear axle is at x = rear_axle_x (entry.json); swb's add_ego_body() puts it on
base_link.

Geometry, wheels and every sensor's pose are read from the twin's vehicle spec
(../vehicle/autosdv_car_a.yaml), so the body cannot drift from it. The parts between are
estimated from 14 photos of car A (2026-11-10), scaled by the 0.319 m wheelbase and the
VLP-16's 103 x 72 mm housing; heights are in base_link. No physics, no colliders: aib's PhysX
vehicle and its box are the physics.

Needs only usd-core (pxr), numpy and PyYAML. The mesh helpers follow the golf cart twin's
make_body.py (NEWSLabNTU/2026-golf-cart, src/simulation/golfcart_twin/body).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

ASSET_ID = "vehicle.newslab.autosdv_car_a"
SPEC = Path(__file__).resolve().parents[1] / "vehicle" / "autosdv_car_a.yaml"

S = yaml.safe_load(SPEC.read_text())
G = S["geometry"]
WHEEL_BASE, WHEEL_TREAD = G["wheel_base"], G["wheel_tread"]
WHEEL_RADIUS, WHEEL_WIDTH = G["wheel_radius"], G["wheel_width"]
MAX_STEER_DEG = math.degrees(G["max_steer_angle"])
X_FRONT = WHEEL_BASE + G["front_overhang"]
X_REAR = -G["rear_overhang"]
HALF_W = WHEEL_TREAD / 2 + G["left_overhang"]
X_SHIFT = -(X_FRONT + X_REAR) / 2.0          # base_link -> asset frame
REAR_AXLE_X = X_SHIFT


def _xyz(m):
    return np.array([m["x"], m["y"], m["z"]], float)


KIT = _xyz(S["sensor_kit"]["mount"])          # all rotations are 0 in this spec
LIDAR = KIT + _xyz(S["sensors"]["vlp16"]["mount"])
ZED_LEFT = KIT + _xyz(S["sensors"]["zed"]["mount"])
GNSS = KIT + _xyz(S["sensors"]["gnss"]["mount"])
for name, s in S["sensors"].items():
    m = s["mount"]
    if any(abs(m[k]) > 1e-9 for k in ("roll", "pitch", "yaw")):
        raise SystemExit(f"{name}: a rotated mount; make_body.py places housings unrotated")

# ---- mesh primitives: (points [N,3], faces [index lists]) in base_link ------------------------


class Part:
    def __init__(self):
        self.pts, self.faces = [], []

    def add(self, pts, faces):
        o = sum(len(p) for p in self.pts)
        self.pts.append(np.asarray(pts, float))
        self.faces += [[i + o for i in f] for f in faces]
        return self

    def points(self):
        return np.concatenate(self.pts) if self.pts else np.zeros((0, 3))


def box_span(x, y, z):
    lo, hi = np.array([x[0], y[0], z[0]], float), np.array([x[1], y[1], z[1]], float)
    p = np.array([[sx, sy, sz] for sx in (0, 1) for sy in (0, 1) for sz in (0, 1)], float)
    p = lo + p * (hi - lo)
    f = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return p, f


def cbox(c, size):
    c, h = np.asarray(c, float), np.asarray(size, float) / 2
    return box_span((c[0] - h[0], c[0] + h[0]), (c[1] - h[1], c[1] + h[1]), (c[2] - h[2], c[2] + h[2]))


def cylinder(p0, p1, r, n=20):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = (p1 - p0) / np.linalg.norm(p1 - p0)
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < 0.9 else [1, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    ring = [r * (math.cos(t) * u + math.sin(t) * v) for t in np.arange(n) * 2 * math.pi / n]
    pts = [p0 + q for q in ring] + [p1 + q for q in ring]
    f = [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    f += [list(range(n))[::-1], list(range(n, 2 * n))]
    return np.array(pts), f


def zcyl(x, y, z0, z1, r, n=20):
    return cylinder((x, y, z0), (x, y, z1), r, n)


def lathe_y(profile, n=24):
    m = len(profile)
    pts = [[r * math.cos(2 * math.pi * k / n), y, r * math.sin(2 * math.pi * k / n)]
           for k in range(n) for r, y in profile]
    f = [[k * m + i, k * m + (i + 1) % m, ((k + 1) % n) * m + (i + 1) % m, ((k + 1) % n) * m + i]
         for k in range(n) for i in range(m)]
    return np.array(pts), f


# ---- car A, in base_link -----------------------------------------------------------------------
# Heights from the side photos (78ce8998, 23aca77f): chassis tub 0.02-0.045, the middle deck at
# 0.095, the Jetson deck at 0.166, the Jetson 0.17-0.26, the top plate at 0.274, the lidar's
# base at its optical centre less 37.7 mm (Velodyne's VLP-16 drawing).

LIDAR_R, LIDAR_BELOW = 0.0515, 0.0377
# The RTX lidar renders this body: a housing around its origin catches every ray inside the
# near range and the sensor returns nothing (the golf cart twin's VLP_CLEAR). So the VLP-16's
# housing ends LIDAR_CLEAR below its optical centre: its base is drawn, its window and cap,
# which the rays would leave through, are not. Likewise the ZED's box ends behind its lens.
LIDAR_CLEAR = 0.03
TOP_PLATE_Z = 0.274
JETSON_DECK_Z = 0.166
MID_DECK_Z = 0.095


def build():
    parts = {}

    def part(name, mat):
        parts[name] = (Part(), mat)
        return parts[name][0]

    # chassis: the tub, the bumpers, the shock towers and the oil shocks
    tub = part("Chassis/Tub", "black_plastic")
    tub.add(*box_span((-0.045, 0.355), (-0.095, 0.095), (0.018, 0.026)))
    tub.add(*box_span((0.335, 0.37), (-0.075, 0.075), (0.026, 0.06)))          # front bumper
    tub.add(*box_span((-0.07, -0.04), (-0.07, 0.07), (0.03, 0.06)))            # rear bumper
    arms = part("Chassis/Arms", "black_plastic")
    for xa in (0.0, WHEEL_BASE):
        arms.add(*box_span((xa - 0.012, xa + 0.012), (-WHEEL_TREAD / 2 + 0.02, WHEEL_TREAD / 2 - 0.02),
                           (0.035, 0.045)))
        arms.add(*box_span((xa - 0.03, xa + 0.005), (-0.07, 0.07), (0.045, 0.115)))   # shock tower
    shocks = part("Chassis/Shocks", "alu_black")
    for xa in (0.0, WHEEL_BASE):
        for s in (-1, 1):
            shocks.add(*cylinder((xa - 0.01, s * 0.06, 0.112), (xa - 0.005, s * 0.105, 0.05), 0.008, 10))
    drive = part("Chassis/Drivetrain", "alloy")
    drive.add(*cylinder((0.0, 0.0, 0.035), (WHEEL_BASE, 0.0, 0.035), 0.004, 8))   # centre shaft
    drive.add(*zcyl(0.13, 0.03, 0.03, 0.065, 0.018))                            # motor can
    batt = part("Chassis/Battery", "dark_plastic")
    batt.add(*box_span((0.16, 0.30), (-0.085, -0.04), (0.026, 0.05)))

    # the middle deck, the breaker and the red printed parts on it
    deck = part("Decks/Middle", "black_plastic")
    deck.add(*box_span((-0.04, 0.30), (-0.085, 0.085), (MID_DECK_Z - 0.003, MID_DECK_Z)))
    part("Decks/Breaker", "white_plate").add(*box_span((0.0, 0.07), (-0.075, -0.03), (MID_DECK_Z, 0.165)))
    part("Decks/Red_blocks", "red_print").add(*box_span((0.075, 0.14), (-0.04, 0.04), (MID_DECK_Z, 0.12)))

    # the Jetson deck on red posts, the Jetson, the top plate, the lidar on standoffs
    jd = part("Compute/Deck", "black_plastic")
    jd.add(*box_span((-0.025, 0.155), (-0.075, 0.075), (JETSON_DECK_Z - 0.004, JETSON_DECK_Z)))
    posts = part("Compute/Posts", "red_anodized")
    for px in (-0.015, 0.145):
        for py in (-0.065, 0.065):
            posts.add(*zcyl(px, py, MID_DECK_Z, TOP_PLATE_Z, 0.005, 10))
    part("Compute/Jetson", "jetson_grey").add(*box_span((0.01, 0.135), (-0.055, 0.055), (JETSON_DECK_Z, 0.26)))
    part("Compute/Jetson_fins", "black_plastic").add(*box_span((0.02, 0.125), (-0.056, 0.056), (0.225, 0.255)))
    part("Compute/Top_plate", "black_plastic").add(
        *box_span((-0.025, 0.16), (-0.075, 0.075), (TOP_PLATE_Z - 0.004, TOP_PLATE_Z)))

    lx, ly, lz = LIDAR
    base_z = lz - LIDAR_BELOW
    so = part("Lidar/Standoffs", "alloy")
    for dx, dy in ((0.03, 0.02), (0.03, -0.02), (-0.03, 0.02), (-0.03, -0.02)):
        so.add(*zcyl(lx + dx, ly + dy, TOP_PLATE_Z, base_z - 0.012, 0.0055, 10))
    part("Lidar/Mount", "red_anodized").add(*zcyl(lx, ly, base_z - 0.012, base_z, LIDAR_R + 0.004, 28))
    part("Lidar/Base", "lidar_body").add(*zcyl(lx, ly, base_z, lz - LIDAR_CLEAR, LIDAR_R, 32))

    # the front deck on standoffs, the Velodyne interface box, the ZED X Mini
    fd = part("Front/Deck", "black_plastic")
    fd.add(*box_span((0.17, 0.31), (-0.07, 0.07), (0.158, 0.162)))
    fso = part("Front/Standoffs", "alloy")
    for px in (0.18, 0.30):
        for py in (-0.06, 0.06):
            fso.add(*zcyl(px, py, MID_DECK_Z, 0.158, 0.004, 8))
    part("Front/Interface_box", "dark_plastic").add(*box_span((0.199, 0.282), (-0.055, 0.055), (0.162, 0.201)))
    zx, zy, zz = ZED_LEFT
    zc = np.array([zx - 0.020, 0.0, zz])                       # the box's front face 4 mm behind the lens
    part("Front/ZED", "alu_black").add(*cbox(zc, (0.032, 0.094, 0.032)))
    lens = part("Front/ZED_lenses", "lens")
    for y in (zy, -zy):
        lens.add(*cylinder((zx - 0.004, y, zz), (zx - 0.002, y, zz), 0.008, 16))

    # the red rear mount: bracket, the hexagonal enclosure, the GNSS puck on its disc
    rm = part("Rear/Mount", "red_print")
    rm.add(*box_span((-0.075, -0.03), (-0.035, 0.035), (0.06, MID_DECK_Z)))
    rm.add(*box_span((-0.075, -0.045), (-0.055, 0.055), (0.10, 0.215)))         # the enclosure
    gx, gy, gz = GNSS
    rm.add(*zcyl(gx, gy, gz - 0.066, gz - 0.06, 0.022, 20))                     # the puck's disc
    part("Rear/GNSS", "black_plastic").add(*zcyl(gx, gy, gz - 0.06, gz, 0.017, 20))
    return parts


def wheel(left):
    """One wheel, centred on its origin, axle along y; the spokes face out (+y on the left)."""
    s = 1.0 if left else -1.0
    hw, R, rim = WHEEL_WIDTH / 2, WHEEL_RADIUS, 0.034
    prof = [(rim, -hw + 0.002), (rim + 0.004, -hw), (R - 0.006, -hw + 0.001), (R - 0.0015, -hw + 0.004),
            (R, -hw + 0.008), (R, hw - 0.008), (R - 0.0015, hw - 0.004), (R - 0.006, hw - 0.001),
            (rim + 0.004, hw), (rim, hw - 0.002)]
    tyre = Part().add(*lathe_y(prof, 28))
    hub = Part().add(*cylinder((0, -hw + 0.003, 0), (0, hw - 0.003, 0), rim, 20))
    face = s * (hw - 0.002)
    spokes = Part()
    for k in range(10):
        t = 2 * math.pi * k / 10
        d = np.array([math.cos(t), 0, math.sin(t)])
        spokes.add(*cylinder(d * 0.008 + [0, face, 0], d * (rim - 0.002) + [0, face, 0], 0.0018, 6))
    spokes.add(*cylinder((0, face - s * 0.002, 0), (0, face + s * 0.003, 0), 0.008, 12))
    return {"Tyre": (tyre, "tyre"), "Hub": (hub, "rim_black"), "Spokes": (spokes, "rim_black")}


MATERIALS = {   # name: (diffuse, roughness, metallic)
    "black_plastic": ((0.025, 0.025, 0.025), 0.6, 0.0),
    "dark_plastic": ((0.08, 0.085, 0.09), 0.5, 0.0),
    "white_plate": ((0.80, 0.80, 0.78), 0.4, 0.0),
    "red_print": ((0.80, 0.05, 0.04), 0.7, 0.0),
    "red_anodized": ((0.60, 0.03, 0.03), 0.3, 0.7),
    "jetson_grey": ((0.35, 0.36, 0.37), 0.4, 0.6),
    "alu_black": ((0.05, 0.05, 0.055), 0.35, 0.6),
    "alloy": ((0.55, 0.56, 0.58), 0.3, 0.9),
    "lidar_body": ((0.62, 0.63, 0.65), 0.45, 0.5),
    "lens": ((0.01, 0.012, 0.015), 0.05, 0.0),
    "tyre": ((0.02, 0.02, 0.02), 0.85, 0.0),
    "rim_black": ((0.04, 0.04, 0.045), 0.35, 0.6),
}


def define_materials(stage, root):
    UsdGeom.Scope.Define(stage, f"{root}/Looks")
    out = {}
    for name, (col, rough, metal) in MATERIALS.items():
        mat = UsdShade.Material.Define(stage, f"{root}/Looks/{name}")
        sh = UsdShade.Shader.Define(stage, f"{root}/Looks/{name}/PreviewSurface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        out[name] = mat
    return out


def write_mesh(stage, path, part, mat_name, mat, shift=(0.0, 0.0, 0.0)):
    pts = part.points() + np.asarray(shift)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    m.CreateFaceVertexCountsAttr([len(f) for f in part.faces])
    m.CreateFaceVertexIndicesAttr([i for f in part.faces for i in f])
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    m.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([Gf.Vec3f(*MATERIALS[mat_name][0])])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return sum(len(f) - 2 for f in part.faces), pts


def wheel_positions():
    """name -> (centre in base_link, left?)"""
    out = {}
    for name, xb, left in (("Wheel_Front_Left", WHEEL_BASE, True), ("Wheel_Front_Right", WHEEL_BASE, False),
                           ("Wheel_Rear_Left", 0.0, True), ("Wheel_Rear_Right", 0.0, False)):
        out[name] = (np.array([xb, (1 if left else -1) * WHEEL_TREAD / 2, WHEEL_RADIUS]), left)
    return out


def all_parts_base_link():
    """Every part, the wheels included, in base_link: [(name, Part, material)]."""
    out = [(n, p, m) for n, (p, m) in build().items()]
    for name, (c, left) in wheel_positions().items():
        for pn, (p, m) in wheel(left).items():
            q = Part().add(p.points() + c, p.faces)
            out.append((f"{name}/{pn}", q, m))
    return out


def write_dae(path):
    """The whole car as one COLLADA file in base_link, a geometry per material."""
    by_mat = {}
    for _, p, m in all_parts_base_link():
        q = by_mat.setdefault(m, Part())
        tris = []
        for f in p.faces:
            tris += [[f[0], f[i], f[i + 1]] for i in range(1, len(f) - 1)]
        q.add(p.points(), tris)
    lib_fx, lib_mat, lib_geo, nodes = [], [], [], []
    for m, p in by_mat.items():
        r, g, b = MATERIALS[m][0]
        lib_fx.append(f'<effect id="{m}-fx"><profile_COMMON><technique sid="c"><lambert><diffuse>'
                      f'<color>{r} {g} {b} 1</color></diffuse></lambert></technique></profile_COMMON></effect>')
        lib_mat.append(f'<material id="{m}-mat" name="{m}"><instance_effect url="#{m}-fx"/></material>')
        pts = p.points()
        flat = " ".join(f"{v:.5f}" for v in pts.ravel())
        idx = " ".join(str(i) for f in p.faces for i in f)
        lib_geo.append(
            f'<geometry id="{m}-geo"><mesh><source id="{m}-pos"><float_array id="{m}-arr" count="{pts.size}">'
            f'{flat}</float_array><technique_common><accessor source="#{m}-arr" count="{len(pts)}" stride="3">'
            f'<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
            f'</accessor></technique_common></source><vertices id="{m}-vtx"><input semantic="POSITION" '
            f'source="#{m}-pos"/></vertices><triangles material="{m}-sym" count="{len(p.faces)}">'
            f'<input semantic="VERTEX" source="#{m}-vtx" offset="0"/><p>{idx}</p></triangles></mesh></geometry>')
        nodes.append(f'<node id="{m}-node"><instance_geometry url="#{m}-geo"><bind_material><technique_common>'
                     f'<instance_material symbol="{m}-sym" target="#{m}-mat"/></technique_common></bind_material>'
                     f'</instance_geometry></node>')
    doc = ('<?xml version="1.0" encoding="utf-8"?>\n'
           '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
           '<asset><contributor><authoring_tool>autosdv_twin make_body.py</authoring_tool></contributor>'
           '<unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>'
           f'<library_effects>{"".join(lib_fx)}</library_effects>'
           f'<library_materials>{"".join(lib_mat)}</library_materials>'
           f'<library_geometries>{"".join(lib_geo)}</library_geometries>'
           f'<library_visual_scenes><visual_scene id="Scene">{"".join(nodes)}</visual_scene></library_visual_scenes>'
           '<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>\n')
    Path(path).write_text(doc)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out_dir")
    ap.add_argument("--dae", help="also write the car as one COLLADA mesh in base_link")
    a = ap.parse_args()
    out = Path(a.out_dir) / ASSET_ID
    out.mkdir(parents=True, exist_ok=True)

    wpath = out / "wheels.usda"
    if wpath.exists():
        wpath.unlink()
    ws = Usd.Stage.CreateNew(str(wpath))
    UsdGeom.SetStageUpAxis(ws, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(ws, 1.0)
    tris_w = 0
    for name, (_, left) in wheel_positions().items():
        UsdGeom.Xform.Define(ws, f"/{name}")
        Usd.ModelAPI(ws.GetPrimAtPath(f"/{name}")).SetKind("component")
        mats = define_materials(ws, f"/{name}")
        for pn, (p, mn) in wheel(left).items():
            tris_w += write_mesh(ws, f"/{name}/{pn}", p, mn, mats[mn])[0]
    ws.SetDefaultPrim(ws.GetPrimAtPath("/Wheel_Front_Left"))
    ws.GetRootLayer().Save()

    bpath = out / f"{ASSET_ID}.usda"
    if bpath.exists():
        bpath.unlink()
    st = Usd.Stage.CreateNew(str(bpath))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/Vehicle")
    st.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    st.GetRootLayer().customLayerData = {"source": "make_body.py, AutoSDV src/simulation/autosdv_twin/body"}
    mats = define_materials(st, "/Vehicle")
    shift = (X_SHIFT, 0.0, 0.0)
    tris, allp = 0, []
    for name, (p, mn) in build().items():
        segs = name.split("/")
        for i in range(len(segs) - 1):
            UsdGeom.Xform.Define(st, "/Vehicle/" + "/".join(segs[:i + 1]))
        t, pts = write_mesh(st, f"/Vehicle/{name}", p, mn, mats[mn], shift)
        tris += t
        allp.append(pts)
    UsdGeom.Xform.Define(st, "/Vehicle/Wheels")
    wheels = []
    for name, (c, left) in wheel_positions().items():
        pos = c + np.array(shift)
        x = UsdGeom.Xform.Define(st, f"/Vehicle/Wheels/{name}")
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        x.GetPrim().GetReferences().AddReference("./wheels.usda", Sdf.Path(f"/{name}"))
        allp.append(pos + np.array([[s1 * WHEEL_RADIUS, s2 * WHEEL_WIDTH / 2, s3 * WHEEL_RADIUS]
                                    for s1 in (-1, 1) for s2 in (-1, 1) for s3 in (-1, 1)]))
        wheels.append({"name": name, "path": f"/Vehicle/Wheels/{name}", "pos": [round(v, 4) for v in pos],
                       "radius": WHEEL_RADIUS, "width": WHEEL_WIDTH,
                       "max_steer_deg": round(MAX_STEER_DEG, 3) if c[0] > 0 else 0.0,
                       "usd": f"{ASSET_ID}/wheels.usda", "usd_prim": f"/{name}"})
    st.GetRootLayer().Save()
    allp = np.concatenate(allp)
    lo, hi = allp.min(0), allp.max(0)
    height = G["vehicle_height"]
    entry = {
        "id": ASSET_ID,
        "base_type": "car",
        "wheels_n": 4,
        "usd": f"{ASSET_ID}/{ASSET_ID}.usda",
        "prim": "/Vehicle",
        "render_min": [round(v, 4) for v in lo],
        "render_max": [round(v, 4) for v in hi],
        "render_size": [round(v, 4) for v in hi - lo],
        # the footprint vehicle_info declares (front to rear overhang, the overhangs' width, its height)
        "carla_bbox": {"center": [0.0, 0.0, round(height / 2, 4)],
                       "extent": [round((X_FRONT - X_REAR) / 2, 4), round(HALF_W, 4), round(height / 2, 4)]},
        "wheels": wheels,
        "rear_axle_x": round(REAR_AXLE_X, 4),
        "wheel_base": WHEEL_BASE,
        "track": WHEEL_TREAD,
    }
    (out / "entry.json").write_text(json.dumps(entry, indent=1) + "\n")
    print(f"{bpath}: {tris} triangles; wheels.usda: {tris_w} triangles (4 wheels)")
    print(f"render box {np.round(lo, 3).tolist()} .. {np.round(hi, 3).tolist()}, rear_axle_x {REAR_AXLE_X:.4f}")
    if a.dae:
        write_dae(a.dae)
        print(f"{a.dae}: base_link mesh")


if __name__ == "__main__":
    main()
