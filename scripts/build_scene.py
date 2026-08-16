"""Scene setup — roller-conveyor line, T-junction, drop bin.

Scene 2: a 90-degree spawn CURVE at the head feeds a straight ROLLER line
running south past a fenced cell of two KUKA palletizing arms, into a T
junction whose branch runs east and drops boxes into a KLT bin. Two workers +
a forklift dress the aisle; the racks come in from a hand-tuned layout
referenced verbatim (USER_BLOCKS below). Real rigid-body physics, no
animation.

RUN:
  <isaac>\\python.bat scripts/build_scene.py [--out path.usd]
  -> assets/warehouse_tote_flow.usd

OPEN (GUI): File > Open the USD, pick camera RecordCam_Ref, press Play.

GEOMETRY FACTS measured off the assets (probe, not guesses):
  ConveyorBelt_A05  straight roller, 2.00 m long, 1.16 m wide, ROLLER TOP z=0.769
  ConveyorBelt_A02  90-degree curve, the head of the line (main_00)
  ConveyorBelt_A23  T junction: 4.00 m trunk (+X) + a 1.9 m branch (+Y)
  Every piece ships ONE rigid body per belt run, named `Rollers` (the A23 tee has
  two: `Rollers` = trunk, `Rollers_01` = branch). They are already KINEMATIC and
  keep their colliders -- the box rides the rollers themselves.

DRIVE: Isaac's own conveyor OmniGraph. setup_conveyor builds ONE ActionGraph
(/World/Scenario1/ConveyorGraph) holding an OnPlaybackTick plus one
`isaacsim.asset.gen.conveyor.IsaacConveyor` node per roller body, each pointed
at its `Rollers` prim via inputs:conveyorPrim -- the A23 tee gets TWO nodes
(trunk + branch bodies). There are no hand-built slabs and nothing is frozen:
the rollers ARE the driven surface. The curve runs on an angular velocity
(inputs:curved). The tee's branch body is what diverts a box east. ALL
directions are WORLD-frame and the same velocities are baked as
PhysxSurfaceVelocityAPI attrs (localSpace=False) on the bodies at build time
-- see conveyor_graph for why world-frame everywhere is the only convention
where the baked parse, the node's per-tick writes, and a user live-editing
inputs:direction in the GUI all mean the same thing. The bake also means the
scene plays correctly where the conveyor extension is off.
(The conveyor extension is disabled by default; the boot below enables it.)

ASCII-only prints (Isaac stdout is cp1252 — non-ASCII kills the app silently).
"""
import argparse
import json
import math
import os

# ---------------------------------------------------------------------------
# paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(SCRIPT_DIR)
# Machine-local roots. Required: asset_root, from config/assets.json (copy
# config/assets.example.json and edit). Optional: extra_assets_root (the
# hand-tuned RackRows block, baked walk clips) and nvidia_packs_root (the
# ArchViz sample packs the fences, arm shell and cartons reference). Both
# may be empty -- every consumer checks os.path.exists first, so a clean
# checkout still builds the core running line.
with open(os.path.join(PROJ, "config", "assets.json"), encoding="utf-8") as _f:
    _CFG = json.load(_f)
EXTRA_ASSETS = _CFG.get("extra_assets_root", "") or os.environ.get("TF_EXTRA_ASSETS", "")
NV_PACKS = _CFG.get("nvidia_packs_root", "") or os.environ.get("TF_NVIDIA_ASSETS", "")

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=None)
ARGS = ap.parse_args()
SCENE_OUT = ARGS.out or os.path.join(PROJ, "assets", "warehouse_tote_flow.usd")

ASSET_ROOT = _CFG["asset_root"]
IND_ROOT = NV_PACKS + "/Industrial/Assets/ArchVis/Industrial"

WAREHOUSE_USD = ASSET_ROOT + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
CONV_DIR = ASSET_ROOT + "/Isaac/Props/Conveyors"
STRAIGHT_USD = CONV_DIR + "/ConveyorBelt_A05.usd"      # 2.00 m straight roller
TJUNCTION_USD = CONV_DIR + "/ConveyorBelt_A23.usd"     # 4.00 m trunk + branch

# ARM: the GRAFTED KUKA (see kuka_graft.py) — the welding cell's own robot body
# on Isaac's KR210 articulation. No Isaac library arm can look like the user's
# robot (the best of them, the Kawasaki RS080N that used to stand here, ships 14
# meshes / 4 materials against the cell robot's 178 / 34) and no sample robot can
# move (zero joints), so the two are married. Reach 2.45 m — still clears the
# belt (1.40 m) and the pallet (1.53 m).
ARM_REACH = 2.45          # kuka_graft.REACH (kept literal: that module needs
                          # pxr, so it cannot be imported until Isaac has booted)

# ARM BASE: the pedestal from the user's own welding cell (they pointed at
# .../RobotCell/asset/line/asset/root/RobotPedestal_U20__U23_10/geometry).
# The pack is in MILLIMETRES: 1020 x 1020 x 970 raw -> scale 0.001.
WELDING_USD = (NV_PACKS +
               "/USDExplorerSample/Usd_Explorer/Samples/Examples/2023_2/Factory/"
               "Source_Files/Visual_Components/raw_cell_w_animation.usd")
PEDESTAL_PRIM = "/World/welding_cell/root/RobotPedestal_U20__U23_10/geometry"
PEDESTAL_SCALE = 0.001
PEDESTAL_TOP_Z = 0.97          # measured: arms mount here

# DROP BIN: the KLT bin the user asked for, scaled 5x (0.99 x 1.49 x 0.73 m).
# 5x is the ceiling: any bigger and the rim clears the roller top (0.769 m) and
# boxes hit the wall instead of dropping in.
KLT_USD = ASSET_ROOT + "/Isaac/Props/KLT_Bin/small_KLT.usd"
KLT_SCALE = 5.0
KLT_RAW = (0.198, 0.297, 0.146)     # measured

# PACKAGES: real cardboard boxes (Industrial pack, CENTIMETRES -> scale 0.01)
CARDBOARD = IND_ROOT + "/Containers/Cardboard"
BOX_SCALE = 0.01
# name -> real size in metres, measured
BOXES = {
    "Cardbox_A1": (0.699, 0.521, 0.510),
    "Cardbox_B1": (0.510, 0.510, 0.521),
    "Cardbox_C1": (0.513, 0.511, 0.260),
    "Cardbox_B2": (0.510, 0.510, 0.521),
    "Cardbox_C2": (0.513, 0.511, 0.260),
}
# what rides the belt: LOW cartons only (0.26 m, top at z=1.03). The stopper
# arm hangs at z 0.91-1.04: a low box grazes its bottom edge and glances into
# the branch every time, while a tall box (0.52 m, arm hits it mid-face)
# wedges against the arm tip at the mouth often enough to jam the line
# (measured: bistable across identical runs -- useless for filming). The tall
# A/B cartons still appear in the scene, stacked on the cell pallets.
LINE_BOXES = ["Cardbox_C1", "Cardbox_C2", "Cardbox_C1", "Cardbox_C2",
              "Cardbox_C1"]          # one per start slot
PKG_MASS = 4.0
PEOPLE_DIR = ASSET_ROOT + "/Isaac/People/Characters"
WORKERS = ["male_adult_construction_01_new", "male_adult_construction_05_new"]

# STOPPER at the T: the two barriers the user hand-placed in the GUI to
# replace the fence-panel diverter. A short
# safety post with a fold-down arm folded across the branch mouth -- the arm is
# what the boxes strike, so it becomes the heatmap target (35k welded verts,
# far denser than the fence panel it replaces).
BARRIERS = (ASSET_ROOT + "/NVIDIA/Assets/DigitalTwin/Assets/Warehouse/Safety/"
            "Barriers")
BOLLARD_POST_USD = BARRIERS + "/SafetyBollard_A/SafetyBollard_A06_01.usd"
BOLLARD_ARM_USD = (BARRIERS
                   + "/FoldDownSafetyBollard_A/FoldDownSafetyBollard_A04_01.usd")
BOLLARD_SCALE = 0.01           # the DigitalTwin barriers are authored in cm

# cell fence: the GenericFence panels from the welding-cell sample (a prim
# reference, mm units), 1.60 m wide x 2.50 m tall, with the sample's own door
# leaf for the way in. Replaces the low MetalFencing panels.
GENFENCE_PRIM = "/World/welding_cell/root/GenericFence_U20__U23_10"   # 1.60 m
GENDOOR_PRIM = "/World/welding_cell/root/GenericDoorLeft"             # 1.00 m
GENFENCE_W = 1.60
GENFENCE_SCALE = 0.001

# drop target + haulage: the cell robots fill a pallet (a KLT bin on an
# Isaac pallet, the same container that catches boxes at the T), and a forklift
# stands ready to carry it to the racks -- the story the reference photo tells.
PALLET_DROP_USD = ASSET_ROOT + "/Isaac/Props/Pallet/pallet.usd"
FORKLIFT_USD = ASSET_ROOT + "/Isaac/Props/Forklift/forklift.usd"

SCENARIO = "/World/Scenario1"

# ---------------------------------------------------------------------------
# LAYOUT (world metres). The line runs NORTH -> SOUTH (-Y) down the west bay.
#
#        y=-1   [spawn curve + hood]  <- head of line, packages enter here
#          |    ||
#          |    ||   (fenced robot cell, line passes THROUGH it N->S)
#          |    ||
#        y=-18  TT====== branch (+X) ======> [BIN]      <- T junction + drop bin
#                                             ^workers patrol the aisle east
# ---------------------------------------------------------------------------
# Position BAKED from the layout the user dragged in the GUI. Verified
# clash-free: the only warehouse geometry it crosses is Section0, the ROOF
# TRUSSES — their bbox reaches the floor solely because the support posts do
# (posts sit at x=-26.4 and -17.3; the line passes between them, under the
# beams).
# MOVED EAST out of the west bay (was -24.92, hard against the west wall at
# x=-26.33). probe_layout.py swept 13 candidates against every blocker whose
# bbox BOTTOM sits inside the 2.60 m working volume: the pillar assemblies
# fence off bays at x[-26.43,-17.33] and again east of about -4.3, leaving a
# clear span roughly x[-17.3,-4.3]. The run needs 7.30 m (1.60 west for the
# hood + curve, 5.70 east for the cell fence and the bin mouth), so -14.00
# sits in open floor with margin on both sides. Verdict CLEAR.
LINE_X = -14.00           # centreline of the main conveyor
LINE_HEAD_Y = -0.97       # north end (packages enter)
# The T junction sits SOUTH, near the south wall: it fills the empty south floor
# and stretches the palletizing run.
T_Y = -18.00              # A23 origin: trunk spans [T_Y-4, T_Y]
# No pieces past the T: the user deleted the tail straight in the GUI (16/07)
# -- it poked out of the warehouse and every box diverts east anyway.
HEAD_PIECES = 9           # straight A05 slots from the head to the T; slot 0 is
                          # the spawn CURVE (main_00), so the straights are 1..8

ROLLER_TOP_Z = 0.769      # measured: package rides at this height
BELT_SPEED = 1.2          # m/s (2.0 of the old loop was too quick to film)

# EVERYTHING below is expressed RELATIVE to LINE_X / LINE_HEAD_Y / T_Y. The user
# re-positions the cell in the GUI between rounds; absolute coordinates here
# drifted out of sync with it twice, so the three anchors above are now the only
# numbers to touch.
# A23 (rot -90) puts its branch rollers at LINE_X + [0.51 .. 2.44].
BRANCH_Y = T_Y - 2.10                  # branch centreline
BRANCH_PIECE_X = LINE_X + 2.44         # where the extra A05 starts
BRANCH_END_X = BRANCH_PIECE_X + 2.0    # last roller -> boxes drop into the bin

# STOPPER: the SafetyBollard post + fold-down ARM, at the BRANCH MOUTH --
# the user dragged the pair back up here in the GUI on 16/07 (baked
# verbatim, adapt not re-place). The arm across the mouth is the visual
# story beat (boxes strike it and glance east) and the IMPACT HEATMAP target
# (35k welded verts; triangle-mesh collider so the hit lands on the real
# bar, low-friction material so arrivals glance, not slam-stick). The
# transfer strip under the seam is what keeps a striking box from beaching.
STOPPER_POST = {"pos": (LINE_X + 0.63, BRANCH_Y - 0.60), "yaw": 0.0}
STOPPER_ARM = {"pos": (LINE_X + 0.20, BRANCH_Y - 0.37), "z": 0.91,
               "yaw": 245.0}

# drop bin: KLT bin, long side along the branch so a box rolling off the end
# lands inside it. Pushed hard against the last roller (+0.65 m, not +1.20 as
# the old 2.4 m box needed) — a box leaves the belt at 1.2 m/s and must not
# overshoot the far wall or fall short of the near one.
BIN = {"pos": (BRANCH_END_X + 0.65, BRANCH_Y), "yaw": 90.0, "scale": KLT_SCALE}

# ---------------------------------------------------------------------------
# KR210 PAINT
# The asset is NOT missing materials. It carries exactly one — an MDL OmniPBR
# at /kuka_kr210/Looks/material_C0C0C0 with diffuse_color_constant = 0.753 grey
# (#C0C0C0) — bound to all 7 visual meshes (base_link + link_1..link_6). Isaac
# library robots ship unbranded on purpose.
#
# The welding cell's orange CANNOT be borrowed: its 43 materials live in
# /World/Looks, outside the robot prim, so referencing a sub-prim drops every
# binding ("refers to a path outside the scope of the reference. Ignoring").
#
# So: author our own MDL OmniPBR materials — MDL, not UsdPreviewSurface, or the
# arm reads as flat plastic — and split them per link the way a real KR210 is
# finished: orange body, dark cast base and wrist.
# We now reuse the WELDING CELL'S OWN MATERIALS (verified A/B: they reference in
# fine and bind to the KR210's meshes). They do not travel automatically because
# they live in that file's /World/Looks — a SIBLING of /World/welding_cell — so
# a sub-prim reference leaves them behind. Pull each one across explicitly and
# the imported robot matches the user's cell exactly.
NV_LOOKS = "/World/Looks"                 # inside raw_cell_w_animation.usd
# The RS080N arrives already finished (white body / black joints / grey), so we
# do NOT repaint it — overriding would flatten it back to one colour, which is
# exactly the problem we just moved away from.
ARM_PAINT = {}
PEDESTAL_MAT = "grey"                     # the pedestal DOES lose its material

# ROBOT CELL — TWO KUKA palletizing arms on the east side of the main
# line, north of the T (the user asked for two). Each fills a pallet beside it;
# a forklift runs them to the racks. The line keeps its T + drop bin + workers
# at the south, and packages enter from a black-box hood at the north around a
# 90-degree curve (the spawn is hidden inside the hood).
PALLET_ARM_X = LINE_X + 1.40           # arms one working-distance east of belt
PALLET_STACK_X = LINE_X + 3.15         # pallets east of the arms (both in reach)
# ONE arm, not two. Two arms need 9.2 m of fenced cell between them, and that
# wall of fence panels is what hides the conveyor from every camera angle that
# also shows the tee. The brief here is the TOTE FLOW, so the cell shrinks to
# the single station that dresses the line and stops occluding it.
CELL_ROBOTS = [(PALLET_ARM_X, LINE_HEAD_Y - 4.80, 180.0, "R1")]

# the fenced cell now wraps ONE arm. y0 clears R1's 2.45 m reach (-4.80 - 2.45
# = -7.25) with a little margin; the old -14.20 was there only to reach R3.
CELL = {"x0": LINE_X - 1.10, "x1": LINE_X + 4.40,
        "y0": LINE_HEAD_Y - 7.60, "y1": LINE_HEAD_Y - 2.80}
# entrance re-centred on the SHORT cell: the old -7.10 now sits 0.5 m from the
# south wall, so a 2.40 m gap would run off the end of the panel run.
CELL_ENTRANCE_Y = LINE_HEAD_Y - 5.20            # gap in the east wall
CELL_ENTRANCE_W = 2.40                          # wide enough for the forklift
FORKLIFT = {"pos": (LINE_X + 6.30, CELL_ENTRANCE_Y), "yaw": -90.0}

# SPAWN at the north: a 90-degree curve (A02) turns the line toward the west
# wall -- the user deleted the spare straight piece so the curve IS the head now.
# CURVE_POS is BAKED from the user's own placement (prim `ConveyorTrack` in
# the saved GUI layout): from there the curve's exit port sits ON the line
# and its south edge MEETS main_01's north edge (measured: -2.966 vs -2.970),
# so a box rolls straight off the curve onto the first straight. The old
# scripted spot (LINE_X, LINE_HEAD_Y + 0.6) left a 1.0 m hole before main_01 --
# harmless under the old slab drive, fatal now that the rollers do the driving.
SPAWN_CURVE_USD = CONV_DIR + "/ConveyorBelt_A02.usd"   # a 90-deg belt curve
CURVE_POS = (LINE_X - 1.4951, LINE_HEAD_Y - 0.4003)
# hood = a five-sided black enclosure (open south) over the curve mouth only,
# exactly where the user left it in the backup
# Hood lengthened south, 1.9 -> 3.2 m, so it also covers the RESPAWN point.
# Measured (probe_belt.py) along x=LINE_X:
#     y -0.92 .. -2.94   main_00, the CURVE: lin=(0,0,0), ang=(0,0,-44.4)
#     y -2.97 .. -4.96   main_01, the first straight: lin=(0,-1.20,0)
# The curve has NO linear drive, so a tote respawned on it just sits there
# (measured: parked at (-13.91,-2.18) for 1500 frames). The respawn therefore
# has to be at y <= -3.0, and the old 1.9 m hood only reached -2.25 -- there
# was no point that was both driven and hidden. At 3.2 m it spans
# y[-3.55,-0.35] and the tote reappears out of sight.
# South edge -3.55 still clears the cell's north fence at LINE_HEAD_Y-2.80.
SPAWN_HOOD = {"center": (LINE_X - 0.4804, LINE_HEAD_Y - 0.98),
              "size": (1.8, 3.2, 1.7), "wall": 0.05}

# (workers no longer stand still -- see WALK_LOOP below: they patrol the open
# aisle east of the cell)

# ---------------------------------------------------------------------------
# Blocks lifted VERBATIM from layouts the user hand-tuned in the GUI. Referenced
# whole — nothing inside them is touched.
#   RackRows   the loaded pallet racks, from scenario1_scene_USER_EDIT_2343.usd.
# ---------------------------------------------------------------------------
EDIT_SCENE = os.path.join(EXTRA_ASSETS,
                          "scenario1_scene_USER_EDIT_2343.usd").replace("\\", "/")
USER_BLOCKS = [
    {"name": "RackRows", "usd": EDIT_SCENE,
     "prim": "/World/RackRows", "shift": (0.0, 0.0)},
]

# packages riding the line at t=0 (they start flowing the moment you hit Play).
# THREE boxes at 7.2 m, not five at 2.5: the divert at the tee dwells ~5-6 s
# per box, so the feed must not beat that rhythm -- a box arriving while the
# mouth is busy presses into the one under divert and the pair wedges against
# the stopper (measured at every tighter spacing tried, even with carton
# friction). 7.2 m at belt speed is ~6 s between arrivals. In steady state the
# recycler is self-limiting (it can only respawn what already diverted), so
# only this t=0 batch needs the margin.
# 5 totes at 4.0 m, not 3 at 7.2 m. The 7.2 m figure was measured for the
# 0.51 m CARTONS, whose width against the stopper arm is what wedged the line.
# A tote is 0.40 m across the belt, and the first render proved the tighter
# case empirically: the recycler fed at 6.05 / 6.9 / 4.6 / 3.3 s intervals and
# nothing jammed, so 3.3 s is a demonstrated-safe headway. 4.0 m at 1.2 m/s IS
# that 3.3 s. At 7.2 m the 30 s orbit spent whole seconds looking at an empty
# conveyor, which reads as a line that is not running.
PKG_START_Y = [LINE_HEAD_Y - 0.6 - 4.0 * i for i in range(5)]
# Spare totes for the feed. A 30 s take at one tote every ~4.7 s needs ~7;
# 12 leaves margin for a longer clip without another rebuild. Parked well
# above the scene and kinematic, so they cost nothing until they are dropped
# in and they carry no momentum when they are.
POOL_SIZE = 12
PARK_XY = (LINE_X - 6.0, LINE_HEAD_Y)
PARK_Z = 150.0
# the FIRST package rides in on the spawn CURVE's north arm, inside the hood
# (baked from pkg_00 in the saved GUI layout); the rest start on the line.
PKG0_SPAWN_X = LINE_X - 0.986

# 3DGS PACKAGE: the user's BagPack splat (gaussian-splatting extension output:
# Visual_Splat NuRec volume + Geometry_Mesh reconstruction in one usdz).
# It rides the line as pkg_00 -- the star of the output video. A splat has no
# collision, and convex-hulling a scanned mesh is the PhysX crash gotcha, so
# the physics is an INVISIBLE box proxy sized from the measured mesh bbox.
# TOTE: every package on this line is a black industrial KLT tote -- the
# container this line is specced for. Same asset as the drop bin, different
# scale. The raw bin is 0.198 x 0.297 x 0.146, i.e. ALREADY the 400:600 ratio
# of a real Euro/KLT tote, and its LONG axis is +Y = the direction of travel
# down the line. Normalising the long side to 0.60 therefore lands the
# CROSS-BELT width at 0.40 -- narrower than the 0.51 carton that was measured
# to divert cleanly at the tee, so the divert margin only improves.
TOTE_USD = KLT_USD
TOTE_LONG = 0.60           # -> 0.40 (cross-belt) x 0.60 (travel) x 0.295 tall
TOTE_COLOR = (0.045, 0.045, 0.05)   # stock KLT is PINK -- see setup_bin
TOTE_ROUGH = 0.55          # injection-moulded PP: matte, not glossy

# cameras: the 4 corner cams sit at the four corners of the ROBOT AREA (the
# fenced cell), not the warehouse -- each just outside its cell corner, high
# enough to shoot over the 2.5 m fence, aimed at the cell centre. The west
# pair is clamped inside the warehouse west wall (x=-26.3). The 2 record cams
# stay aimed relative to the line.
_MID = (LINE_X + 1.0, (LINE_HEAD_Y + T_Y) / 2)          # middle of the run
_CELL_C = ((CELL["x0"] + CELL["x1"]) / 2, (CELL["y0"] + CELL["y1"]) / 2)
_CAM_M, _CAM_Z = 2.6, 5.2            # margin outside the cell / camera height
_CAM_W = max(CELL["x0"] - _CAM_M, -25.9)      # west cams: stay inside the wall
CAMERAS = {
    "CornerCam_NW": {"eye": (_CAM_W, CELL["y1"] + _CAM_M, _CAM_Z),
                     "tgt": (_CELL_C[0], _CELL_C[1], 1.0), "f": 18.0},
    "CornerCam_NE": {"eye": (CELL["x1"] + _CAM_M, CELL["y1"] + _CAM_M, _CAM_Z),
                     "tgt": (_CELL_C[0], _CELL_C[1], 1.0), "f": 18.0},
    "CornerCam_SW": {"eye": (_CAM_W, CELL["y0"] - _CAM_M, _CAM_Z),
                     "tgt": (_CELL_C[0], _CELL_C[1], 1.0), "f": 18.0},
    "CornerCam_SE": {"eye": (CELL["x1"] + _CAM_M, CELL["y0"] - _CAM_M, _CAM_Z),
                     "tgt": (_CELL_C[0], _CELL_C[1], 1.0), "f": 18.0},
    "RecordCam_Full": {"eye": (LINE_X + 13.0, T_Y - 10.0, 11.0),
                       "tgt": (_MID[0], _MID[1], 1.0), "f": 18.0},
    # the reference-photo angle: 3/4 view down the line, T + bin + workers in frame
    "RecordCam_Ref": {"eye": (LINE_X + 6.5, T_Y + 8.0, 4.6),
                      "tgt": (LINE_X, T_Y - 2.1, 0.9), "f": 24.0},
    # --- CINEMATIC set (video output) -- same framings the shot builders in
    # shots/scenario1_conveyor.py fly, parked here as statics so the GUI shows
    # exactly what each clip will frame ---
    # low 3/4 on the T: boxes strike the arm and glance into the branch
    "CineCam_Divert": {"eye": (LINE_X + 3.1, T_Y - 4.6, 2.1),
                       "tgt": (LINE_X + 0.6, BRANCH_Y + 0.3, 0.85), "f": 32.0},
    # low, looking up the line: boxes come at the lens, arms on the right
    "CineCam_LineLow": {"eye": (LINE_X + 1.6, T_Y + 0.6, 1.15),
                        "tgt": (LINE_X - 0.1, LINE_HEAD_Y - 7.0, 0.95),
                        "f": 30.0},
    # crane over the fenced cell, line running through frame
    "CineCam_CellCrane": {"eye": (LINE_X + 8.0, LINE_HEAD_Y - 1.2, 6.8),
                          "tgt": (LINE_X + 1.0, LINE_HEAD_Y - 9.8, 0.8),
                          "f": 22.0},
    # tight on the drop bin: boxes fly off the branch and land
    "CineCam_BinDrop": {"eye": (BIN["pos"][0] + 1.9, BIN["pos"][1] - 2.2, 1.9),
                        "tgt": (BIN["pos"][0] - 0.1, BIN["pos"][1] + 0.1, 0.75),
                        "f": 35.0},
    # eye-level down the west rack aisle: a worker walks through the shot
    "CineCam_Aisle": {"eye": (-2.68, -21.5, 1.75),
                      "tgt": (-2.68, -8.0, 1.35), "f": 28.0},
}

FPS = 60.0

# WORKER PATROL: the two workers WALK one big closed loop that covers the
# whole floor -- west aisle by the cell, the north corridor, DOWN one rack
# aisle, along the south corridor by the wall, back UP the other rack aisle
# (rack rows measured: 3 solid columns x[-5.9,-3.9]/[-1.5,0.6]/[2.7,4.8]
# spanning y[-22,-6], so the walkable aisles are x=-2.68 and x=1.63; the
# corridors are y=-4 north of the racks and y=-22.6 by the south wall).
# Clear of the forklift (-18.6,-8.1) and the drop bin (-19.8,-20.1).
# Scripted nav: an in-place walk clip supplies the legs, time-sampled root
# motion on the wrapper walks the loop -- no anim-graph / navmesh machinery
# to break in a headless run. One lap per timeline loop, half a lap apart.
# The walk clips are RETARGETED + TILED per character by bake_walk_clip.py.
# The stock Isaac clips are authored on the BIPED rig; the dressed characters
# use a Reallusion rig and silently IGNORE a biped clip (T-pose -- measured;
# the old 'idle anim bound' never actually animated either). UsdSkel also
# does not loop a clip, so the bake tiles the cycle over the whole timeline.
# Run bake_walk_clip.py once if the files are missing.
WALK_CLIP_DIR = os.path.join(EXTRA_ASSETS, "people")
WALK_LOOP = [(-16.0, -4.0),      # by the cell, north corridor
             (-2.68, -4.0),      # across the top of the racks
             (-2.68, -22.6),     # DOWN the west rack aisle
             (1.63, -22.6),      # along the south wall
             (1.63, -4.0)]       # UP the east rack aisle, then back west
WALK_SPEED = 1.25          # m/s, near the clip's cadence so feet do not skate
WALK_YAW_OFF = 90.0        # the Reallusion rig walks along its -Y, not +X
                           # (measured twice: 0 has them facing 90 deg off,
                           # -90 has them walking BACKWARD -- +90 aligns the
                           # face with the direction of travel)
_WALK_PERIM = sum(
    math.hypot(WALK_LOOP[(i + 1) % len(WALK_LOOP)][0] - p[0],
               WALK_LOOP[(i + 1) % len(WALK_LOOP)][1] - p[1])
    for i, p in enumerate(WALK_LOOP))
# one full patrol lap per timeline pass -> looping playback is seamless
END_FRAME = int(_WALK_PERIM / WALK_SPEED * FPS)


def log(msg):
    print(f"[scene1] {msg}", flush=True)


# ---------------------------------------------------------------------------
# boot
# ---------------------------------------------------------------------------
log("booting Isaac Sim (headless)...")
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({
    "headless": True,
    "width": 1280, "height": 720,
    "renderer": "RaytracedLighting",
    "extra_args": [
        "--/rtx/post/histogram/enabled=false",
        f"--/persistent/isaac/asset_root/default={ASSET_ROOT}",
    ],
})
log("SimulationApp ready")

import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux,  # noqa: E402
                 UsdPhysics, UsdShade, Vt)

import kuka_graft  # noqa: E402  (imports pxr -- only valid after the boot above)

_mgr = omni.kit.app.get_app().get_extension_manager()
_mgr.set_extension_enabled_immediate("omni.kit.scripting", True)
# The conveyor node lives in an extension that is OFF by default -- without this
# the IsaacConveyor node type is not registered and og.Controller.edit() below
# raises. (Probed: isaacsim.asset.gen.conveyor, enabled=False on a stock boot.)
_mgr.set_extension_enabled_immediate("isaacsim.asset.gen.conveyor", True)
for _ in range(10):
    app.update()

import omni.graph.core as og  # noqa: E402  (needs the extension above)


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def bbox_of(prim):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    return rng.GetMin(), rng.GetMax()


def place(stage, path, usd, pos, yaw=0.0, z=0.0, scale=1.0, prim=None):
    """Reference `usd` (optionally a single prim inside it) under a clean
    wrapper Xform we own, and pose the wrapper. (Referencing straight onto the
    target prim composes the asset's own xformOps onto it and XformCommonAPI
    then silently refuses to write.)"""
    UsdGeom.Xform.Define(stage, path)
    refs = stage.DefinePrim(f"{path}/asset", "Xform").GetReferences()
    refs.AddReference(usd, prim) if prim else refs.AddReference(usd)
    xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
    xf.SetTranslate(Gf.Vec3d(pos[0], pos[1], z))
    xf.SetRotate(Gf.Vec3f(0.0, 0.0, float(yaw)))
    if scale != 1.0:
        xf.SetScale(Gf.Vec3f(scale, scale, scale))
    return stage.GetPrimAtPath(path)


def place_subprim(stage, path, usd, prim_path, pos, yaw=0.0, scale=1.0):
    """Like place(), but for a SUB-PRIM that carries its own in-file placement
    (the GenericFence panels live at fixed spots inside the welding cell). A
    reference brings that in-file transform along on /asset, which would teleport
    every panel back into the cell's footprint -- so it is cleared, leaving the
    raw geometry at the origin for the wrapper to pose."""
    p = place(stage, path, usd, pos, yaw, scale=scale, prim=prim_path)
    for _ in range(3):
        app.update()
    UsdGeom.Xformable(stage.GetPrimAtPath(f"{path}/asset")).ClearXformOpOrder()
    for _ in range(2):
        app.update()
    return p


def roller_bodies(prim):
    """Every rigid body under a placed conveyor piece -- i.e. its belt run(s).

    Probed: each ConveyorBelt_A* ships exactly one `Rollers` body per run, already
    KINEMATIC and keeping its collider, and the A23 tee ships two (`Rollers` =
    trunk, `Rollers_01` = branch). Nothing is frozen or collision-disabled here:
    these bodies ARE the driven surface, and the conveyor node applies the
    surface velocity to them at run time."""
    return [p for p in Usd.PrimRange(prim)
            if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def colored_box(stage, path, center, size, color, yaw=0.0, collider=True,
                dynamic=False, mass=1.0):
    """A plain Cube — never a referenced prop. Props go magenta headless (G10)
    and convex-hulling a scanned mesh kills the app (G-crash)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    if yaw:
        xf.AddRotateZOp().Set(float(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    if collider:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(float(mass))
    return cube.GetPrim()


def _slick_material(stage):
    """One low-friction, no-bounce physics material, reused by every barrier a
    box glances off. A stock-friction surface parks boxes against it (measured:
    the divert dropped from 5/5 to 3/5); this lets them slide on."""
    path = "/World/Looks/rail_slick"
    if stage.GetPrimAtPath(path).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(path))
    mat = UsdShade.Material.Define(stage, path)
    m = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    m.CreateStaticFrictionAttr(0.08)
    m.CreateDynamicFrictionAttr(0.05)
    m.CreateRestitutionAttr(0.0)
    return mat


def stopper(stage, path):
    """The two safety barriers the user placed at the branch mouth, replacing the
    fence-panel diverter, plus the physics that makes them work as a deflector
    and as the heatmap's target.

    A vertical SafetyBollard post carries a FoldDownSafetyBollard arm folded
    across the mouth. Both are baked at the user's own positions (STOPPER_POST /
    STOPPER_ARM). Three things each of them needs, none about looks:

      * de-instanced -- an instanceable prim exposes read-only proxies, so the
        heatmap's primvars:displayColor writes are silently dropped and it never
        colours (gotcha G4);
      * a TRIANGLE-MESH collider (plain CollisionAPI, no approximation) -- a
        convex hull would report the hit on a flat slab, not the real bar, and
        the heat would land on a surface you cannot see;
      * the low-friction physics material, so a box glances off toward the
        branch rather than stopping dead against it.

    The ARM (the bar the boxes actually strike, 35k welded verts) is the heat
    target; the post is dressing that also happens to be solid.
    """
    UsdGeom.Xform.Define(stage, path)
    mat = _slick_material(stage)

    def barrier(name, usd, pos, yaw, z=0.0):
        place(stage, f"{path}/{name}", usd, pos, yaw, z=z, scale=BOLLARD_SCALE)
        for _ in range(4):
            app.update()
        for p in Usd.PrimRange(stage.GetPrimAtPath(f"{path}/{name}"),
                               Usd.TraverseInstanceProxies()):
            if p.IsInstance():
                p.SetInstanceable(False)
        for _ in range(3):
            app.update()
        meshes = verts = 0
        for p in Usd.PrimRange(stage.GetPrimAtPath(f"{path}/{name}")):
            if not p.IsA(UsdGeom.Mesh):
                continue
            UsdPhysics.CollisionAPI.Apply(p)         # triangle mesh: exact
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            meshes += 1
            verts += len(UsdGeom.Mesh(p).GetPointsAttr().Get() or [])
        return meshes, verts

    barrier("post", BOLLARD_POST_USD, STOPPER_POST["pos"], STOPPER_POST["yaw"])
    am, av = barrier("arm", BOLLARD_ARM_USD, STOPPER_ARM["pos"],
                     STOPPER_ARM["yaw"], z=STOPPER_ARM["z"])
    log(f"  stopper: SafetyBollard post + fold-down arm at the branch mouth "
        f"(user's placement); arm is the heat target, {am} mesh / {av} verts")
    return stage.GetPrimAtPath(path)


def conveyor_graph(stage, runs):
    """Isaac's own conveyor drive: ONE ActionGraph, one IsaacConveyor node per
    belt run, each pointed at that run's `Rollers` rigid body.

    This is what CreateConveyorBelt builds (probed), minus its per-piece graph
    and its Velocity-variable indirection: a graph per piece would be a dozen
    graphs to tick, and the variable only exists so the UI can scrub speed. Here
    one graph drives every run and the speed is set straight on the node.

    `runs` = [{prim, dir, vel, curved, label}]. `dir` is in WORLD frame and
    the baked attrs say localSpace=False to match. World, not the body-local
    frame NVIDIA's track_types.json uses, because PhysX applies any MID-SIM
    write of these attrs in world space regardless of the local flag
    (measured): a GUI user live-tuning inputs:direction, and the node's own
    per-tick writes, therefore always speak world -- authoring the baked
    parse-time values in world too makes every path mean the same thing.
    The cost is that directions must be re-derived if a piece is re-yawed;
    the layout is baked, so that is fine. Curved runs take the ROTATION AXIS
    instead, scaled so dir*vel is the angular speed in deg/s -- NVIDIA ships
    A02 as (0,0,-37): omega = v/r for its ~1.55 m mid-belt radius (measured:
    -44.4 deg/s carries a box at 1.2 m/s around the bend); a Z axis is the
    same in either frame.

    The velocity is ALSO baked onto each body as PhysxSurfaceVelocityAPI
    attrs -- the very attrs the node writes at run time. It must be: the
    attrs have to exist when the body is PARSED (sim start) or the node's
    first mid-sim write creates them with mismatched frame semantics
    (measured: boxes slid into the side rail instead of down the line). With
    the values pre-baked the parse gets them right, the node's tick writes
    re-state the same values, and the belts even run if the conveyor
    extension is off at play time.
    """
    gpath = f"{SCENARIO}/ConveyorGraph"
    keys = og.Controller.Keys
    nodes = [("OnTick", "omni.graph.action.OnPlaybackTick")]
    values, connects = [], []
    for i, r in enumerate(runs):
        n = f"Conveyor_{i:02d}"
        nodes.append((n, "isaacsim.asset.gen.conveyor.IsaacConveyor"))
        connects += [("OnTick.outputs:tick", f"{n}.inputs:onStep"),
                     ("OnTick.outputs:deltaSeconds", f"{n}.inputs:delta")]
        values += [(f"{n}.inputs:velocity", float(r["vel"])),
                   (f"{n}.inputs:direction", Gf.Vec3f(*r["dir"])),
                   (f"{n}.inputs:enabled", True),
                   (f"{n}.inputs:curved", bool(r.get("curved", False)))]
    og.Controller.edit({"graph_path": gpath, "evaluator_name": "execution"},
                       {keys.CREATE_NODES: nodes,
                        keys.SET_VALUES: values,
                        keys.CONNECT: connects})
    for _ in range(4):
        app.update()

    # conveyorPrim is a RELATIONSHIP -- og.Controller cannot set it, so target it
    # on the USD prim the way CreateConveyorBelt does.
    for i, r in enumerate(runs):
        node = stage.GetPrimAtPath(f"{gpath}/Conveyor_{i:02d}")
        rel = (node.GetRelationship("inputs:conveyorPrim")
               or node.CreateRelationship("inputs:conveyorPrim"))
        rel.SetTargets([r["prim"].GetPath()])

        # bake the velocity the node will write, in float32 so the node's own
        # first write is bit-identical and fires no mid-sim change event
        vel = Gf.Vec3f(*r["dir"]) * float(r["vel"])
        api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(r["prim"])
        api.CreateSurfaceVelocityEnabledAttr(True)
        api.CreateSurfaceVelocityLocalSpaceAttr(False)   # WORLD, see docstring
        if r.get("curved"):
            api.CreateSurfaceAngularVelocityAttr(vel)
            api.CreateSurfaceVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        else:
            api.CreateSurfaceVelocityAttr(vel)
            api.CreateSurfaceAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        log(f"  conveyor node {i:02d} -> {r['label']:12s} "
            f"dir={tuple(r['dir'])} vel={r['vel']:.2f}"
            f"{' curved' if r.get('curved') else ''}  baked={tuple(vel)}")
    for _ in range(4):
        app.update()
    log(f"  {len(runs)} IsaacConveyor nodes under {gpath} (one OnPlaybackTick), "
        f"surface velocities baked on the bodies")
    return stage.GetPrimAtPath(gpath)


def omni_pbr(stage, mat_name, color, rough=0.45, metal=0.0):
    """An MDL OmniPBR material — the same shader the KR210 already uses.

    A UsdPreviewSurface here is what made the arm read 'flat plastic': no
    metallic response, no MDL reflection. OmniPBR gives the painted-steel look.
    """
    path = f"/World/Looks/{mat_name}"
    if stage.GetPrimAtPath(path).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(path))
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(rough))
    sh.CreateInput("metallic_constant",
                   Sdf.ValueTypeNames.Float).Set(float(metal))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def bind_material(stage, root, mat, links=None):
    """Bind `mat` to the visual meshes under `root` (optionally only those whose
    parent link name is in `links`). Collision proxies are skipped — they are
    invisible and binding them just bloats the stage.

    De-instances first: an instanceable prim exposes its children as read-only
    proxies, so Bind() lands on a proxy and is silently dropped (this is what
    left the welding-cell pedestal white — 0 meshes bound)."""
    for p in Usd.PrimRange(stage.GetPrimAtPath(root),
                           Usd.TraverseInstanceProxies()):
        if p.IsInstance():
            p.SetInstanceable(False)
    for _ in range(3):
        app.update()

    n = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not p.IsA(UsdGeom.Mesh) or "collision" in p.GetName().lower():
            continue
        if links is not None and p.GetParent().GetName() not in links:
            continue
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            mat, UsdShade.Tokens.strongerThanDescendants)
        n += 1
    return n


def nv_material(stage, name):
    """Reference ONE material out of the welding cell's /World/Looks.

    This is the whole answer to 'can we carry NVIDIA's material over': yes, but
    it must be referenced EXPLICITLY. A sub-prim reference of the robot never
    drags it along, because the material is a sibling of the robot, not a child
    — so the binding points outside the reference and USD drops it."""
    path = f"/World/Looks/NV_{name}"
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        prim = stage.DefinePrim(path)      # no type: let the reference supply it
        prim.GetReferences().AddReference(WELDING_USD, f"{NV_LOOKS}/{name}")
        for _ in range(4):
            app.update()
    return UsdShade.Material(stage.GetPrimAtPath(path))


def paint_arm(stage, arm_root):
    """KUKA livery, using the welding cell's own materials so the imported arm
    matches the user's robot line exactly."""
    total = 0
    for mat_name, links in ARM_PAINT.items():
        mat = nv_material(stage, mat_name)
        total += bind_material(stage, arm_root, mat, links=links)
    return total


def _carton_material(stage):
    """One physics material for every box on the line: real cardboard slides
    (mu ~0.3), PhysX's default 0.5 does not. This is what unlocks the divert
    queue -- a following box pressing into the one under divert grips it with
    default friction and the pair wedges against the stopper (measured, 70 s
    static); at carton friction the lead box shears free and carries on. The
    roller drive only needs mu*g >> the belt's 1.2 m/s ramp, so grip on the
    rollers stays plentiful."""
    path = "/World/Looks/carton_phys"
    if stage.GetPrimAtPath(path).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(path))
    mat = UsdShade.Material.Define(stage, path)
    m = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    m.CreateStaticFrictionAttr(0.30)
    m.CreateDynamicFrictionAttr(0.25)
    m.CreateRestitutionAttr(0.0)
    return mat


def rigid_box(stage, path, box_name, pos, z, yaw=0.0, mass=PKG_MASS):
    """A REAL cardboard box (Industrial pack) as a dynamic rigid body.

    The pack ships pure geometry — no physics — so we add it: RigidBodyAPI on
    the wrapper, CollisionAPI + CONVEX HULL on each mesh underneath. Convex hull
    is safe here (a box is ~12 triangles); it is only fatal on scanned meshes.
    Box pivots sit at the base, so `z` is the height of the box's UNDERSIDE.
    """
    usd = f"{CARDBOARD}/{box_name}.usd"
    UsdGeom.Xform.Define(stage, path)
    stage.DefinePrim(f"{path}/asset", "Xform").GetReferences().AddReference(usd)
    xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
    xf.SetTranslate(Gf.Vec3d(pos[0], pos[1], z))
    xf.SetRotate(Gf.Vec3f(0.0, 0.0, float(yaw)))
    xf.SetScale(Gf.Vec3f(BOX_SCALE, BOX_SCALE, BOX_SCALE))
    for _ in range(3):
        app.update()

    prim = stage.GetPrimAtPath(path)
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
    mat = _carton_material(stage)
    n = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        n += 1
    return n


def rigid_tote(stage, path, pos, mass=PKG_MASS, tote_mat=None):
    """A black industrial KLT tote as a dynamic package on the line.

    VISUAL vs COLLISION ARE SEPARATE ON PURPOSE. The KLT mesh is an open-top
    moulded bin: convex-hulling it fills the mouth (nothing could ever be put
    in it) and decomposing it per-tote is expensive at ~20 copies. So the
    visual is the referenced asset and the physics is RigidBodyAPI on the
    wrapper plus ONE invisible box proxy sized from the measured bbox -- the
    same split rigid_gs_package used for the splat, for the same reason.

    Normalised on the LONG axis, not the height: the raw bin is already the
    400:600 footprint of a real tote and its long side is +Y = the travel
    direction, so TOTE_LONG fixes the whole size class in one number and the
    cross-belt width falls out at 0.40.

    The stock KLT is PINK. `tote_mat` (an omni_pbr) is bound over every mesh
    in the asset; pass one in so N totes share ONE material instead of N.
    """
    UsdGeom.Xform.Define(stage, path)
    stage.DefinePrim(f"{path}/asset", "Xform").GetReferences() \
        .AddReference(TOTE_USD)
    for _ in range(8):
        app.update()

    b = bbox_of(stage.GetPrimAtPath(f"{path}/asset"))
    if not b:
        log("  WARNING: tote bbox empty -> falling back to a cardboard box")
        stage.RemovePrim(path)
        return False
    lo, hi = b
    raw_x, raw_y, raw_z = hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]
    s = TOTE_LONG / raw_y if raw_y > 1e-6 else 1.0
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2

    xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
    xf.SetScale(Gf.Vec3f(s, s, s))
    # centre the tote on `pos`, base ON the rollers (+2 cm so it settles down
    # onto them rather than starting interpenetrated). Offsets are the RAW
    # bbox scaled by s; no yaw, so they need no rotation.
    xf.SetTranslate(Gf.Vec3d(pos[0] - cx * s, pos[1] - cy * s,
                             ROLLER_TOP_Z + 0.02 - lo[2] * s))
    for _ in range(3):
        app.update()

    if tote_mat is not None:
        bind_material(stage, f"{path}/asset", tote_mat)

    prim = stage.GetPrimAtPath(path)
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(False)
    # author both velocity attrs up front so the capture-side recycler can
    # ZERO them on respawn. Without them a recycled tote keeps the momentum it
    # picked up falling into the bin and outruns the belt.
    rb.CreateVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    rb.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))

    # invisible box proxy, authored in the wrapper's RAW frame (the wrapper's
    # scale op scales it along with the visual). 0.92/0.98 shrink keeps the
    # proxy just inside the moulded wall so two totes touching on the belt do
    # not read as a gap.
    cube = UsdGeom.Cube.Define(stage, f"{path}/collider")
    cube.GetSizeAttr().Set(1.0)
    cxf = UsdGeom.Xformable(cube.GetPrim())
    cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, (lo[2] + hi[2]) / 2))
    cxf.AddScaleOp().Set(Gf.Vec3f(raw_x * 0.92, raw_y * 0.92, raw_z * 0.98))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        _carton_material(stage), UsdShade.Tokens.weakerThanDescendants,
        "physics")
    UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
    wrapper_z = ROLLER_TOP_Z + 0.02 - lo[2] * s
    log(f"  tote: raw {raw_x:.3f}x{raw_y:.3f}x{raw_z:.3f} -> scale {s:.3f} "
        f"= {raw_x*s:.2f} (cross-belt) x {raw_y*s:.2f} (travel) x "
        f"{raw_z*s:.2f} tall, wrapper z={wrapper_z:.3f}, "
        f"invisible box proxy + carton friction")
    # the RESPAWN z the recycler must use. NOT ROLLER_TOP_Z + 0.02: the KLT
    # pivot is at the bin's CENTRE (raw z -0.073..0.073), so the wrapper sits
    # |lo_z| * s ABOVE the roller top. Teleporting to the roller top instead
    # buries the tote ~0.15 m in the rollers and the surface velocity spits it
    # off the belt -- which is exactly the "respawned box drops to the floor"
    # that got recycling switched off.
    return wrapper_z


def industrial_scale(stage, probe_usd, target_m, axis=1):
    """The Industrial pack is authored in CENTIMETRES; whether USD corrects for
    that on reference depends on the asset shipping a unitsResolve op — so
    measure and derive instead of assuming."""
    p = stage.DefinePrim("/World/_probe", "Xform")
    p.GetReferences().AddReference(probe_usd)
    for _ in range(10):
        app.update()
    b = bbox_of(p)
    stage.RemovePrim("/World/_probe")
    if not b:
        return 1.0
    raw = b[1][axis] - b[0][axis]
    s = target_m / raw if raw > 1e-6 else 1.0
    log(f"  unit probe {os.path.basename(probe_usd)}: raw {raw:.2f} -> scale {s:.4f}")
    return s


# ---------------------------------------------------------------------------
# environment
# ---------------------------------------------------------------------------
def new_stage_at(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    ctx = omni.usd.get_context()
    ctx.new_stage()
    ctx.save_as_stage(path)
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(END_FRAME)
    log(f"stage created at {path}")
    return stage


def setup_environment(stage):
    log("environment: full_warehouse + physics scene")
    wh = stage.DefinePrim("/World/Warehouse", "Xform")
    wh.GetReferences().AddReference(WAREHOUSE_USD)
    for _ in range(12):
        app.update()

    if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()) \
            .CreateTimeStepsPerSecondAttr(60.0)
        log("  physics scene @ 60 Hz")

    stage.DefinePrim(SCENARIO, "Scope")
    dome = UsdLux.DomeLight.Define(stage, "/World/SceneDome")
    dome.CreateIntensityAttr(350.0)
    dome.CreateColorAttr(Gf.Vec3f(0.9, 0.93, 1.0))


# ---------------------------------------------------------------------------
# the conveyor: main line (N->S) + T junction + branch (E) + driven surfaces
# ---------------------------------------------------------------------------
def setup_conveyor(stage):
    """Every conveyor piece, and the one graph that drives them all.

    The spawn CURVE is part of this now: it is the head of the line, so it is
    `main_00` in this same Conveyor scope, not a stray prim off in Spawn/ (only
    the black hood stays there). Slot 0 is the curve, slots 1..8 the straights.

    Nothing is frozen and no slab is built: each piece's `Rollers` body is
    already kinematic with its collider on, and a conveyor node drives it.
    """
    root = f"{SCENARIO}/Conveyor"
    UsdGeom.Xform.Define(stage, root)

    # ALL directions below are WORLD-frame (localSpace=False on the baked
    # attrs): what you read is the direction the boxes go, and editing a
    # node's inputs:direction LIVE in the GUI means exactly the same thing --
    # a mid-sim write is applied world-space by PhysX no matter what the
    # local flag says, so world everywhere is the only self-consistent choice
    # (the user tuned (1,0,0) live on the branch and it ran east: that
    # experiment IS the world-frame runtime path).
    log("conveyor: spawn curve (main_00) + roller line + T junction + branch")
    runs = []

    def add(prim, label, direction, vel=BELT_SPEED, curved=False):
        """One conveyor node per rigid body under this piece."""
        for b in roller_bodies(prim):
            runs.append({"prim": b, "dir": direction, "vel": vel,
                         "curved": curved, "label": label})

    # slot 0: the 90-degree curve -- the head the packages ride in on, at the
    # user's baked spot (its exit mates main_01's north edge, see CURVE_POS).
    # A curved run wants the ROTATION AXIS, not a linear direction: (0,0,-37)
    # is NVIDIA's own A02 preset (track_types.json) -- omega about -Z, deg/s,
    # scaled so dir*vel matches v/r for the A02's ~1.55 m mid-belt radius.
    # (An axis about Z reads the same in world and local frames.)
    c = place(stage, f"{root}/main_00", SPAWN_CURVE_USD, CURVE_POS, 0.0)
    add(c, "main_00 curve", direction=(0.0, 0.0, -37.0), curved=True)

    for i in range(1, HEAD_PIECES):
        y = LINE_HEAD_Y - 2.0 * i
        p = place(stage, f"{root}/main_{i:02d}", STRAIGHT_USD, (LINE_X, y), -90.0)
        add(p, f"main_{i:02d}", direction=(0.0, -1.0, 0.0))     # south

    # the tee ships TWO bodies: `Rollers` (trunk) and `Rollers_01` (branch),
    # one conveyor node each. Values are the USER'S OWN, live-tuned in the GUI
    # on 16/07 and baked back verbatim (adapt, do not re-tune):
    #   * TRUNK (0.6 east, -0.8 south): mostly south with enough east drift
    #     to walk a passing box across the mouth.
    #   * BRANCH (1,0,0) pure east at BELT_SPEED: carries the diverted box to
    #     branch_00 and the bin.
    tj = place(stage, f"{root}/tee", TJUNCTION_USD, (LINE_X, T_Y), -90.0)
    tee_bodies = roller_bodies(tj)
    for b in tee_bodies:
        branch = b.GetName().endswith("_01")
        runs.append({"prim": b,
                     "dir": (1.0, 0.0, 0.0) if branch else (0.6, -0.8, 0.0),
                     "vel": BELT_SPEED,
                     "curved": False,
                     "label": "tee branch" if branch else "tee trunk"})
    log(f"  tee bodies: {[b.GetName() for b in tee_bodies]}")

    p = place(stage, f"{root}/branch_00", STRAIGHT_USD,
              (BRANCH_PIECE_X, BRANCH_Y), 0.0)
    add(p, "branch_00", direction=(1.0, 0.0, 0.0))              # east

    # POWERED TRANSFER STRIP over the trunk->branch seam. The A23 frame's own
    # ridge in the 4 cm seam between its two roller banks sits at z=0.7699,
    # ~1 mm ABOVE the 0.769 crowns (raycast-measured): a box crossing the
    # seam beaches on that undriven ridge and parks. Measured A/B, same
    # everything else: WITHOUT this strip 0/3 boxes make the turn (even solo
    # ones); WITH it 3/3 ride into the bin. It is the divert's load-bearing
    # part, not dressing -- a real sorter powers its transfer exactly here.
    # Flush, dark, all but invisible next to the rollers.
    strip = colored_box(stage, f"{root}/transfer_strip",
                        (LINE_X + 0.478, BRANCH_Y, 0.7455),
                        (0.09, 0.90, 0.05), (0.10, 0.12, 0.14),
                        collider=True)
    rb = UsdPhysics.RigidBodyAPI.Apply(strip)
    rb.CreateKinematicEnabledAttr(True)
    runs.append({"prim": strip, "dir": (0.97, 0.24, 0.0),
                 "vel": BELT_SPEED * 1.5, "curved": False,
                 "label": "transfer strip"})
    log("  powered transfer strip over the trunk->branch seam (the A23's own "
        "ridge there is 1 mm proud of the crowns and beaches crossing boxes)")

    # SLICK GUARD RAILS: every frame/rail mesh of every piece gets the
    # low-friction physics material (the ROLLERS keep theirs -- friction is
    # the drive). The diagonal trunk presses boxes into the tee's east rail
    # north of the mouth, and at stock friction the rail simply HOLDS them
    # there (measured: the box pinning at y=-19.6/-19.2 anchored every
    # pair-lock; the very first probe round showed a box parked dead against
    # an A05 rail the same way). Painted-steel guards vs cardboard are slick
    # in reality too.
    n_slick = 0
    mat = _slick_material(stage)
    for piece in stage.GetPrimAtPath(root).GetChildren():
        for m in Usd.PrimRange(piece):
            if not m.IsA(UsdGeom.Mesh) or "Rollers" in str(m.GetPath()):
                continue
            UsdShade.MaterialBindingAPI.Apply(m).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            n_slick += 1
    log(f"  {n_slick} frame/rail meshes bound to the slick physics material "
        f"(rollers keep stock friction)")

    for _ in range(8):
        app.update()
    log(f"  {HEAD_PIECES + 2} conveyor modules, "
        f"{len(runs)} driven roller bodies")

    conveyor_graph(stage, runs)
    stopper(stage, f"{SCENARIO}/Stopper")


# ---------------------------------------------------------------------------
# drop bin at the end of the branch
# ---------------------------------------------------------------------------
def setup_bin(stage):
    """The KLT bin the user asked for, in place of the hand-built box.

    KLT ships its own wall colliders, so a scaled copy catches packages with no
    extra physics. Its pivot is at the CENTRE (z -0.073..0.073 raw), hence the
    z offset that puts the bin's floor on the ground."""
    log("drop bin: KLT_Bin asset (scaled), replacing the hand-built box")
    root = f"{SCENARIO}/DropBin"
    UsdGeom.Xform.Define(stage, root)
    # the KLT bin at the east end of the branch (the T diverts into it). Boxes
    # the palletizing arms did not lift end up here; the recycler respawns them
    # back at the head.
    (cx, cy), yaw = BIN["pos"], BIN["yaw"]
    s = BIN["scale"]
    half_h = KLT_RAW[2] / 2 * s
    place(stage, f"{root}/bin", KLT_USD, (cx, cy), yaw, z=half_h, scale=s)
    for _ in range(6):
        app.update()

    # the stock KLT is PINK; the reference photo's skip is industrial blue
    blue = omni_pbr(stage, "BinBlue", (0.06, 0.28, 0.68), rough=0.40, metal=0.0)
    painted = bind_material(stage, f"{root}/bin", blue)
    log(f"  repainted {painted} bin meshes industrial blue (stock KLT is pink)")

    b = bbox_of(stage.GetPrimAtPath(f"{root}/bin"))
    if b:
        lo, hi = b
        log(f"  bin at ({cx:.2f},{cy:.2f}) scale {s} -> "
            f"{hi[0]-lo[0]:.2f} x {hi[1]-lo[1]:.2f} x {hi[2]-lo[2]:.2f} m, "
            f"rim z={hi[2]:.2f} (roller top {ROLLER_TOP_Z:.2f} -> boxes drop in), "
            f"mouth spans x[{lo[0]:.2f},{hi[0]:.2f}] "
            f"vs branch end x={BRANCH_END_X:.2f}")
        if hi[2] > ROLLER_TOP_Z:
            log("  WARNING: bin rim is ABOVE the rollers — boxes will hit the "
                "wall instead of falling in; lower BIN['scale']")


# ---------------------------------------------------------------------------
# head of the line: pallet of packages + the loading arm
# ---------------------------------------------------------------------------
def place_arm(stage, root, name, pos, yaw, gripper=True):
    """The GRAFTED KUKA: the welding cell's own robot body (178 meshes, 34
    materials) on Isaac's KR210 articulation, standing on the cell's pedestal.

    This replaces the Kawasaki RS080N that stood here before. The RS080N was
    only ever a least-bad choice -- 14 meshes, white plastic -- and it never
    looked like the user's line. kuka_graft.py explains how the two robots are
    married; the short version is that the shell's meshes ride on the skeleton's
    rigid bodies, and the skeleton's joints were reshaped onto the shell's
    dimensions so nothing is stretched. It picks and places like a KR210 and
    photographs like the cell robot, because it is both.

    gripper=False for the cell arms: they are dressing, not pickers, so the
    suction gripper (a welded pad + four D6 joints) is dead weight on them."""
    kuka_graft.graft(stage, app, f"{root}/{name}", (pos[0], pos[1], 0.0), yaw,
                     gripper=gripper, log=log)
    for _ in range(6):
        app.update()
    d_belt = abs(pos[0] - LINE_X)
    d_pallet = abs(pos[0] - PALLET_STACK_X)
    log(f"  {name}: grafted KUKA at ({pos[0]:.2f},{pos[1]:.2f}) yaw {yaw:.0f} "
        f"on a {PEDESTAL_TOP_Z:.2f} m pedestal -> belt {d_belt:.2f} m, "
        f"drop pallet {d_pallet:.2f} m (reach {ARM_REACH:.2f} m)")
    if max(d_belt, d_pallet) > ARM_REACH:
        log(f"  WARNING: {max(d_belt, d_pallet):.2f} m is beyond the "
            f"{ARM_REACH:.2f} m reach — the arm cannot serve both")


# ---------------------------------------------------------------------------
# the fenced safety cell (two arms + pallets), line passes THROUGH it
# ---------------------------------------------------------------------------
def setup_robot_cell(stage):
    log(f"palletizing line: GenericFence + {len(CELL_ROBOTS)} KUKA stations "
        "down the DEPTH + pallets + forklift (line runs through N+S)")
    root = f"{SCENARIO}/RobotCell"
    UsdGeom.Xform.Define(stage, root)

    # --- fence: the sample's own GenericFence panels (2.5 m tall), tiled along
    # the four walls. Two ways IN are left open: the belt doorway in the N and S
    # walls, and a forklift entrance in the EAST wall on the drop-off pallet.
    cnt = [0]

    def run(fixed, a, b, horizontal):
        """Fill [a,b] along one axis with 1.60 m panels; right-align the last so
        the far corner has no gap; skip a stretch too short for even one panel."""
        if b - a < 0.80:
            return
        n = max(1, int((b - a) / GENFENCE_W))
        starts = [a + i * GENFENCE_W for i in range(n)]
        if a + n * GENFENCE_W < b - 0.15:          # remainder -> one more, flush
            starts.append(b - GENFENCE_W)
        for s in starts:
            pos = (s, fixed) if horizontal else (fixed, s)
            place_subprim(stage, f"{root}/fence_{cnt[0]:02d}", WELDING_USD,
                          GENFENCE_PRIM, pos, 0.0 if horizontal else 90.0,
                          scale=GENFENCE_SCALE)
            cnt[0] += 1

    belt_hi = LINE_X + 0.60                    # the belt hugs the west wall, so
    ent_lo = CELL_ENTRANCE_Y - CELL_ENTRANCE_W / 2   # the N/S runs are only east
    ent_hi = CELL_ENTRANCE_Y + CELL_ENTRANCE_W / 2   # of the belt doorway
    run(CELL["y1"], belt_hi, CELL["x1"], True)         # N wall (east of belt)
    run(CELL["y0"], belt_hi, CELL["x1"], True)         # S wall (east of belt)
    run(CELL["x0"], CELL["y0"], CELL["y1"], False)     # W wall (full)
    run(CELL["x1"], CELL["y0"], ent_lo, False)         # E wall, south of entrance
    run(CELL["x1"], ent_hi, CELL["y1"], False)         # E wall, north of entrance
    log(f"  {cnt[0]} GenericFence panels (2.5 m) around x[{CELL['x0']:.1f},"
        f"{CELL['x1']:.1f}] y[{CELL['y0']:.1f},{CELL['y1']:.1f}]; belt doorway "
        f"N+S, {CELL_ENTRANCE_W:.1f} m forklift entrance in the east wall")

    # a KUKA at each station, and a pallet just east of it that it fills
    for rx, ry, ryaw, label in CELL_ROBOTS:
        place_arm(stage, root, label, (rx, ry), ryaw, gripper=False)
        pallet_with_stack(stage, f"{root}/{label}_drop", (PALLET_STACK_X, ry),
                          0.0)
    log(f"  {len(CELL_ROBOTS)} palletizing stations "
        f"({', '.join(l for *_, l in CELL_ROBOTS)}), each a KUKA + pallet")

    # forklift waiting at the east entrance to run finished pallets to the racks
    place(stage, f"{root}/forklift", FORKLIFT_USD, FORKLIFT["pos"],
          FORKLIFT["yaw"])
    for _ in range(6):
        app.update()
    log(f"  forklift at {FORKLIFT['pos']} (east entrance, heading to the racks)")


def pallet_with_stack(stage, path, pos, yaw=0.0):
    """A pallet carrying a short stack of cartons -- the palletised output a
    forklift carries off. Static props (display dressing). Returns the deck
    height so callers can log it."""
    place(stage, f"{path}_pallet", PALLET_DROP_USD, pos, yaw)
    for _ in range(4):
        app.update()
    pb = bbox_of(stage.GetPrimAtPath(f"{path}_pallet"))
    deck = pb[1][2] if pb else 0.14
    bw, bd, bh = BOXES["Cardbox_C1"]
    for i, (cx, layer) in enumerate([(-0.5, 0), (0.5, 0), (0.0, 1)]):
        place(stage, f"{path}_box_{i}", f"{CARDBOARD}/Cardbox_C1.usd",
              (pos[0] + cx * (bw + 0.02), pos[1]), z=deck + layer * bh,
              scale=BOX_SCALE)
    for _ in range(4):
        app.update()
    return deck


# ---------------------------------------------------------------------------
# the black hood that hides where the packages appear
# ---------------------------------------------------------------------------
def setup_spawn(stage):
    """The black-box hood over the head of the line.

    The CURVE it covers is no longer built here -- it is a conveyor piece like any
    other now (Conveyor/main_00). All that is left in this scope is the hood: a
    five-sided black enclosure, open to the south, so the packages read as coming
    in from the next hall and the moment they spawn is out of sight inside it.
    The hood sits exactly where the user left it in the saved GUI layout.
    """
    log("spawn: black-box hood over the head (hides the package spawn)")
    root = f"{SCENARIO}/Spawn"
    UsdGeom.Xform.Define(stage, root)

    # small black-box hood over the curve MOUTH: N, E, W walls + roof, open SOUTH
    # so the boxes exit onto the line. No colliders (pure dressing) -- the belt is
    # well inside the walls, so nothing on it is touched. Sized just to cover.
    sx, sy, sz = SPAWN_HOOD["size"]
    w = SPAWN_HOOD["wall"]
    cx, cy = SPAWN_HOOD["center"]
    black = (0.02, 0.02, 0.025)

    def wall(name, c, size):
        colored_box(stage, f"{root}/{name}", c, size, black, collider=False)

    wall("hood_n", (cx, cy + sy / 2, sz / 2), (sx, w, sz))       # north wall
    wall("hood_e", (cx + sx / 2, cy, sz / 2), (w, sy, sz))       # east wall
    wall("hood_w", (cx - sx / 2, cy, sz / 2), (w, sy, sz))       # west wall
    wall("hood_top", (cx, cy, sz - w / 2), (sx, sy, w))          # roof
    log(f"  hood {sx:.1f}x{sy:.1f}x{sz:.1f} m at ({cx:.1f},{cy:.1f}) open south; "
        f"curve at the head ({CURVE_POS[0]:.2f},{CURVE_POS[1]:.2f})")


# ---------------------------------------------------------------------------
# packages already riding the line at t=0, plus workers (AMR removed)
# ---------------------------------------------------------------------------
def setup_packages(stage):
    """Every package on this line is a black KLT tote -- no cartons, no splat.

    ONE shared material for all of them: a per-tote material would author N
    shaders for a single colour, and the drop bin already proved the bind
    (stock KLT is pink).
    """
    log("packages on the line: black KLT totes")
    root = f"{SCENARIO}/Packages"
    UsdGeom.Xform.Define(stage, root)
    tote_mat = omni_pbr(stage, "ToteBlack", TOTE_COLOR,
                        rough=TOTE_ROUGH, metal=0.0)
    made = 0
    respawn_z = ROLLER_TOP_Z + 0.02        # carton fallback value
    for i, y in enumerate(PKG_START_Y):
        # pkg_0 rides in on the CURVE (west of the line centre); the rest start
        # centred on the belt (tote is 0.40 across on a 1.15 m belt).
        x = PKG0_SPAWN_X if i == 0 else LINE_X
        z = rigid_tote(stage, f"{root}/pkg_{i:02d}", (x, y), tote_mat=tote_mat)
        if z:
            log(f"  pkg_{i:02d} tote at ({x:.2f},{y:.2f})")
            respawn_z = z
            made += 1
            continue
        # fallback only if the KLT reference failed to resolve
        name = LINE_BOXES[i % len(LINE_BOXES)]
        rigid_box(stage, f"{root}/pkg_{i:02d}", name, (x, y),
                  ROLLER_TOP_Z + 0.02, yaw=90.0 * (i % 2))
        log(f"  pkg_{i:02d} FALLBACK carton {name} at ({x:.2f},{y:.2f})")
    # ------------------------------------------------------------------
    # PARKED POOL -- the feed for the rest of the take.
    #
    # Recycling a tote OUT of the bin does not work: it is a live dynamic body
    # carrying the momentum of the fall, teleporting it does not clear that
    # (writing physics:velocity is ignored once PhysX owns the body), and it
    # re-enters faster than the 1.20 m/s belt, drifts off the roller edge at
    # x=-14.45, climbs the side rail to z=0.97 and jams. Every downstream
    # symptom -- stalls, rear-end pile-ups, the head-clear deadlock -- came
    # from that one thing.
    #
    # A parked tote is KINEMATIC from build time, so it has no momentum to
    # carry. The capture only has to move it and flip it dynamic. This is the
    # pattern the package pool follows -- every physics prim exists before
    # the timeline starts.
    for i in range(POOL_SIZE):
        z = rigid_tote(stage, f"{root}/pool_{i:02d}",
                       (PARK_XY[0] + 0.9 * i, PARK_XY[1]), tote_mat=tote_mat)
        if z:
            p = stage.GetPrimAtPath(f"{root}/pool_{i:02d}")
            p.GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(PARK_XY[0] + 0.9 * i, PARK_XY[1], PARK_Z))
            p.GetAttribute("physics:kinematicEnabled").Set(True)
    log(f"  {POOL_SIZE} spare totes parked kinematic at z={PARK_Z:.0f}")
    log(f"  {made}/{len(PKG_START_Y)} totes built")

    # RESPAWN ON THE STRAIGHT, NOT ON THE CURVE.
    #
    # The carton build put this at the curve mouth (PKG0_SPAWN_X) so a recycled
    # box re-entered exactly like the first one. A TOTE dropped there gets
    # thrown: main_00 is driven by an ANGULAR surface velocity
    # (dir=(0,0,-37) curved) and the tote's lighter, taller, centre-pivoted
    # body slides off it eastward. Measured on a clear head -- so this is the
    # curve, not a collision: respawned at (-14.99,-1.57), came to rest at
    # (-12.89,-2.62,0.15), on the floor, 2.1 m east.
    #
    # y = LINE_HEAD_Y - 2.33 (= -3.30) is on main_01, whose measured span is
    # y[-4.96,-2.97] with lin=(0,-1.20,0) -- a pure southward drive, no
    # angular term to throw the tote off. It is also inside the lengthened
    # hood (y[-3.55,-0.35]), so the reappearance is hidden.
    respawn_xy = (LINE_X, LINE_HEAD_Y - 2.33)
    stage.GetPrimAtPath(root).CreateAttribute(
        "omni:recycler:respawn", Sdf.ValueTypeNames.Double3, custom=True
    ).Set(Gf.Vec3d(respawn_xy[0], respawn_xy[1], respawn_z))
    log(f"  recycler respawn point ({respawn_xy[0]:.2f},"
        f"{respawn_xy[1]:.2f},{respawn_z:.3f}) -- on main_01 straight, "
        f"under the hood")


def bind_clip(stage, char_root, clip_usd):
    """Isaac People characters load in a T-POSE: the rig ships with no
    animation source bound (omni.anim.people normally binds one at runtime).
    Reference a skel clip in and point the skeleton at it, or the workers
    stand there like scarecrows."""
    from pxr import UsdSkel

    if not os.path.exists(clip_usd):
        log(f"  WARNING: clip missing: {clip_usd} -> T-pose")
        return False
    # the rig arrives as an UNLOADED PAYLOAD: without this the skeleton is
    # invisible to PrimRange and every worker stays a scarecrow
    stage.Load(Sdf.Path(char_root))
    for _ in range(4):
        app.update()

    def walk(root):
        return Usd.PrimRange(stage.GetPrimAtPath(root),
                             Usd.TraverseInstanceProxies())

    # An INSTANCEABLE character is composed from a shared prototype: everything
    # under it is an instance proxy, so Apply()/SetTargets() land on a read-only
    # proxy and are silently dropped (bind reported success, worker still
    # T-posed). De-instance this copy first.
    for p in walk(char_root):
        if p.IsInstance():
            p.SetInstanceable(False)
    for _ in range(4):
        app.update()

    skels = [p for p in walk(char_root) if p.IsA(UsdSkel.Skeleton)]
    roots = [p for p in walk(char_root) if p.IsA(UsdSkel.Root)]
    if not skels:
        log(f"  WARNING: no Skeleton under {char_root} -> T-pose")
        return False

    # The clip must live INSIDE the SkelRoot: UsdSkel resolves animationSource
    # within the SkelRoot's scope, and a sibling prim is simply not seen.
    # DefinePrim with NO type — an explicit "Xform" is a local opinion that
    # beats the referenced layer's own SkelAnimation type.
    holder = roots[0] if roots else stage.GetPrimAtPath(char_root)
    anim_root = f"{holder.GetPath()}/clip"
    stage.DefinePrim(anim_root).GetReferences().AddReference(clip_usd)
    stage.Load(Sdf.Path(anim_root))
    for _ in range(4):
        app.update()

    anim = next((p for p in walk(anim_root) if p.IsA(UsdSkel.Animation)), None)
    if anim is None:
        log(f"  WARNING: {os.path.basename(clip_usd)} has no SkelAnimation "
            f"prim -> T-pose")
        return False
    for target in skels + roots:
        UsdSkel.BindingAPI.Apply(target).CreateAnimationSourceRel() \
            .SetTargets([anim.GetPath()])
    return True


def setup_user_blocks(stage):
    """Bring the user's hand-tuned RobotCell + RackRows in AS THEY ARE.

    Referenced, not rebuilt: whatever is inside those prims (fence layout, rack
    spacing, dressing) composes in untouched. The only thing we author is the
    wrapper's translate, and only where the block would otherwise sit on top of
    the conveyor."""
    log("user blocks (referenced verbatim from the hand-tuned layouts)")
    root = f"{SCENARIO}/UserBlocks"
    stage.DefinePrim(root, "Scope")
    for blk in USER_BLOCKS:
        usd, prim, (dx, dy) = blk["usd"], blk["prim"], blk["shift"]
        # RackRows references the STABLE source (scenario1_scene_USER_EDIT_2343,
        # which already carries the user's three spread columns).
        # It must NOT be referenced out of the live GUI save file:
        # the user saves the GUI-composed scene back over that very file, so a
        # reference into it becomes a SELF-REFERENCE CYCLE -- USD detects the
        # cycle and silently drops the whole subtree, which is exactly why the
        # three racks vanished. 2343 is an external file the output never
        # overwrites, so the cycle cannot form.
        if not os.path.exists(usd):
            log(f"  WARNING: {usd} missing -> {blk['name']} skipped")
            continue
        path = f"{root}/{blk['name']}"
        UsdGeom.Xform.Define(stage, path)
        stage.DefinePrim(f"{path}/asset", "Xform").GetReferences() \
            .AddReference(usd, prim)
        UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path)).SetTranslate(
            Gf.Vec3d(dx, dy, 0.0))
        for _ in range(8):
            app.update()
        b = bbox_of(stage.GetPrimAtPath(path))
        if b:
            lo, hi = b
            log(f"  {blk['name']}: shift ({dx},{dy}) -> bbox "
                f"({lo[0]:.1f},{lo[1]:.1f})..({hi[0]:.1f},{hi[1]:.1f}) "
                f"h={hi[2]:.1f}  [line is at x={LINE_X}, bin centre "
                f"x={BIN['pos'][0]:.1f}]")
        else:
            log(f"  WARNING: {blk['prim']} referenced but bbox empty")


def walk_worker(stage, path, phase):
    """Patrol root motion: translate + rotateZ time samples along WALK_LOOP,
    exactly one lap over the timeline (END_FRAME is derived from the loop, so
    looping playback is seamless). The walk clip is in place; this is the
    scripted-nav root. `phase` is in laps -- keep it off the corners so the
    loop point does not land mid-turn."""
    segs = []
    for i, a in enumerate(WALK_LOOP):
        b = WALK_LOOP[(i + 1) % len(WALK_LOOP)]
        segs.append((a, b, math.hypot(b[0] - a[0], b[1] - a[1])))

    poses, yaws = [], []
    for f in range(END_FRAME + 1):
        s = ((f / FPS) * WALK_SPEED + phase * _WALK_PERIM) % _WALK_PERIM
        acc = 0.0
        for a, b, d in segs:
            if s <= acc + d and d > 0.0:
                u = (s - acc) / d
                poses.append((a[0] + (b[0] - a[0]) * u,
                              a[1] + (b[1] - a[1]) * u))
                yaws.append(math.degrees(math.atan2(b[1] - a[1],
                                                    b[0] - a[0])))
                break
            acc += d

    # unwrap, then box-smooth the yaw so a corner reads as a turn, not a snap
    for i in range(1, len(yaws)):
        while yaws[i] - yaws[i - 1] > 180.0:
            yaws[i] -= 360.0
        while yaws[i] - yaws[i - 1] < -180.0:
            yaws[i] += 360.0
    k = 15                                    # +-0.25 s blend
    smooth = [sum(yaws[max(0, i - k):i + k + 1])
              / len(yaws[max(0, i - k):i + k + 1]) for i in range(len(yaws))]

    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp()
    r_op = xf.AddRotateZOp()
    for f, ((x, y), yaw) in enumerate(zip(poses, smooth)):
        t_op.Set(Gf.Vec3d(x, y, 0.0), Usd.TimeCode(f))
        r_op.Set(float(yaw + WALK_YAW_OFF), Usd.TimeCode(f))


def setup_people_and_props(stage):
    log("workers: walk-clip patrol around the aisle east of the cell")
    root = f"{SCENARIO}/Props"
    stage.DefinePrim(root, "Scope")
    for i, char in enumerate(WORKERS):
        usd = f"{PEOPLE_DIR}/{char}/{char}.usd"
        if not os.path.exists(usd):
            log(f"  WARNING: worker asset missing: {usd}")
            continue
        place(stage, f"{root}/worker_{i}", usd, WALK_LOOP[0], 0.0)
        clip = os.path.join(WALK_CLIP_DIR,
                            f"walk_{char}.skelanim.usd").replace("\\", "/")
        animated = bind_clip(stage, f"{root}/worker_{i}", clip)
        phase = 0.10 + 0.5 * i                # half a lap apart, off corners
        walk_worker(stage, f"{root}/worker_{i}", phase)
        log(f"  worker {i} ({char}) patrols {_WALK_PERIM:.0f} m loop, "
            f"phase {phase:.2f}, "
            f"{'walk clip bound' if animated else 'T-POSE (clip missing)'}")
    # The forklift at the cell's east entrance is the only mobile-robot
    # dressing; the AMR was removed at the user's request.


# ---------------------------------------------------------------------------
# cameras: 4 wall corners + 2 record cams
# ---------------------------------------------------------------------------
def _look(eye, tgt):
    eye, tgt = Gf.Vec3d(*eye), Gf.Vec3d(*tgt)
    fwd = (tgt - eye).GetNormalized()
    right = Gf.Cross(fwd, Gf.Vec3d(0, 0, 1)).GetNormalized()
    up = Gf.Cross(right, fwd)
    m = Gf.Matrix4d()
    m.SetIdentity()
    m.SetRotateOnly(Gf.Matrix3d(right[0], right[1], right[2],
                                up[0], up[1], up[2],
                                -fwd[0], -fwd[1], -fwd[2]))
    m.SetTranslateOnly(eye)
    return m


def setup_cameras(stage):
    """All cameras live under /World/Cameras — the scope the user's backup
    keeps them in, so GUI camera picks land in the same place either way."""
    log("cameras: 4 wall corners + RecordCam_Full + RecordCam_Ref")
    stage.DefinePrim("/World/Cameras", "Scope")
    for name, c in CAMERAS.items():
        cam = UsdGeom.Camera.Define(stage, f"/World/Cameras/{name}")
        cam.GetFocalLengthAttr().Set(c["f"])
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(_look(c["eye"], c["tgt"]))
        log(f"  {name} eye={c['eye']} focal={c['f']}")


# ---------------------------------------------------------------------------
# package recycler: keeps the flow going forever (GUI Play / long renders)
# ---------------------------------------------------------------------------
RECYCLER = os.path.join(SCRIPT_DIR, "package_recycler.py").replace("\\", "/")


def attach_recycler(stage):
    if not os.path.exists(RECYCLER):
        log(f"  WARNING: {RECYCLER} missing -> no recycling (boxes fill the bin "
            f"and stop)")
        return
    import omni.kit.commands
    target = f"{SCENARIO}/Packages"
    try:
        omni.kit.commands.execute("ApplyScriptingAPICommand",
                                  paths=[Sdf.Path(target)])
    except Exception as e:
        log(f"  ApplyScriptingAPICommand failed: {type(e).__name__}: {e}")
    prim = stage.GetPrimAtPath(target)
    attr = prim.GetAttribute("omni:scripting:scripts")
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute("omni:scripting:scripts",
                                    Sdf.ValueTypeNames.AssetArray, custom=False)
    attr.Set([Sdf.AssetPath(RECYCLER)])
    log(f"  recycler attached to {target} (boxes that land in the bin respawn "
        f"at the head of the line)")


# ---------------------------------------------------------------------------
def main():
    stage = new_stage_at(SCENE_OUT)
    setup_environment(stage)
    setup_conveyor(stage)       # curve (main_00) + line + tee + branch + graph
    setup_bin(stage)            # KLT bin at the east end of the branch
    setup_user_blocks(stage)    # the racks, referenced from the user's layout
    setup_robot_cell(stage)     # 2 palletizing arms + fence + forklift
    setup_spawn(stage)          # black-box hood over the head (hides the spawn)
    setup_packages(stage)
    setup_people_and_props(stage)
    setup_cameras(stage)
    attach_recycler(stage)
    for _ in range(10):
        app.update()

    ok = omni.usd.get_context().save_stage()
    log(f"scene saved -> {SCENE_OUT} (ok={ok}, "
        f"{os.path.getsize(SCENE_OUT) / 1e6:.2f} MB)")
    log("open in GUI -> camera RecordCam_Ref -> Play")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback
        log("FATAL:\n" + traceback.format_exc())
        raise
    finally:
        app.close()
