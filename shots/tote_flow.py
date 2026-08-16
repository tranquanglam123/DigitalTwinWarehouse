"""shots: warehouse tote flow -- headless capture against assets/warehouse_tote_flow.usd.

The scene file is NEVER modified: the timeline is extended on the SESSION
layer and nothing is saved back.

BUILDERS
  build_test      180 f  static 3/4 on the line mid-section. The ONLY job of
                         this one is to answer "do the totes render BLACK or
                         MAGENTA" before any hours go into a full render.
                         (KLT ships a pink MDL; setup_packages binds OmniPBR
                         over it -- this is the shot that proves the bind.)
  build_overview  N  f   smooth analytic orbit around the line + tee + bin.

RUN
  python.bat lib/cine_capture_core.py --builder shots/tote_flow.py:build_test \
      --frames 180 --subframes 4 --out output/frames/_test \
      --asset-root <asset_root from config/assets.json>

ASCII-only prints (Isaac stdout is cp1252).
"""
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

SCENE = os.environ.get(
    "TF_SCENE",
    os.path.join(REPO_ROOT, "assets", "warehouse_tote_flow.usd")
).replace("\\", "/")

FPS = 60

# ---------------------------------------------------------------------------
# geometry of interest, read off the build log (build_scene.py prints these)
# ---------------------------------------------------------------------------
# Everything is derived from LINE_X so the cameras follow the layout instead
# of being re-typed every time it moves. Keep in step with build_scene.py.
LINE_X = -14.00                 # conveyor centreline (moved east, was -24.92)
LINE_HEAD_Y = -0.97             # north end, totes enter here
T_Y = -18.00                    # tee
BRANCH_Y = T_Y - 2.10           # branch centreline
BIN_XY = (LINE_X + 5.09, BRANCH_Y)      # drop bin at the branch end
ROLLER_TOP_Z = 0.769

# the run worth filming: head -> tee -> branch -> bin
FOCUS_CENTER = (LINE_X + 1.9, -11.0, 1.6)
FOCUS_DIAG = 26.0

# orbit for the overview. The warehouse ROOF is at z~9 and the lamp band
# reaches z5.83, so 5.0 keeps the lens inside the building -- the first survey
# put cameras at z16-34 and rendered the roof from outside, all white.
ORBIT_CENTER = (LINE_X + 1.9, -10.5, 1.2)
ORBIT_TARGET = (LINE_X + 1.2, -11.0, 1.9)
# Tightened after render 1: rx11/ry14 at z5.0 with focal 18 put the lens far
# enough out that half the orbit framed empty floor instead of the line. The
# warehouse ROOF is at z~9 and the lamp band reaches 5.83, so 4.2 still clears
# the fixtures while reading much less top-down.
ORBIT_RX, ORBIT_RY = 8.5, 11.0
ORBIT_Z = 4.2
ORBIT_PERIOD_S = 30.0
ORBIT_FOCAL = 20.0


def _open(app, total_frames):
    """Open the built scene, enable the conveyor extension, stretch the
    session timeline. The file itself is never written."""
    import omni.kit.app          # type: ignore
    import omni.usd              # type: ignore
    mgr = omni.kit.app.get_app().get_extension_manager()
    # the IsaacConveyor nodes need their extension; surface velocities are
    # ALSO baked on the bodies, so the scene still moves if this ever fails
    mgr.set_extension_enabled_immediate("isaacsim.asset.gen.conveyor", True)
    # the package recycler is an omni.kit.scripting BehaviorScript authored
    # into the file -- without this the line runs dry once the totes divert
    mgr.set_extension_enabled_immediate("omni.kit.scripting", True)

    ctx = omni.usd.get_context()
    ctx.open_stage(SCENE)
    for _ in range(60):
        app.update()
    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError("open_stage failed: %s" % SCENE)

    sess = stage.GetSessionLayer()
    sess.startTimeCode = 0
    sess.endTimeCode = total_frames + 120

    pkgs = stage.GetPrimAtPath("/World/Scenario1/Packages")
    n = len([c for c in pkgs.GetChildren()
             if c.GetName().startswith("pkg_")]) if pkgs else 0
    print("[tote_flow] scene open, %d packages, session endTimeCode=%d"
          % (n, total_frames + 120), flush=True)
    return stage


# ---------------------------------------------------------------------------
# RECYCLER -- driven from on_step, NOT from the BehaviorScript in the file.
#
# The scene does author omni:scripting:scripts -> package_recycler.py, and the
# probe proved it never fires headless: totes reached the bin, sat there for
# 30 s past the dwell, recycled=0. Rather than chase the scripting manager
# through a headless boot, the capture drives the same logic itself. Same
# pattern CAPTURE_NOTES 4.3 used for the heatmap tracker, same reason.
#
# Freeze -> move -> thaw, so the tote wakes with ZERO velocity instead of
# inheriting the drop into the bin.
# ---------------------------------------------------------------------------
FLOOR_Z = 0.60          # below this it is off the rollers
BIN_X = LINE_X + 3.00   # east of this it is over/in the bin
DWELL_F = 90            # 1.5 s settled in the bin before it goes back
GAP_F = 280             # 4.7 s between two respawns. At 200 with five totes the
                        # recycler fired continuously (f=1220/1420/1620/1820)
                        # and dropped a tote onto one still sitting on the
                        # spawn CURVE, whose angular drive then flung it east
                        # onto the floor -- the exact failure earlier builds
                        # warned about for the line-centre respawn.
COOLDOWN_F = 600        # per tote. Head-to-bin transit is ~1020 frames, so this
                        # cannot block a real recycle -- it only kills the
                        # double-fire described in _make_recycler.
RESPAWN_CLEAR = 2.2     # m. Do not drop a tote onto the head while another is
                        # still standing there.
STRANDED_F = 180        # 3 s. A tote off the belt but not in the bin is stuck
                        # for good; recover it rather than leave dead geometry
                        # in frame. Longer than DWELL_F so a tote still
                        # tumbling into the bin is not grabbed mid-fall.


def _make_recycler(stage, respawn_pt, verbose=False):
    """FEEDER, not a recycler.

    Totes that reach the bin stay in the bin -- which is what a real chute
    does anyway. New totes come from a pool parked kinematic above the scene
    by build_scene.py. Feeding a parked body is safe; recycling a live one is
    not (see the note on POOL_SIZE in build_scene.py).
    """
    root = stage.GetPrimAtPath("/World/Scenario1/Packages")
    pool = sorted([c for c in root.GetChildren()
                   if c.GetName().startswith("pool_")],
                  key=lambda p: p.GetName()) if root else []
    print("[tote_flow] feeder: %d parked totes, drop=(%.2f,%.2f,%.2f)"
          % (len(pool), respawn_pt[0], respawn_pt[1], respawn_pt[2]),
          flush=True)
    last_respawn = [-10 ** 6]
    nxt = [0]
    count = [0]

    def tick(f):
        from pxr import Gf   # type: ignore
        if nxt[0] >= len(pool) or f - last_respawn[0] < GAP_F:
            return
        # head clear? never drop a tote onto one that has not moved off yet.
        # Only a tote ON THE BELT counts; a tote on the floor near the head
        # must not latch this forever, which deadlocked the earlier build.
        for p in pool[:nxt[0]]:
            t = p.GetAttribute("xformOp:translate").Get()
            if t is None:
                continue
            if (t[2] > FLOOR_Z
                    and abs(t[0] - respawn_pt[0]) < RESPAWN_CLEAR
                    and abs(t[1] - respawn_pt[1]) < RESPAWN_CLEAR):
                return

        p = pool[nxt[0]]
        # parked totes are kinematic from build time, so there is no momentum
        # to clear -- move it, then hand it to the solver.
        p.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(respawn_pt[0], respawn_pt[1], respawn_pt[2]))
        kin = p.GetAttribute("physics:kinematicEnabled")
        if kin and kin.IsValid():
            kin.Set(False)
        nxt[0] += 1
        last_respawn[0] = f
        count[0] += 1
        if verbose:
            print("[tote_flow] fed %s at f=%d (total %d)"
                  % (p.GetName(), f, count[0]), flush=True)

    return tick, count


def _respawn_point(stage):
    """The scene authors this on the Packages prim, carrying the tote wrapper's
    pivot offset (KLT pivot is at the bin CENTRE, so the wrapper rides ~0.15 m
    above the roller top). Teleporting to ROLLER_TOP_Z instead buries the tote
    in the rollers and the surface velocity spits it off the belt."""
    a = stage.GetPrimAtPath("/World/Scenario1/Packages") \
             .GetAttribute("omni:recycler:respawn")
    if a and a.HasValue():
        v = a.Get()
        return (float(v[0]), float(v[1]), float(v[2]))
    return (LINE_X - 0.99, LINE_HEAD_Y - 0.6, 0.937)


def _orbit_eye(t_sim):
    ang = 2.0 * math.pi * (t_sim / ORBIT_PERIOD_S)
    cx, cy, _ = ORBIT_CENTER
    return (cx + ORBIT_RX * math.cos(ang),
            cy + ORBIT_RY * math.sin(ang),
            ORBIT_Z)


# ---------------------------------------------------------------------------
# TEST -- colour/geometry sanity, 180 frames, subframes 4. Not a deliverable.
# ---------------------------------------------------------------------------
def build_test(app):
    _open(app, 180)
    from lib.cine_capture_core import ensure_camera   # type: ignore

    # 3/4 from the east side of the line, low, framing the stretch the totes
    # ride between the head and the tee. focal 35: Isaac's aperture is narrow
    # (34 mm already reads tele), and at 6 m out that shows ~2 m of height --
    # enough for a 0.30 m tote to fill a useful part of the frame.
    eye = (LINE_X + 3.32, -9.6, 1.85)
    tgt = (LINE_X, -12.4, 0.95)

    def on_step(f):
        ensure_camera(pos=eye, target=tgt, focal_length=35.0)
        if f % 60 == 0:
            print("[tote_flow/test] f=%d" % f, flush=True)

    keys = [(0.0, eye, tgt), (1.0, eye, tgt)]
    return FOCUS_CENTER, FOCUS_DIAG, on_step, keys


# ---------------------------------------------------------------------------
# PROBE -- physics only, NO render. Answers the three questions that decide
# whether a full render is worth starting:
#   1. do the totes ride the line south?
#   2. does the tee divert them east onto the branch?
#   3. do they land in the bin, and does the recycler put them back?
# Runs ~50x faster than rendering the same span.
#
#   python.bat lib/cine_capture_core.py --builder shots/tote_flow.py:build_probe \
#       --probe 1800 --out output/frames/_ignore --asset-root <root>
# ---------------------------------------------------------------------------
def build_probe(app):
    stage = _open(app, 1800)

    root = stage.GetPrimAtPath("/World/Scenario1/Packages")
    pkgs = [c for c in root.GetChildren()
            if c.GetName().startswith("pkg_")] if root else []
    print("[probe] tracking %d totes" % len(pkgs), flush=True)

    recycle, rcount = _make_recycler(stage, _respawn_point(stage), verbose=True)
    state = {"seen_branch": set(), "seen_bin": set(), "last": {}}

    def on_step(f):
        recycle(f)
        if f % 60:
            return
        parts = []
        for p in pkgs:
            t = p.GetAttribute("xformOp:translate").Get()
            if t is None:
                continue
            nm = p.GetName()
            x, y, z = float(t[0]), float(t[1]), float(t[2])
            # kinematic flag + linear velocity: a tote that teleported but was
            # never handed back to the solver reads kin=1, and one that PhysX
            # owns but is blocked reads kin=0 with v~0. Those look identical
            # in a position dump, which is why four position-only guesses in a
            # row failed to explain the stalls.
            kin = p.GetAttribute("physics:kinematicEnabled")
            kv = "?" if not (kin and kin.IsValid()) else ("1" if kin.Get() else "0")
            vel = p.GetAttribute("physics:velocity")
            vy = float(vel.Get()[1]) if (vel and vel.HasValue()) else float("nan")
            parts.append("%s(%6.2f,%6.2f,%5.2f k%s vy%6.2f)"
                         % (nm, x, y, z, kv, vy))
            # east of the trunk = it took the branch
            if x > LINE_X + 1.20:
                state["seen_branch"].add(nm)
            # below roller height out east = it dropped into the bin
            if z < 0.60 and x > LINE_X + 3.00:
                state["seen_bin"].add(nm)
            state["last"][nm] = y
        print("[probe] f=%-5d %s" % (f, "  ".join(parts)), flush=True)
        if f % 600 == 0:
            print("[probe]   branch=%d bin=%d recycled=%d"
                  % (len(state["seen_branch"]), len(state["seen_bin"]),
                     rcount[0]), flush=True)

    keys = [(0.0, ORBIT_CENTER, ORBIT_TARGET), (1.0, ORBIT_CENTER, ORBIT_TARGET)]
    return FOCUS_CENTER, FOCUS_DIAG, on_step, keys


# ---------------------------------------------------------------------------
# SURVEY -- 4 frames from 4 high wide angles. Not a deliverable: this is how
# the free floor east of the line gets picked for the layout move, instead of
# guessing a LINE_X and finding out after a 5-minute rebuild.
# ---------------------------------------------------------------------------
SURVEY_EYES = [
    ((-12.0, -30.0, 16.0), (-16.0, -8.0, 1.0)),    # from the south, looking N
    ((2.0, -6.0, 18.0), (-18.0, -8.0, 1.0)),       # from the east, looking W
    ((-14.0, 22.0, 18.0), (-16.0, -6.0, 1.0)),     # from the north, looking S
    ((-14.0, -8.0, 34.0), (-14.0, -8.0, 0.0)),     # plan view straight down
]


def build_survey(app):
    _open(app, len(SURVEY_EYES))
    from lib.cine_capture_core import ensure_camera   # type: ignore

    def on_step(f):
        eye, tgt = SURVEY_EYES[min(f, len(SURVEY_EYES) - 1)]
        ensure_camera(pos=eye, target=tgt, focal_length=18.0)
        print("[tote_flow/survey] f=%d eye=%s" % (f, eye), flush=True)

    keys = [(0.0, SURVEY_EYES[0][0], SURVEY_EYES[0][1]),
            (1.0, SURVEY_EYES[0][0], SURVEY_EYES[0][1])]
    return FOCUS_CENTER, 60.0, on_step, keys


# ---------------------------------------------------------------------------
# OVERVIEW -- the deliverable wide. Smooth analytic orbit, no cuts.
# ---------------------------------------------------------------------------
def build_overview(app, total=1800):
    stage = _open(app, total)
    from lib.cine_capture_core import ensure_camera   # type: ignore
    recycle, rcount = _make_recycler(stage, _respawn_point(stage))

    def on_step(f):
        recycle(f)
        ensure_camera(pos=_orbit_eye(f / float(FPS)), target=ORBIT_TARGET,
                      focal_length=ORBIT_FOCAL)
        if f % 300 == 0:
            print("[tote_flow/overview] f=%d recycled=%d"
                  % (f, rcount[0]), flush=True)

    keys = [(0.0, _orbit_eye(0.0), ORBIT_TARGET),
            (1.0, _orbit_eye(0.0), ORBIT_TARGET)]
    return ORBIT_CENTER, 52.0, on_step, keys
