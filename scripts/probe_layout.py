"""probe_layout.py -- where can the conveyor line GO?

Answers one question with numbers instead of eyeballed renders: for each
candidate LINE_X, how close does the run [LINE_HEAD_Y .. T_Y] come to any
existing warehouse geometry?

Renders nothing. Opens the built scene, walks every Imageable under
/World/Warehouse (the env) and every top-level prim, computes world bboxes,
then for a sweep of candidate x values reports the nearest blocker.

RUN
  <isaac>\\python.bat scripts/probe_layout.py

ASCII-only prints (Isaac stdout is cp1252).
"""
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(SCRIPT_DIR)
SCENE = os.path.join(PROJ, "assets", "warehouse_tote_flow.usd").replace("\\", "/")
OUT = os.path.join(PROJ, "output", "layout_probe.txt")

# the run we want to relocate, in its own local terms
RUN_Y0, RUN_Y1 = -20.5, -0.5      # head .. past the tee/bin
LINE_HALF_W = 0.75                # belt half-width + a little clearance
CELL_EAST = 4.40                  # cell fence reaches LINE_X + 4.40
BIN_EAST = 5.10                   # bin mouth reaches about LINE_X + 5.10
NEED_EAST = max(CELL_EAST, BIN_EAST) + 0.60     # total east footprint needed
NEED_WEST = 1.60                  # hood + curve reach west of the centreline
# anything whose bbox BOTTOM sits above this is overhead (ceiling panels, roof
# beams, light fixtures) and cannot obstruct a 0.30 m tote on a 0.77 m belt.
# The tallest thing the line carries is the 2.50 m cell fence.
WORK_Z = 2.60

CANDIDATES = [-24.92, -22.0, -20.0, -18.0, -16.0, -14.0, -12.0,
              -10.0, -8.0, -6.0, -4.0, -2.0, 0.0]

_lines = []


def log(msg):
    _lines.append(str(msg))
    print("[layout] %s" % msg, flush=True)
    with open(OUT, "w", encoding="ascii", errors="replace") as fh:
        fh.write("\n".join(_lines) + "\n")


log("booting SimulationApp (headless, no render)")
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
for _ in range(60):
    app.update()
stage = ctx.get_stage()
if stage is None:
    raise SystemExit("open_stage failed: %s" % SCENE)

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


def wbox(prim):
    try:
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        lo, hi = r.GetMin(), r.GetMax()
        return (lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])
    except Exception:
        return None


# ---------------------------------------------------------------------------
# 1. overall extents
# ---------------------------------------------------------------------------
log("")
log("=== top-level prims ===")
for child in stage.GetPseudoRoot().GetChildren():
    for p in child.GetChildren():
        b = wbox(p)
        if b:
            log("  %-42s x[%7.2f,%7.2f] y[%7.2f,%7.2f] z[%6.2f,%6.2f]"
                % (p.GetPath(), b[0], b[3], b[1], b[4], b[2], b[5]))

# ---------------------------------------------------------------------------
# 2. every sizeable blocker in the environment
# ---------------------------------------------------------------------------
wh = stage.GetPrimAtPath("/World/Warehouse")
blockers = []          # (name, x0,y0,x1,y1, z1)
if wh and wh.IsValid():
    for p in wh.GetChildren():
        b = wbox(p)
        if not b:
            continue
        # A blocker must occupy the WORKING VOLUME, so filter on the bbox
        # BOTTOM, not the top. Filtering on the top marks the ceiling panels
        # and roof beams (bottom z=8-9, top z=9) as blockers and every
        # candidate comes back BLOCKED -- which is what the first run did.
        if b[5] < 0.45:          # too short to obstruct anything
            continue
        if b[2] > WORK_Z:        # hangs entirely above the working volume
            continue
        blockers.append((p.GetName(), b[0], b[1], b[3], b[4], b[2], b[5]))
log("")
log("=== %d blockers inside the working volume (bottom < %.1f m) ==="
    % (len(blockers), WORK_Z))
for nm, x0, y0, x1, y1, z0, z1 in sorted(blockers, key=lambda r: r[1])[:60]:
    log("  %-34s x[%7.2f,%7.2f] y[%7.2f,%7.2f] z[%5.2f,%5.2f]"
        % (nm, x0, x1, y0, y1, z0, z1))

# ---------------------------------------------------------------------------
# 3. candidate sweep
# ---------------------------------------------------------------------------
log("")
log("=== candidate LINE_X sweep ===")
log("  corridor needed: x in [LINE_X-%.2f, LINE_X+%.2f], y in [%.1f,%.1f]"
    % (NEED_WEST, NEED_EAST, RUN_Y0, RUN_Y1))
log("  %-9s %-8s %s" % ("LINE_X", "verdict", "overlapping blockers"))
for cx in CANDIDATES:
    lo_x, hi_x = cx - NEED_WEST, cx + NEED_EAST
    hits = []
    for nm, x0, y0, x1, y1, _z0, _z1 in blockers:
        if x1 < lo_x or x0 > hi_x:
            continue
        if y1 < RUN_Y0 or y0 > RUN_Y1:
            continue
        hits.append(nm)
    verdict = "CLEAR" if not hits else "BLOCKED"
    log("  %-9.2f %-8s %s" % (cx, verdict,
                              ", ".join(sorted(set(hits))[:6]) or "-"))

log("")
log("wrote %s" % OUT)
app.close()
