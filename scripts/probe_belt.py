"""probe_belt.py -- which stretches of the line are actually DRIVEN?

The recycler needs a respawn point that is (a) on a driven roller body and
(b) hidden. Two guesses have now failed: the spawn CURVE throws totes off
(angular surface velocity), and LINE_HEAD_Y-1.2 leaves them standing still,
which means it is not on a driven body at all.

Stop guessing. This lists every prim carrying PhysxSurfaceVelocityAPI with
its world bbox and its velocity, so the respawn point can be read off the
table instead of inferred from piece counts.

Renders nothing.

RUN
  <isaac>\\python.bat scripts/probe_belt.py
"""
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(SCRIPT_DIR)
SCENE = os.path.join(PROJ, "assets", "warehouse_tote_flow.usd").replace("\\", "/")
OUT = os.path.join(PROJ, "output", "belt_probe.txt")

_lines = []


def log(msg):
    _lines.append(str(msg))
    print("[belt] %s" % msg, flush=True)
    with open(OUT, "w", encoding="ascii", errors="replace") as fh:
        fh.write("\n".join(_lines) + "\n")


log("booting SimulationApp (headless, no render)")
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
for _ in range(60):
    app.update()
stage = ctx.get_stage()

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


log("")
log("=== prims carrying a surface velocity (the driven surfaces) ===")
rows = []
for prim in stage.Traverse():
    sv = prim.GetAttribute("physxSurfaceVelocity:surfaceVelocity")
    ang = prim.GetAttribute("physxSurfaceVelocity:surfaceAngularVelocity")
    has = (sv and sv.HasValue()) or (ang and ang.HasValue())
    if not has:
        continue
    b = wbox(prim)
    if not b:
        continue
    v = sv.Get() if (sv and sv.HasValue()) else None
    a = ang.Get() if (ang and ang.HasValue()) else None
    rows.append((b[1], prim.GetPath().pathString, b, v, a))

rows.sort(key=lambda r: -r[0])          # north -> south
for _y, path, b, v, a in rows:
    log("  %-52s" % path[-52:])
    log("        x[%7.2f,%7.2f] y[%7.2f,%7.2f] z[%5.2f,%5.2f]  lin=%s ang=%s"
        % (b[0], b[3], b[1], b[4], b[2], b[5],
           "(%.2f,%.2f,%.2f)" % tuple(v) if v is not None else "-",
           "(%.1f,%.1f,%.1f)" % tuple(a) if a is not None else "-"))

log("")
log("=== driven coverage along the line centreline ===")
LINE_X = -14.00
log("  sampling x=%.2f, y from -0.5 down to -22.0 every 0.25 m" % LINE_X)
covered = []
y = -0.5
while y > -22.01:
    hit = None
    for _y0, path, b, v, a in rows:
        if b[0] - 0.35 <= LINE_X <= b[3] + 0.35 and b[1] <= y <= b[4]:
            hit = (path.split("/")[-2] if "/" in path else path, v, a)
            break
    covered.append((round(y, 2), hit))
    y -= 0.25

run_start, run_name = None, None
for yv, hit in covered:
    name = hit[0] if hit else None
    ang = hit[2] if hit else None
    tag = "%s%s" % (name or "-", " ANGULAR" if ang and any(ang) else "")
    if tag != run_name:
        if run_name is not None:
            log("  y %7.2f .. %7.2f   %s" % (run_start, yv + 0.25, run_name))
        run_name, run_start = tag, yv
log("  y %7.2f .. %7.2f   %s" % (run_start, covered[-1][0], run_name))

log("")
log("wrote %s" % OUT)
app.close()
