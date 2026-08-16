"""shots: the overview orbit, with a Spot quadruped trotting beside the line.

Same scene and same camera move as tote_flow.py:build_overview -- this only
adds the dog, so v2 and v3 are comparable frame for frame.

WHY SPOT AND NOT H1/G1
  Spot is Boston Dynamics, not Unitree. Spot is here because
  it is the one legged robot already PROVEN to walk in
  this setup (probe: 9.7 m in 20 s at 0.49 m/s, fell_over=None). H1 is the
  honest Unitree stand-in and the follower below is policy-agnostic -- swapping
  SpotFlatTerrainPolicy for H1FlatTerrainPolicy is a one-line change -- but H1
  is bipedal and has not been run here yet. Do not describe this clip as a
  Unitree robot.

PHYSICS RATE
  The policy wants a fine dt; the capture wants ONE app.update() per video
  frame, because substeps > 1 causes systematic black grabs (CAPTURE_NOTES
  5.16). SimulationContext(physics_dt=1/200, rendering_dt=1/60) gives both.
  That moves the whole scene from 60 Hz to 200 Hz, including the conveyor,
  whose divert was tuned at 60 -- probe it before rendering.

WAYPOINT ADVANCE = PLANE CROSSING
  Dot product against the leg direction, not a distance sphere. A sphere is
  missed whenever per-step travel exceeds the radius and the follower then
  chases a point it has already walked past.

RUN
  python.bat lib/cine_capture_core.py \
      --builder shots/overview_robot.py:build_probe --probe 2400 \
      --out output/frames/_ignore --asset-root <root>
  python.bat lib/cine_capture_core.py --builder shots/overview_robot.py:build \
      --frames 1800 --subframes 12 --out output/frames/overview_robot ...

ASCII-only prints (Isaac stdout is cp1252).
"""
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from shots.tote_flow import (           # noqa: E402
    LINE_X, LINE_HEAD_Y, FPS, ORBIT_TARGET, ORBIT_FOCAL,
    _open, _orbit_eye, _make_recycler, _respawn_point)

DOG_PRIM = "/World/Spot"

# West of the line, on open floor. The fenced cell, its pallet and the forklift
# are all EAST, so the west aisle is the only side a robot can walk without
# threading between props. probe_layout put the bay wall/pillar line at
# x=-17.33 and the hood reaches x=-15.4, so -16.3 splits the gap.
WALK_X = LINE_X - 2.30
WAYPOINTS = [(WALK_X, -3.0 - 4.0 * i) for i in range(5)]    # 16 m

SPEED = 0.7
YAW_GAIN = 1.6
YAW_MAX = 1.0
TURN_IN_PLACE = math.radians(40.0)
WP_PLANE_EPS = 0.05
WARMUP_S = 1.5
PHYS_HZ = 200.0


def _spawn_dog(app):
    from isaacsim.robot.policy.examples.robots import (   # type: ignore
        SpotFlatTerrainPolicy)
    import numpy as np                                    # type: ignore
    start = np.array([WAYPOINTS[0][0], WAYPOINTS[0][1], 0.80])
    policy = SpotFlatTerrainPolicy(prim_path=DOG_PRIM, name="Spot",
                                   position=start)
    for _ in range(12):
        app.update()
    print("[overview_robot] Spot at (%.2f,%.2f)" % (start[0], start[1]),
          flush=True)
    return policy


def _follower(policy):
    """Waypoint follower emitting (v_x, v_y, w_z). Policy-agnostic: any
    PolicyController with .robot and .forward(dt, command) works."""
    import numpy as np                                    # type: ignore
    state = {"i": 0, "t": 0.0, "done": False}

    def command(dt):
        state["t"] += dt
        if state["t"] < WARMUP_S or state["done"]:
            return np.zeros(3, dtype=np.float32)

        pos, quat = policy.robot.get_world_pose()
        x, y = float(pos[0]), float(pos[1])
        w, qx, qy, qz = (float(quat[0]), float(quat[1]),
                         float(quat[2]), float(quat[3]))
        yaw = math.atan2(2.0 * (w * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))

        i = state["i"]
        tx, ty = WAYPOINTS[i]
        px, py = WAYPOINTS[i - 1] if i > 0 else (x, y)
        lx, ly = tx - px, ty - py
        seg = math.hypot(lx, ly)
        if seg > 1e-6:
            nx, ny = lx / seg, ly / seg
            if (x - tx) * nx + (y - ty) * ny > -WP_PLANE_EPS:
                state["i"] += 1
                print("[overview_robot] wp %d at (%.2f,%.2f) t=%.1fs"
                      % (i, x, y, state["t"]), flush=True)
                if state["i"] >= len(WAYPOINTS):
                    state["done"] = True
                    return np.zeros(3, dtype=np.float32)
                tx, ty = WAYPOINTS[state["i"]]

        err = math.atan2(ty - y, tx - x) - yaw
        err = (err + math.pi) % (2.0 * math.pi) - math.pi
        wz = max(-YAW_MAX, min(YAW_MAX, YAW_GAIN * err))
        vx = 0.0 if abs(err) > TURN_IN_PLACE else SPEED * math.cos(err)
        return np.array([vx, 0.0, wz], dtype=np.float32)

    return command, state


def _common(app, total, verbose):
    stage = _open(app, total)
    from isaacsim.core.api.simulation_context import (     # type: ignore
        SimulationContext)
    policy = _spawn_dog(app)
    feed, fcount = _make_recycler(stage, _respawn_point(stage), verbose=verbose)
    command, fstate = _follower(policy)

    sim = SimulationContext(physics_dt=1.0 / PHYS_HZ,
                            rendering_dt=1.0 / FPS,
                            stage_units_in_meters=1.0)
    sim.reset()
    policy.initialize()
    sim.add_physics_callback("dog_walk", lambda dt: policy.forward(dt,
                                                                   command(dt)))
    print("[overview_robot] physics %.0f Hz, render %d fps"
          % (PHYS_HZ, FPS), flush=True)
    return stage, policy, feed, fcount, fstate


def build_probe(app):
    """Physics only. Two questions: does the dog walk the aisle without
    falling, and does the tee still divert now the scene runs at 200 Hz
    instead of the 60 Hz it was tuned at?"""
    stage, policy, feed, fcount, fstate = _common(app, 2400, True)
    root = stage.GetPrimAtPath("/World/Scenario1/Packages")
    pkgs = [c for c in root.GetChildren()
            if c.GetName().startswith("pkg_")] if root else []
    seen = set()

    def on_step(f):
        feed(f)
        if f % 120:
            return
        for p in pkgs:
            t = p.GetAttribute("xformOp:translate").Get()
            if t is not None and t[0] > LINE_X + 1.20:
                seen.add(p.GetName())
        pos, _ = policy.robot.get_world_pose()
        print("[probe] f=%-5d dog=(%6.2f,%6.2f,%5.2f) wp=%d branch=%d fed=%d"
              % (f, pos[0], pos[1], pos[2], fstate["i"], len(seen), fcount[0]),
              flush=True)

    keys = [(0.0, _orbit_eye(0.0), ORBIT_TARGET),
            (1.0, _orbit_eye(0.0), ORBIT_TARGET)]
    return (LINE_X + 1.9, -10.5, 1.2), 30.0, on_step, keys


def build(app, total=1800):
    stage, policy, feed, fcount, fstate = _common(app, total, False)
    from lib.cine_capture_core import ensure_camera        # type: ignore

    def on_step(f):
        feed(f)
        ensure_camera(pos=_orbit_eye(f / float(FPS)), target=ORBIT_TARGET,
                      focal_length=ORBIT_FOCAL)
        if f % 300 == 0:
            pos, _ = policy.robot.get_world_pose()
            print("[overview_robot] f=%-5d dog=(%6.2f,%6.2f) wp=%d fed=%d"
                  % (f, pos[0], pos[1], fstate["i"], fcount[0]), flush=True)

    keys = [(0.0, _orbit_eye(0.0), ORBIT_TARGET),
            (1.0, _orbit_eye(0.0), ORBIT_TARGET)]
    return (LINE_X + 1.9, -10.5, 1.2), 30.0, on_step, keys
