"""shots: Unitree H1 humanoid walking the aisle beside the running tote line.

WHY H1 AND NOT G1 / Go2
  Isaac Sim 5.1 ships the G1
  and Go2 ASSETS (Isaac/Robots/Unitree/{G1,Go2,H1,...}) but only FOUR
  locomotion policies: Anymal, Franka, H1, Spot
  (Isaac/Samples/Policies/). There is no G1 or Go2 policy, so neither can
  walk out of the box. H1 is the one Unitree humanoid that both ships an
  asset AND has a trained checkpoint (h1_policy.pt), so it is the one
  that can honestly walk here. Say "H1, not G1" out loud.

WHY THE FOLLOWER IS SELF-CONTAINED
  Reusable follow-scenario code elsewhere hardcodes SpotFlatTerrainPolicy,
  and swapping the policy there is the wrong place to make this change.
  The follower below is ~40 lines against the policy's own
  forward(dt, command) interface, so this shot has no external dependency.

PHYSICS RATE
  H1's policy decimates internally (h1.py: `_policy_counter % _decimation`),
  so it wants a fine physics dt, while the capture wants ONE app.update()
  per video frame (substeps > 1 causes systematic black grabs -- see
  CAPTURE_NOTES 5.16). SimulationContext(physics_dt=1/200,
  rendering_dt=1/60) gives both: PhysX substeps internally, the capture loop
  still advances exactly 1/60 s per rendered frame.

WAYPOINT ADVANCE = PLANE CROSSING, NOT A DISTANCE SPHERE
  A sphere test is missed whenever the body's per-step travel exceeds the
  radius, and the follower then chases a waypoint it has already walked
  past. The dot-product test below fires the moment the body crosses the
  plane through the waypoint normal to the leg, so it cannot be skipped.

RUN
  python.bat lib/cine_capture_core.py --builder shots/robot_aisle.py:build \
      --frames 1260 --subframes 12 --out output/frames/robot_aisle \
      --asset-root <asset_root from config/assets.json>

ASCII-only prints (Isaac stdout is cp1252).
"""
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from shots.tote_flow import (          # noqa: E402
    LINE_X, FPS, _open, _make_recycler, _respawn_point)

H1_PRIM = "/World/H1"
H1_NAME = "H1"

# The aisle east of the fenced cell. The cell's east fence is at LINE_X+4.40
# (-9.60) and the forklift sits at LINE_X+6.30 (-7.70) with roughly a metre of
# body either side, so the walk line stands off at LINE_X+8.50 to clear both.
WALK_X = LINE_X + 8.50
WALK_Y0, WALK_Y1 = -2.0, -19.0
WAYPOINTS = [(WALK_X, WALK_Y0 - 4.25 * i) for i in range(5)]   # 17 m, 5 points

SPEED = 0.8              # m/s forward command; 17 m -> ~21 s -> ~1260 frames
YAW_GAIN = 1.6           # rad/s per rad of heading error
YAW_MAX = 1.0
TURN_IN_PLACE = math.radians(40.0)   # beyond this, stop and turn first
WP_PLANE_EPS = 0.05      # m past the plane counts as crossed
WARMUP_S = 1.5           # zero-command ticks so the policy settles on its feet

PHYS_HZ = 200.0


def _spawn_h1(app):
    """Reference H1 through its own policy wrapper so the checkpoint, the env
    yaml and the articulation all come from the same place."""
    from isaacsim.robot.policy.examples.robots import (  # type: ignore
        H1FlatTerrainPolicy)
    import numpy as np                                   # type: ignore

    start = np.array([WAYPOINTS[0][0], WAYPOINTS[0][1], 1.05])
    policy = H1FlatTerrainPolicy(prim_path=H1_PRIM, name=H1_NAME,
                                 position=start)
    for _ in range(12):
        app.update()
    print("[robot_aisle] H1 spawned at (%.2f,%.2f)" % (start[0], start[1]),
          flush=True)
    return policy


def _follower(policy):
    """Waypoint follower producing (v_x, v_y, w_z) for the policy."""
    import numpy as np                                   # type: ignore

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

        # PLANE CROSSING: the leg direction is the plane normal. Once the
        # body's projection onto that leg passes the waypoint, advance --
        # regardless of lateral offset or how fast the step was.
        px, py = WAYPOINTS[i - 1] if i > 0 else (x, y)
        lx, ly = tx - px, ty - py
        seg = math.hypot(lx, ly)
        if seg > 1e-6:
            nx, ny = lx / seg, ly / seg
            if (x - tx) * nx + (y - ty) * ny > -WP_PLANE_EPS:
                state["i"] += 1
                print("[robot_aisle] wp %d crossed at (%.2f,%.2f) t=%.1fs"
                      % (i, x, y, state["t"]), flush=True)
                if state["i"] >= len(WAYPOINTS):
                    state["done"] = True
                    return np.zeros(3, dtype=np.float32)
                tx, ty = WAYPOINTS[state["i"]]

        err = math.atan2(ty - y, tx - x) - yaw
        err = (err + math.pi) % (2.0 * math.pi) - math.pi
        wz = max(-YAW_MAX, min(YAW_MAX, YAW_GAIN * err))
        # turning hard? stop translating, or the arc overshoots the aisle
        vx = 0.0 if abs(err) > TURN_IN_PLACE else SPEED * math.cos(err)
        return np.array([vx, 0.0, wz], dtype=np.float32)

    return command, state


def build(app, total=1260):
    stage = _open(app, total)
    from lib.cine_capture_core import ensure_camera       # type: ignore
    from isaacsim.core.api.simulation_context import (    # type: ignore
        SimulationContext)

    policy = _spawn_h1(app)
    recycle, rcount = _make_recycler(stage, _respawn_point(stage))
    command, fstate = _follower(policy)

    sim = SimulationContext(physics_dt=1.0 / PHYS_HZ,
                            rendering_dt=1.0 / FPS,
                            stage_units_in_meters=1.0)
    sim.reset()
    policy.initialize()
    print("[robot_aisle] physics %.0f Hz, render %d fps" % (PHYS_HZ, FPS),
          flush=True)

    def on_physics(dt):
        policy.forward(dt, command(dt))

    sim.add_physics_callback("h1_walk", on_physics)

    # camera: south-east, looking north-west so the aisle and the conveyor
    # line are on the same diagonal. Slow push in over the shot.
    eye0 = (LINE_X + 15.0, -26.0, 4.2)
    eye1 = (LINE_X + 11.0, -22.0, 2.6)
    tgt0 = (LINE_X + 2.0, -12.0, 1.2)
    tgt1 = (LINE_X + 3.0, -13.0, 1.0)

    def lerp(a, b, u):
        return tuple(p + (q - p) * u for p, q in zip(a, b))

    def on_step(f):
        recycle(f)
        u = f / float(max(1, total - 1))
        u = u * u * (3.0 - 2.0 * u)                # smoothstep
        ensure_camera(pos=lerp(eye0, eye1, u), target=lerp(tgt0, tgt1, u),
                      focal_length=22.0)
        if f % 180 == 0:
            pos, _ = policy.robot.get_world_pose()
            print("[robot_aisle] f=%-5d h1=(%6.2f,%6.2f,%5.2f) wp=%d "
                  "recycled=%d" % (f, pos[0], pos[1], pos[2],
                                   fstate["i"], rcount[0]), flush=True)

    keys = [(0.0, eye0, tgt0), (1.0, eye1, tgt1)]
    return (LINE_X + 2.0, -12.0, 1.2), 34.0, on_step, keys
