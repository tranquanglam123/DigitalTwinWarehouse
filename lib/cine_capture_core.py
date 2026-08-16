"""
cine_capture_core.py — portable headless cinematic capture engine for Isaac Sim.

Battle-tested engine extracted from the portfolio capture pipeline (8 debug
rounds, 5 shipped clips, 0 black frames). Self-contained: no external repo
imports. Use as a library from your sim script, or via the CLI with a builder
file.

LIBRARY USE (inside a script run by Isaac's python.bat):
    from cine_capture_core import boot, capture_sequence, orbit_keys, ...
    app = boot(asset_root=r"C:/.../Assets/Isaac/5.1")
    # ... build your scene (omni.* imports AFTER boot) ...
    capture_sequence(app, cam_keys, out_dir, total_frames=560)

CLI USE (builder file exports build(app) -> (center, diag[, on_step[, cam_keys]])):
    <isaac>\\python.bat cine_capture_core.py --builder my_shot.py:build \
        --frames 560 --w 1080 --h 1920 --focal 21 --out frames/my_shot

Read the companion GOTCHAS.md before debugging anything: every weird failure
mode here (silent exit-0 death, all-white frames, black-frame storms, tele
lens surprise) already happened and has a known cause.
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import time

CINECAM = "/World/CineCam"


def log(msg: str) -> None:
    print(f"[cine {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Cinematic camera math (pure python — safe before boot)
# ---------------------------------------------------------------------------
def lerp3(a, b, s):
    return (a[0] + (b[0] - a[0]) * s,
            a[1] + (b[1] - a[1]) * s,
            a[2] + (b[2] - a[2]) * s)


_lerp3 = lerp3  # back-compat alias


def eval_camera(keys, u):
    """keys: sorted [(t, eye, target), ...] with t in [0,1]. Smoothstep-eased.
    Two keys back-to-back (t, ...), (t+eps, ...) = a hard cut."""
    if u <= keys[0][0]:
        return keys[0][1], keys[0][2]
    if u >= keys[-1][0]:
        return keys[-1][1], keys[-1][2]
    for i in range(len(keys) - 1):
        t0, e0, g0 = keys[i]
        t1, e1, g1 = keys[i + 1]
        if t0 <= u <= t1:
            s = (u - t0) / (t1 - t0)
            s = s * s * (3 - 2 * s)  # smoothstep ease in/out
            return lerp3(e0, e1, s), lerp3(g0, g1, s)
    return keys[-1][1], keys[-1][2]


def orbit_keys(center, radius, height, turns=0.75, n=8, t0=0.0, t1=1.0):
    """Slow arc around `center`. n+1 keyframes, eased between each."""
    keys = []
    for i in range(n + 1):
        f = i / n
        a = 2 * math.pi * turns * f
        eye = (center[0] + radius * math.cos(a),
               center[1] + radius * math.sin(a),
               center[2] + height)
        keys.append((t0 + (t1 - t0) * f, eye, center))
    return keys


def dolly_in_keys(center, radius_far, radius_near, height, t0=0.0, t1=1.0, ang=0.6):
    """Push in toward center along a fixed bearing (`ang` radians)."""
    ef = (center[0] + radius_far * math.cos(ang),
          center[1] + radius_far * math.sin(ang), center[2] + height)
    en = (center[0] + radius_near * math.cos(ang),
          center[1] + radius_near * math.sin(ang), center[2] + height * 0.6)
    return [(t0, ef, center), (t1, en, center)]


def concat_shots(*shot_lists):
    """Splice multiple keyframe lists over the full [0,1] timeline, evenly."""
    n = len(shot_lists)
    out = []
    for i, keys in enumerate(shot_lists):
        lo, hi = i / n, (i + 1) / n
        for (t, e, g) in keys:
            out.append((lo + (hi - lo) * t, e, g))
    return sorted(out, key=lambda k: k[0])


def visible_height(dist, focal, res=(1920, 1080), h_aperture=20.955):
    """Meters of world visible vertically at `dist` for a given focal.
    Isaac default camera h-aperture is NARROW (20.955mm): 30-38mm reads tele.
    Vertical aperture scales with the render aspect: v_ap = h_ap * H / W —
    so a 9:16 vertical render sees ~3.2x MORE vertical world than 16:9."""
    v_ap = h_aperture * res[1] / res[0]
    return dist * v_ap / focal


# ---------------------------------------------------------------------------
# Camera prim (Z-up look-at) — embedded, no external deps
# ---------------------------------------------------------------------------
def ensure_camera(camera_path=CINECAM, pos=(8.0, -8.0, 6.0),
                  target=(0.0, 0.0, 0.0), up_world=(0.0, 0.0, 1.0),
                  focal_length: float = 24.0) -> str:
    """Create or reposition a Camera prim with a Z-up look-at transform.
    Focal length is set on EVERY call (the original helper only set it at
    creation — mid-shot focal changes silently no-op'd)."""
    import omni.usd  # type: ignore
    from pxr import Gf, UsdGeom  # type: ignore

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(camera_path)
    if not prim or not prim.IsValid():
        cam = UsdGeom.Camera.Define(stage, camera_path)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))
    UsdGeom.Camera(stage.GetPrimAtPath(camera_path)).GetFocalLengthAttr().Set(
        float(focal_length))

    eye = Gf.Vec3d(*pos)
    look = Gf.Vec3d(*target)
    up = Gf.Vec3d(*up_world)
    forward = (look - eye).GetNormalized()
    right = Gf.Cross(forward, up).GetNormalized()
    cam_up = Gf.Cross(right, forward)

    # USD is row-major (`v * M`): row i = where local +i axis lands in world.
    rot = Gf.Matrix3d(
        right[0], right[1], right[2],
        cam_up[0], cam_up[1], cam_up[2],
        -forward[0], -forward[1], -forward[2],
    )
    m = Gf.Matrix4d()
    m.SetIdentity()
    m.SetRotateOnly(rot)
    m.SetTranslateOnly(eye)

    xformable = UsdGeom.Xformable(stage.GetPrimAtPath(camera_path))
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(m)
    return camera_path


# ---------------------------------------------------------------------------
# Isaac boot + scene helpers
# ---------------------------------------------------------------------------
def boot(headless=True, ext_folder=None, ext_id=None, res=(1920, 1080),
         asset_root=None, extra_settings=None):
    """SimulationApp bootstrap with the two settings that prevent silent death:
    * asset_root local override — else get_assets_root_path() stats a dead
      Nucleus ~40s and the app exits 0 with no traceback (fastShutdown).
    * RTX auto-exposure OFF — else dark/wide scenes render PURE WHITE.
    asset_root: arg > $ISAAC_ASSET_ROOT env > (warn, hope Nucleus works).
    """
    log("importing isaacsim.SimulationApp (long step)...")
    from isaacsim import SimulationApp  # noqa: E402  MUST be first omni import
    root = asset_root or os.environ.get("ISAAC_ASSET_ROOT")
    extra = ["--/rtx/post/histogram/enabled=false"]
    if root:
        extra.append(f"--/persistent/isaac/asset_root/default={root}")
    else:
        log("WARNING: no asset_root — Nucleus lookups may hang/kill the app")
    if extra_settings:
        extra += list(extra_settings)
    if ext_folder:
        extra += ["--ext-folder", ext_folder]
    app = SimulationApp({
        "headless": headless,
        "width": res[0], "height": res[1],
        "renderer": "RaytracedLighting",
        "extra_args": extra,
    })
    log("SimulationApp ready.")
    if ext_id:
        import omni.kit.app  # type: ignore
        mgr = omni.kit.app.get_app().get_extension_manager()
        ok = mgr.set_extension_enabled_immediate(ext_id, True)
        log(f"enable extension {ext_id!r} -> {ok}")
        for _ in range(3):
            app.update()
    return app


def dump_stage(root="/"):
    """Diagnostic: log every prim with type, visibility, purpose, world bbox."""
    import omni.usd  # type: ignore
    from pxr import Usd, UsdGeom  # type: ignore
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    log(f"---- stage dump from {root} ----")
    for prim in stage.Traverse():
        img = UsdGeom.Imageable(prim)
        vis = img.ComputeVisibility() if img else "-"
        purpose = img.ComputePurpose() if img else "-"
        extra = ""
        if prim.IsA(UsdGeom.Boundable) or prim.IsA(UsdGeom.Xformable):
            try:
                r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if not r.IsEmpty():
                    lo, hi = r.GetMin(), r.GetMax()
                    extra = (f" bbox=({lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f})"
                             f"..({hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f})")
            except Exception:
                pass
        log(f"  {prim.GetPath()} [{prim.GetTypeName()}] vis={vis} purpose={purpose}{extra}")
    log("---- end dump ----")


def scene_bounds(root="/World"):
    """(center, diagonal) of the world-space bbox under `root`."""
    import omni.usd  # type: ignore
    from pxr import Usd, UsdGeom  # type: ignore
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(root)).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    center = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
    return center, float((hi - lo).GetLength())


def add_studio_lights(key_intensity=3000.0, dome_intensity=300.0,
                      rim_intensity=1200.0, fill_intensity=0.0):
    """Dark-tech studio look: dim cool dome + warm angled key + cool rim
    (+ optional soft fill). NOTE: raw frames still render LIGHT grey — the
    final dark look comes from the ffmpeg grade (contrast+vignette)."""
    import omni.usd  # type: ignore
    from pxr import UsdLux, UsdGeom, Gf  # type: ignore
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/CineDome")
    dome.CreateIntensityAttr(dome_intensity)
    dome.CreateColorAttr(Gf.Vec3f(0.40, 0.45, 0.60))
    key = UsdLux.DistantLight.Define(stage, "/World/CineKey")
    key.CreateIntensityAttr(key_intensity)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.90))
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 20.0, 0.0))
    rim = UsdLux.DistantLight.Define(stage, "/World/CineRim")
    rim.CreateIntensityAttr(rim_intensity)
    rim.CreateColorAttr(Gf.Vec3f(0.55, 0.70, 1.00))
    UsdGeom.Xformable(rim.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 160.0))
    if fill_intensity:
        fill = UsdLux.DistantLight.Define(stage, "/World/CineFill")
        fill.CreateIntensityAttr(float(fill_intensity))
        fill.CreateColorAttr(Gf.Vec3f(0.85, 0.90, 1.0))
        UsdGeom.Xformable(fill.GetPrim()).AddRotateXYZOp().Set(
            Gf.Vec3f(-35.0, 0.0, 200.0))


def make_grid_mesh(stage, path, size=4.0, div=96,
                   color=(0.22, 0.24, 0.28)):
    """Tessellated square plane (div x div quads) centered at origin, z=0.
    Dense enough for per-vertex effects (heatmaps, deformation readouts)."""
    from pxr import UsdGeom, Gf, Vt  # type: ignore
    mesh = UsdGeom.Mesh.Define(stage, path)
    n = div + 1
    step = size / div
    half = size / 2.0
    pts = [Gf.Vec3f(-half + i * step, -half + j * step, 0.0)
           for j in range(n) for i in range(n)]
    counts, indices = [], []
    for j in range(div):
        for i in range(div):
            a = j * n + i
            counts.append(4)
            indices.extend([a, a + 1, a + n + 1, a + n])
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(pts))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(counts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
    mesh.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-half, -half, -0.01),
                                            Gf.Vec3f(half, half, 0.01)]))
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    return mesh


def bind_dark_floor_material(stage, prim, diffuse=(0.045, 0.05, 0.065),
                             roughness=0.85):
    """Big flat slabs render as bright specular mirrors under a dome light —
    bind a rough dark PreviewSurface so the floor reads matte."""
    from pxr import Gf, Sdf, UsdShade  # type: ignore
    mat = UsdShade.Material.Define(stage, "/World/Looks/CineFloorMat")
    sh = UsdShade.Shader.Define(stage, "/World/Looks/CineFloorMat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    return mat


# ---------------------------------------------------------------------------
# Capture loop — the hardened core
# ---------------------------------------------------------------------------
def capture_sequence(app, cam_keys, out_dir, total_frames=180,
                     res=(1920, 1080), subframes=12, phys_substeps=1,
                     focal=32.0, warmup=8, on_step=None, camera_path=CINECAM):
    """Play the timeline and write one PNG per frame while flying `cam_keys`.

    * phys_substeps: app.update() calls per captured frame. KEEP AT 1 for
      physics shots — at 2 the annotator returns systematic black buffers.
      Faster motion = tune the physics (speeds) + raise total_frames.
    * subframes: RT accumulation per frame (higher = cleaner, slower).
    * on_step(frame): per-substep callback — tick controllers here; it may
      also call ensure_camera() to OVERRIDE the keyed camera (chase cams).
    * Black-frame guard: 12 retries, each = 0.3s sleep + app.update() kick +
      re-render. Plain re-renders alone stay black when the desktop fights
      for the GPU; the kick resyncs the annotator pipeline.
    """
    import omni.replicator.core as rep  # type: ignore
    import omni.timeline  # type: ignore
    from PIL import Image  # type: ignore
    import numpy as np  # type: ignore

    os.makedirs(out_dir, exist_ok=True)

    # Attach the rgb annotator ONCE (reused every frame -> fast).
    ensure_camera(camera_path, pos=cam_keys[0][1], target=cam_keys[0][2],
                  focal_length=focal)
    rp = rep.create.render_product(camera_path, res)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])

    tl = omni.timeline.get_timeline_interface()
    for _ in range(warmup):
        app.update()
    # Throwaway orchestrator steps BEFORE the run: the annotator's first
    # buffers are mis-shaped and crash PIL otherwise.
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0,
                              pause_timeline=False)
    log("timeline.play()")
    tl.play()

    t0 = time.time()
    for f in range(total_frames):
        u = f / max(1, total_frames - 1)
        eye, tgt = eval_camera(cam_keys, u)
        ensure_camera(camera_path, pos=eye, target=tgt, focal_length=focal)
        for _ in range(phys_substeps):
            if on_step:
                on_step(f)
            app.update()
        rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0,
                              pause_timeline=False)
        arr = np.asarray(rgb.get_data())
        for retry in range(12):
            bad_shape = (arr.ndim != 3 or arr.shape[0] != res[1]
                         or arr.shape[1] != res[0])
            black = (not bad_shape) and int(arr[..., :3].max()) < 6
            if not bad_shape and not black:
                break
            log(f"frame {f}: bad grab (shape={getattr(arr, 'shape', None)}, "
                f"black={black}), re-render {retry + 1}/12")
            time.sleep(0.3)
            app.update()
            rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0,
                                  pause_timeline=False)
            arr = np.asarray(rgb.get_data())
        else:
            raise RuntimeError(f"frame {f}: annotator kept returning bad frames")
        arr = np.ascontiguousarray(arr)
        mode = "RGBA" if arr.shape[2] == 4 else "RGB"
        Image.fromarray(arr, mode=mode).save(
            os.path.join(out_dir, f"frame_{f:05d}.png"))
        if f % 20 == 0:
            log(f"frame {f}/{total_frames}  ({time.time()-t0:.0f}s)")

    tl.stop()
    for _ in range(5):
        app.update()
    log(f"done: {total_frames} frames -> {out_dir}")


# ---------------------------------------------------------------------------
# Headless physics probe — tune BEFORE you render
# ---------------------------------------------------------------------------
def probe(app, on_step, frames=320, report=None, every=20):
    """Step physics WITHOUT rendering and log state. ~50x faster than a render
    run — use it to tune speeds/trajectories so the first real render already
    fits the frame budget. `report(f) -> str` supplies the log line."""
    import omni.timeline  # type: ignore
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for f in range(frames):
        if on_step:
            on_step(f)
        app.update()
        if report and f % every == 0:
            log(f"probe f={f} {report(f)}")
    tl.stop()


# ---------------------------------------------------------------------------
# CLI: run a builder file
# ---------------------------------------------------------------------------
def _load_builder(spec_str):
    path, _, fn_name = spec_str.partition(":")
    fn_name = fn_name or "build"
    spec = importlib.util.spec_from_file_location("shot_builder", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["shot_builder"] = mod
    spec.loader.exec_module(mod)
    return getattr(mod, fn_name)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--builder", required=True,
                    help="path/to/shot.py[:build_fn] — fn(app) -> "
                         "(center, diag[, on_step[, cam_keys]])")
    ap.add_argument("--out", required=True, help="frame output dir")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--subframes", type=int, default=12)
    ap.add_argument("--phys-substeps", type=int, default=1)
    ap.add_argument("--w", type=int, default=1920)
    ap.add_argument("--h", type=int, default=1080)
    ap.add_argument("--focal", type=float, default=24.0)
    ap.add_argument("--warmup", type=int, default=8,
                    help="app.update()s before capture; raise for heavy NuRec "
                         "splats that render black until fully streamed in")
    ap.add_argument("--asset-root", default=None)
    ap.add_argument("--ext-folder", default=None)
    ap.add_argument("--ext-id", default=None)
    ap.add_argument("--headed", action="store_true")
    ap.add_argument("--dump", action="store_true")
    ap.add_argument("--probe", type=int, default=0,
                    help="N>0: physics-only probe for N frames, no render")
    args = ap.parse_args()

    build_fn = _load_builder(args.builder)
    res = (args.w, args.h)
    app = boot(headless=not args.headed, ext_folder=args.ext_folder,
               ext_id=args.ext_id, res=res, asset_root=args.asset_root)
    try:
        scene = build_fn(app)
        center, diag = scene[0], scene[1]
        on_step = scene[2] if len(scene) > 2 else None
        custom_keys = scene[3] if len(scene) > 3 else None
        for _ in range(5):
            app.update()
        if args.dump:
            dump_stage()
        if args.probe > 0:
            probe(app, on_step, frames=args.probe)
            return
        cam = custom_keys or concat_shots(
            orbit_keys(center, radius=diag * 0.95, height=diag * 0.45, turns=0.5),
            dolly_in_keys(center, radius_far=diag * 0.9,
                          radius_near=diag * 0.45, height=diag * 0.35),
        )
        capture_sequence(app, cam, args.out, total_frames=args.frames, res=res,
                         subframes=args.subframes,
                         phys_substeps=args.phys_substeps,
                         focal=args.focal, warmup=args.warmup, on_step=on_step)
    except BaseException:
        # fastShutdown in app.close() can kill the process before Python's
        # default traceback printer runs — log it explicitly first.
        import traceback
        log("FATAL:\n" + traceback.format_exc())
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
