"""BehaviorScript attached to /World/Scenario1/Packages by build_scene.py.

Keeps the line fed forever: a box that has landed in the drop bin (or fallen on
the floor) is parked back at the head of the conveyor after a short dwell, so a
long GUI Play or a long render never runs dry.

Recycling, not spawning: prims are created once at build time and only moved
(hard rule: every physics prim must exist before the timeline plays). Move happens
while the body is kinematic, then it flips back to dynamic, so it wakes with
zero velocity instead of inheriting the fall.

ASCII-only prints.
"""
from omni.kit.scripting import BehaviorScript
from pxr import Gf

# must match build_scene.py. LINE_X moved east out of the west bay.
# NOTE these three are only the FALLBACK respawn point: the scene authors
# omni:recycler:respawn on /World/Scenario1/Packages and _respawn_point()
# prefers it. That authored value carries the tote's pivot offset, which this
# constant cannot know.
LINE_X = -14.00
HEAD_Y = -1.57
ROLLER_TOP_Z = 0.769
# anything east of this is over/in the bin. Derived from LINE_X, not a baked
# absolute: the branch ends at LINE_X + 4.44 and the bin sits 0.65 m past it,
# so a tote 3.5 m east of the trunk is on its way into the bin and nowhere
# else. Left as -20.5 it would have marked the WHOLE line as "in the bin"
# after the move east, and every tote would recycle on frame 1.
BIN_X = LINE_X + 3.50
DWELL_S = 2.5         # how long a box rests in the bin before it is recycled
FLOOR_Z = 0.45        # below this it is off the rollers (in the bin or on the floor)
RESPAWN_GAP_S = 4.0   # minimum spacing between two respawns: the divert at the
                      # tee takes ~2-3 s per box, and a tighter feed queues
                      # boxes into the stopper until the lead one wedges


class PackageRecycler(BehaviorScript):
    def on_init(self):
        self._landed = {}          # prim path -> sim time it settled
        self._last_respawn = -99.0
        self._pkgs = []

    def on_play(self):
        self._pkgs = [c for c in self.prim.GetChildren()
                      if c.GetName().startswith("pkg_")]
        self._landed.clear()
        self._last_respawn = -99.0
        self._respawn_pt = self._respawn_point()
        print(f"[recycler] tracking {len(self._pkgs)} packages, respawn at "
              f"({self._respawn_pt[0]:.2f},{self._respawn_pt[1]:.2f},"
              f"{self._respawn_pt[2]:.2f})", flush=True)

    def _respawn_point(self):
        """Where a recycled box is put back. Authored per-scene as
        omni:recycler:respawn, because it is NOT always the line centre:

        in V2 the packages ride in on the spawn CURVE (west of the line), whose
        own driven slab walks them south-east onto the line. Respawning at the
        line centre drops a box under that same south-east slab, which flings it
        EAST off the belt -- so V2's scene authors the curve mouth here instead.
        V1 has no curve and no such attribute, so it falls back to the head of
        the straight line (the original behaviour)."""
        a = self.prim.GetAttribute("omni:recycler:respawn")
        if a and a.HasValue():
            v = a.Get()
            return Gf.Vec3d(v[0], v[1], v[2])
        return Gf.Vec3d(LINE_X, HEAD_Y, ROLLER_TOP_Z + 0.02)

    def on_update(self, current_time: float, delta_time: float):
        # RECYCLING RE-ENABLED for the tote build. It was switched off on
        # 16/07 because "the respawned box drops to the floor" -- with the
        # cartons that was a real symptom of a WRONG RESPAWN Z. The scene now
        # authors omni:recycler:respawn with the tote wrapper's own z (the KLT
        # pivot is at the bin centre, so the wrapper rides ~0.15 m above the
        # roller top), and a tote put back there wakes up ON the rollers.
        #
        # It has to be on here: with 3 totes at 7.2 m spacing on a 17 m line
        # at 1.2 m/s, the line runs dry about 26 s in, and a 20-30 s orbit
        # would spend its last third filming an empty conveyor.
        for p in self._pkgs:
            path = str(p.GetPath())
            t = p.GetAttribute("xformOp:translate").Get()
            if t is None:
                continue
            settled = (t[2] < FLOOR_Z) or (t[0] > BIN_X)
            if not settled:
                self._landed.pop(path, None)
                continue
            first = self._landed.setdefault(path, current_time)
            if current_time - first < DWELL_S:
                continue
            if current_time - self._last_respawn < RESPAWN_GAP_S:
                continue
            self._respawn(p, t)
            self._landed.pop(path, None)
            self._last_respawn = current_time

    def _respawn(self, prim, old):
        # packages are now referenced cardboard assets whose pivot is at the
        # BASE (the old plain cubes were centre-pivoted) -> no half-height term
        kin = prim.GetAttribute("physics:kinematicEnabled")
        kin.Set(True)                       # freeze, move, thaw -> zero velocity
        prim.GetAttribute("xformOp:translate").Set(self._respawn_pt)
        kin.Set(False)
        print(f"[recycler] {prim.GetName()} recycled from "
              f"({old[0]:.1f},{old[1]:.1f},{old[2]:.1f}) to the head of the line",
              flush=True)
