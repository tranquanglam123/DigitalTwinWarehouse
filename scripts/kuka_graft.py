"""The grafted KUKA: the user's ArchViz robot body on Isaac's KR210 skeleton.

Import this from any scene script that needs an arm that BOTH looks like the
KUKA in the user's welding cell and can actually pick things up. Boot Isaac
first -- this module imports pxr at import time.

    import kuka_graft
    kuka_graft.graft(stage, app, "/World/Scenario1/Head/arm", (x, y, 0.0), 180)

WHY THIS EXISTS
  Isaac's library arms cannot look like the user's robot (the best of them ships
  30 meshes / 4 materials; the cell robot has 178 / 34) and the sample robot
  cannot move (zero joints). So Isaac's KR210 supplies the articulation and the
  sample supplies every visible surface.

HOW
  The shell's meshes are RE-PARENTED under the skeleton's rigid bodies, one
  group per link. PhysX moves the bodies, USD carries the children -- the graft
  is baked into the file, needs no per-frame code, and cannot drift. (Copying
  joint angles into the shell every frame was the obvious alternative and was
  rejected: it dies the moment the user opens the scene in the GUI without a
  script running.)

  The two machines are NOT the same size -- the shell is a KR120 R2500, the
  skeleton a KR210 L150, and their forearms differ by 50%. Stretching the shell
  to fit would oval out the elbow and wrist housings, so the BONES ARE RESHAPED
  ONTO THE SHELL instead (see LINKS below), and a matching URDF is emitted so
  RMPflow solves for the arm we actually built. Reach 2.70 m -> ~2.45 m, still
  well past the 1.53 m the pallet needs.

  All of it was measured, not guessed: shell_graft_probe.py prints the numbers.

ASCII-only prints (Isaac stdout is cp1252).
"""
import json
import math
import os
import re
import shutil

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
with open(os.path.join(REPO_ROOT, "config", "assets.json"), encoding="utf-8") as _f:
    _CFG = json.load(_f)
ASSET_ROOT = _CFG["asset_root"]
_NV_PACKS = _CFG.get("nvidia_packs_root", "") or os.environ.get("TF_NVIDIA_ASSETS", "")
_ISAAC_ROOT = os.path.dirname(_CFG["isaac_python"])

SKEL_USD = ASSET_ROOT + "/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd"
SHELL_USD = (_NV_PACKS +
             "/USDExplorerSample/Usd_Explorer/Samples/Examples/2023_2/Factory/"
             "Source_Files/Visual_Components/raw_cell_w_animation.usd")
SHELL_ROOT = "/World/welding_cell/root/RobotPedestal_U20__U23_10"
SHELL_ARM = SHELL_ROOT + "/Link1/KR_U20_120_U20_R2500_U20_pro_U20__U23_11"
PEDESTAL_PRIM = SHELL_ROOT + "/geometry"     # the 39-mesh base the arm stands on
PEDESTAL_TOP_Z = 0.97                        # measured: arms mount here
MM = 0.001                                   # the cell is authored in millimetres

REACH = 2.45          # m, the reshaped arm (was 2.70 on the stock KR210)

# ---------------------------------------------------------------------------
# THE ONE TABLE THAT MATTERS -- the shell's kinematics, measured off its A-nodes,
# in the robot's own base frame (metres, mount plane at z=0, arm pointing +X at
# the zero pose). The USD joints AND the URDF are both rebuilt from these, so the
# articulation, the IK and the visible geometry cannot disagree.
#
#   pivot  where the JOINT sits (the bone's frame origin)
#   sh     where the shell's A-node sits -- the casting is modelled around THIS
#          point, so the mesh group is offset by (sh - pivot). Identical
#          everywhere except link_1: A1 turns about a VERTICAL axis, so its pivot
#          height is kinematically free, and the KR210 puts it 331 mm up while
#          the shell models the column from the mount plane. Miss this and the
#          arm floats 331 mm above its own base.
#   R      rotation taking the shell link's frame into the skeleton link's frame.
#          The shell's zero pose has the forearm pointing UP, the skeleton's has
#          it pointing FORWARD -- hence the 90 deg about Y from the elbow out.
#          A5/A6 additionally bake a frame flip (Rot(X,-/+90)) on top of their
#          joint rotation, which is why their quaternions do not read as a plain
#          axis; factored out, the two chains match joint-for-joint.
#   src    which prim of the sample robot supplies this link's meshes
#   prune  the child A-node to switch off after referencing (it belongs to the
#          NEXT link, and a reference drags the whole subtree in)
# ---------------------------------------------------------------------------
A = SHELL_ARM
LINKS = [
    # link        pivot (m)             sh (m)                R                    src                 prune
    ("base_link", (0.000, 0.0, 0.000), (0.000, 0.0, 0.000), [],                    A,                  "A1"),
    ("link_1",    (0.000, 0.0, 0.331), (0.000, 0.0, 0.000), [],                    A + "/A1",          "A2"),
    ("link_2",    (0.350, 0.0, 0.675), (0.350, 0.0, 0.675), [],                    A + "/A1/A2",       "A3"),
    ("link_3",    (0.350, 0.0, 1.825), (0.350, 0.0, 1.825), [("Y", 90)],           A + "/A1/A2/A3",    "A4"),
    ("link_4",    (1.350, 0.0, 1.784), (1.350, 0.0, 1.784), [("Y", 90)],           A + "/A1/A2/A3/A4", "A5"),
    ("link_5",    (1.350, 0.0, 1.784), (1.350, 0.0, 1.784), [("Y", 90), ("X", -90)],
     A + "/A1/A2/A3/A4/A5", "A6"),
    ("link_6",    (1.565, 0.0, 1.784), (1.565, 0.0, 1.784), [("Y", 90)],
     A + "/A1/A2/A3/A4/A5/A6", None),
    # tool0 is the TCP the IK solves for, so it must sit ON the mounting face.
    # The KR210 puts it 37.5 mm past link_6, which lands INSIDE the shell's
    # flange casting; measured, the shell's flange face is 139 mm out.
    ("tool0",     (1.704, 0.0, 1.784), (1.704, 0.0, 1.784), [],                    None,               None),
]
PIVOT = {n: p for n, p, _, _, _, _ in LINKS}

# Axes and limits are left alone: the shell turns about the same axes as the
# skeleton (Z,Y,Y,X,Y,X) and both links keep identity rotation at the zero pose,
# so the joints' localRot0/localRot1 stay valid.
JOINTS = [("joint_a1", "base_link", "link_1"),
          ("joint_a2", "link_1", "link_2"),
          ("joint_a3", "link_2", "link_3"),
          ("joint_a4", "link_3", "link_4"),
          ("joint_a5", "link_4", "link_5"),
          ("joint_a6", "link_5", "link_6"),
          ("link_6_tool0", "link_6", "tool0"),
          ("Link1_link_1", "link_1", "Link1")]

# The KR210's hulls were built around ITS links, and the links just changed
# length. Left alone an invisible hull sticks out of the visible arm and hits
# things in mid-air.
#   link_2  1.25 m of hull on a 1.15 m link -> squeeze 8% down +Z
#   link_3  hull spans elbow -> a4 (0.96 m), shell's forearm is 1.00 m: near
#           enough, left alone
#   link_4  the KR210 carries HALF ITS FOREARM here (a4 -> a5 is 0.54 m); on the
#   link_5  shell that gap is 0.215 m, so both hulls would jut half a metre past
#           the wrist. Switched off. link_6 keeps its hull -- the gripper mounts
#           there, and that is the contact that matters.
COLLIDER_SCALE = {"link_2": (1.0, 1.0, 1.150 / 1.2499)}
COLLIDER_OFF = ("link_4", "link_5")

# RMPflow config emitted by write_motion_config()
CFG_DIR = os.path.join(REPO_ROOT, "assets", "kuka_kr120")
RMPFLOW_CFG = os.path.join(CFG_DIR, "rmpflow", "config.json")

_KR210_MP = os.path.join(_ISAAC_ROOT, "exts",
                         "isaacsim.robot_motion.motion_generation",
                         "motion_policy_configs", "Kuka", "kr210")
_SPHERE_FIX = {                     # link -> (axis index, factor)
    "link_2": (2, 1.150 / 1.2499),  # upper arm, spheres run up +Z
    "link_3": (0, 1.000 / 0.958),   # forearm, spheres run out +X
    "link_4": (0, 0.215 / 0.542),   # wrist stub, shrank hard
}


def _log(m):
    print(f"[kuka] {m}", flush=True)


def rot(axis, deg):
    return Gf.Rotation(Gf.Vec3d(*axis), deg)


def quat(rots):
    q = Gf.Quatd(1)
    for ax, deg in rots:
        q = q * rot({"X": (1, 0, 0), "Y": (0, 1, 0),
                     "Z": (0, 0, 1)}[ax], deg).GetQuat()
    return q


def xform(prim, translate=(0, 0, 0), rots=(), scale=None):
    """Author a clean xform as ONE matrix op, replacing whatever the reference
    brought with it (an A-node arrives carrying its own rest pose). NOT for
    rigid bodies -- see body_xform()."""
    m = Gf.Matrix4d(1.0)
    if scale is not None:
        m = m * Gf.Matrix4d().SetScale(Gf.Vec3d(scale, scale, scale))
    m = m * Gf.Matrix4d().SetRotate(quat(rots))
    m = m * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*translate))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.MakeMatrixXform().Set(m)


def body_xform(prim, translate=(0, 0, 0), rots=(), q=None):
    """Place a RIGID BODY: translate+orient, never a matrix. PhysX writes each
    body's pose back as xformOp:translate + xformOp:orient, and if the op order
    holds a matrix instead those writes stack on top of it -- the body is
    transformed twice and the robot leaves the frame.

    Precision is read off the existing attribute, not assumed: the KR210 ships
    quatd, and AddOrientOp() defaults to float, which USD refuses outright."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    dbl, flt = UsdGeom.XformOp.PrecisionDouble, UsdGeom.XformOp.PrecisionFloat

    a = prim.GetAttribute("xformOp:translate")
    if a and a.GetTypeName() == Sdf.ValueTypeNames.Float3:
        xf.AddTranslateOp(flt).Set(Gf.Vec3f(*translate))
    else:
        xf.AddTranslateOp(dbl).Set(Gf.Vec3d(*translate))

    if q is None:
        q = quat(rots)
    a = prim.GetAttribute("xformOp:orient")
    if a and a.GetTypeName() == Sdf.ValueTypeNames.Quatf:
        xf.AddOrientOp(flt).Set(Gf.Quatf(q))
    else:
        xf.AddOrientOp(dbl).Set(q)


def graft(stage, app, dst, pos, yaw, pedestal=True, show_bones=False,
          gripper=True, log=_log):
    """Build one grafted KUKA at `dst`, standing on the welding cell's pedestal.

    `pos` is the MOUNT PLANE (the pedestal's top, or the floor if pedestal is
    False), so the caller passes floor coordinates and gets a robot standing on
    its base. Returns the articulation root path (= dst).
    """
    z0 = pos[2] + (PEDESTAL_TOP_Z if pedestal else 0.0)
    base = (pos[0], pos[1], z0)

    root = stage.DefinePrim(dst, "Xform")
    root.GetReferences().AddReference(SKEL_USD)
    body_xform(root, translate=base, rots=[("Z", yaw)])
    for _ in range(10):
        app.update()

    # ---- 1. reshape the bones onto the shell's dimensions -----------------
    for name, pivot, _, _, _, _ in LINKS:
        p = stage.GetPrimAtPath(f"{dst}/{name}")
        if p.IsValid():
            body_xform(p, translate=pivot)          # rotation stays identity
    p = stage.GetPrimAtPath(f"{dst}/Link1")         # dead weld-on of link_1
    if p.IsValid():
        body_xform(p, translate=PIVOT["link_1"])

    jmap = {p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath(dst))
            if p.IsA(UsdPhysics.Joint)}
    for jn, parent, child in JOINTS:
        if jn not in jmap:
            log(f"  WARNING: joint {jn} not found")
            continue
        j = UsdPhysics.Joint(jmap[jn])
        pp, cp = PIVOT.get(parent), PIVOT.get(child, PIVOT["link_1"])
        if pp is None:
            continue
        # the pivot is the CHILD's origin in each body's own frame. Both frames
        # are identity-rotated at the zero pose, so this is a plain subtraction,
        # and localPos1 is zero because the child's origin IS the pivot.
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[c - p for c, p in zip(cp, pp)]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    # the fixed root joint anchors base_link to the WORLD, not to the parent
    # xform: leave its default (0,0,0) and the robot snaps to the origin the
    # moment physics starts.
    if "root_joint" in jmap:
        j = UsdPhysics.Joint(jmap["root_joint"])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*base))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(quat([("Z", yaw)])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # ---- 2. the skeleton goes invisible, its hulls get resized ------------
    for name, _, _, _, _, _ in LINKS:
        v = stage.GetPrimAtPath(f"{dst}/{name}/visuals")
        if v.IsValid() and not show_bones:
            UsdGeom.Imageable(v).MakeInvisible()
        c = stage.GetPrimAtPath(f"{dst}/{name}/collisions")
        if not c.IsValid():
            continue
        if name in COLLIDER_OFF:
            UsdPhysics.CollisionAPI(c).CreateCollisionEnabledAttr(False)
        elif name in COLLIDER_SCALE:
            xf = UsdGeom.Xformable(c)         # pre-multiply: scale the mesh
            m = xf.GetLocalTransformation(Usd.TimeCode.Default())
            xf.ClearXformOpOrder()
            xf.MakeMatrixXform().Set(
                Gf.Matrix4d().SetScale(Gf.Vec3d(*COLLIDER_SCALE[name])) * m)

    # ---- 3. hang the shell's meshes on the bones -------------------------
    total = 0
    for name, pivot, sh, rots, src, prune in LINKS:
        if src is None:
            continue
        sp = stage.DefinePrim(f"{dst}/{name}/shell", "Xform")
        sp.GetReferences().AddReference(SHELL_USD, primPath=Sdf.Path(src))
        for _ in range(4):
            app.update()
        # replace the A-node's own rest transform with the frame map, put the
        # casting back on its own pivot, and convert mm -> m
        off = tuple(s - p for s, p in zip(sh, pivot))
        xform(sp, translate=off, rots=rots, scale=MM)
        if prune:
            kid = stage.GetPrimAtPath(f"{dst}/{name}/shell/{prune}")
            if kid.IsValid():
                kid.SetActive(False)          # it belongs to the NEXT link
            else:
                log(f"  WARNING: {name}: nothing to prune at {prune}")
        total += sum(1 for q in Usd.PrimRange(sp, Usd.TraverseInstanceProxies())
                     if q.IsA(UsdGeom.Mesh))

    # The 34 materials survive the sub-prim reference only because each link's
    # materialLibrary lives INSIDE that link. Anything binding outside the
    # referenced prim is dropped silently and renders white -- so count.
    from pxr import UsdShade
    lost = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(dst), Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Mesh) and "shell" in p.GetPath().pathString:
            if not UsdShade.MaterialBindingAPI(p).GetDirectBinding().GetMaterial():
                lost += 1
    log(f"  {os.path.basename(dst)}: {total} shell meshes on the bones"
        f" (expect 139){', ' + str(lost) + ' LOST THEIR MATERIAL' if lost else ''}")

    if pedestal:
        ped = stage.DefinePrim(f"{dst}_base", "Xform")
        ped.GetReferences().AddReference(SHELL_USD,
                                         primPath=Sdf.Path(PEDESTAL_PRIM))
        xform(ped, translate=pos, rots=[("Z", yaw)], scale=MM)
    if gripper:
        add_surface_gripper(stage, dst, pos, yaw, log=log)
    return dst


def add_surface_gripper(stage, dst, pos, yaw, cups=4, spread=0.055,
                        max_grip=0.10, hold_force=2000.0, log=_log):
    """Bolt a suction gripper onto the flange.

    A 51 cm carton with smooth faces cannot be pinched -- two fingers have
    nothing to close around -- so the arm sucks. In 5.1 that means a typed
    IsaacSurfaceGripper prim pointing at D6 joints that carry
    IsaacAttachmentPointAPI, and there are three traps in that sentence:

    1. THE PARK BODY. The plugin only reads body0 (the gripper) and swaps body1
       to whatever it grips. So body1 has to be parked on something harmless in
       the meantime -- and if you leave it EMPTY, USD anchors the joint to the
       WORLD and the flange is welded in mid-air: the arm simply will not move.
       NVIDIA's own sample parks it on the gripper's mount, so we do the same,
       with a small `pad` rigid body welded to link_6.
    2. robot_schema.ApplyAttachmentPointAPI() IS BROKEN. It calls
       Classes.ATTACHMENT_POINT_API.name, and `Classes` is a plain Enum whose
       .name is the MEMBER name -- so it applies a schema called
       "ATTACHMENT_POINT_API" instead of "IsaacAttachmentPointAPI", and the
       gripper then never sees the attachment point. Apply .value by hand.
    3. THE APPROACH AXIS. isaac:forwardAxis is the joint frame's Z, and the
       flange's approach is link_6's +X, so every attachment joint is turned
       90 deg about Y. Point it the wrong way and the gripper reaches for
       objects behind its own wrist.

    The limits/drives are copied from SurfaceGripper_gantry.usda (transX/transY
    locked by low > high; transZ a short sprung stroke; the wrist rotations
    slightly compliant), which is the only configuration NVIDIA ships working.
    """
    from usd.schema.isaac import robot_schema

    link6 = f"{dst}/link_6"
    face = PIVOT["tool0"][0] - PIVOT["link_6"][0]      # 0.139 m: the flange face

    # The pad: the gripper's own body, welded to the flange. It carries no
    # collider -- it must never knock the box it is trying to pick up. It is
    # TURNED 90 deg about Y so that its +Z looks out of the flange: the cups
    # then spread over its X/Y and forwardAxis="Z" means what it says.
    #
    # It lives OUTSIDE the robot, not under link_6. A rigid body nested inside
    # another rigid body is illegal in USD Physics, and PhysX does not complain
    # -- it just behaves strangely: with the pad parented to the flange, the arm
    # would not even hold its own zero pose (measured: link_6 sat 2 m from where
    # the joints said it was) and RMPflow thrashed trying to correct an arm that
    # was being shoved by its own gripper. Out here, welded by a joint, it is
    # exactly the arrangement NVIDIA's own gripper sample uses.
    #
    # body_xform, not xform: this is a rigid body, and PhysX writes its pose back
    # as translate+orient (a matrix op would be applied twice).
    pad = stage.DefinePrim(f"{dst}_gripper/pad", "Xform")
    l6 = PIVOT["link_6"]
    m = (Gf.Matrix4d().SetRotate(quat([("Y", 90)]))
         * Gf.Matrix4d().SetTranslate(Gf.Vec3d(l6[0] + face + 0.01, l6[1], l6[2]))
         * Gf.Matrix4d().SetRotate(quat([("Z", yaw)]))
         * Gf.Matrix4d().SetTranslate(Gf.Vec3d(pos[0], pos[1],
                                               pos[2] + PEDESTAL_TOP_Z)))
    t = Gf.Transform(m)      # take the rotation FROM the matrix: composing the
    #                          quaternions by hand gets the order wrong one time
    #                          in two, and a 90 deg error here aims the suction
    #                          sideways
    body_xform(pad, translate=t.GetTranslation(),
               q=t.GetRotation().GetQuat())
    UsdPhysics.RigidBodyAPI.Apply(pad)
    UsdPhysics.MassAPI.Apply(pad).CreateMassAttr(0.2)

    out = quat([("Y", 90)])            # link_6 -> pad: +Z now points out
    weld = UsdPhysics.FixedJoint.Define(stage, f"{dst}_gripper/weld")
    weld.CreateBody0Rel().SetTargets([Sdf.Path(link6)])
    weld.CreateBody1Rel().SetTargets([pad.GetPath()])
    weld.CreateLocalPos0Attr().Set(Gf.Vec3f(face + 0.01, 0.0, 0.0))
    weld.CreateLocalRot0Attr().Set(Gf.Quatf(out))
    weld.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    weld.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    weld.CreateExcludeFromArticulationAttr(True)   # a maximal-coordinate weld:
    #                                   the articulation keeps its 6 dof exactly

    joints = []
    for i in range(cups):
        a = 2.0 * math.pi * i / cups
        j = UsdPhysics.Joint.Define(stage, f"{dst}_gripper/cup_{i}")
        p = j.GetPrim()
        p.AddAppliedSchema(robot_schema.Classes.ATTACHMENT_POINT_API.value)
        p.CreateAttribute(robot_schema.Attributes.FORWARD_AXIS.name,
                          Sdf.ValueTypeNames.Token, False).Set("Z")
        p.CreateAttribute(robot_schema.Attributes.CLEARANCE_OFFSET.name,
                          Sdf.ValueTypeNames.Float, False).Set(0.008)

        j.CreateBody0Rel().SetTargets([pad.GetPath()])        # the gripper
        j.CreateBody1Rel().SetTargets([Sdf.Path(link6)])      # parked here
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(True)
        j.CreateBreakForceAttr(3.4028235e38)
        j.CreateBreakTorqueAttr(3.4028235e38)

        # the cup ring, in the PAD's frame, where +Z is already the approach --
        # so the cups spread over X/Y and the joint needs no rotation at all.
        cx, cy = spread * math.cos(a), spread * math.sin(a)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(cx, cy, 0.0))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        # the SAME point, written in link_6's frame (the park body): the pad is
        # turned 90 deg about Y, which sends (x,y,z) -> (z,y,-x)
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(face + 0.01, cy, -cx))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(out))

        for axis, lo, hi in (("transX", 1.0, -1.0),      # low > high == LOCKED
                             ("transY", 1.0, -1.0),
                             ("transZ", 0.0, 0.01),      # a short sprung stroke
                             ("rotX", -3.0, 3.0),
                             ("rotY", -3.0, 3.0),
                             ("rotZ", -3.0, 3.0)):
            lim = UsdPhysics.LimitAPI.Apply(p, axis)
            lim.CreateLowAttr(lo)
            lim.CreateHighAttr(hi)
        for axis, stiff, damp in (("transZ", 5000.0, 100.0), ("rotX", 100.0, 0.0),
                                  ("rotY", 100.0, 0.0), ("rotZ", 10000.0, 0.0)):
            d = UsdPhysics.DriveAPI.Apply(p, axis)
            d.CreateStiffnessAttr(stiff)
            d.CreateDampingAttr(damp)
        joints.append(p.GetPath())

    g = robot_schema.CreateSurfaceGripper(stage, f"{link6}/SurfaceGripper")
    g.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name
                      ).SetTargets(joints)
    g.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(max_grip)
    # these are BREAK limits: the box weighs 4 kg (39 N) and gets swung around,
    # so they are set well above that or it lets go mid-transfer
    g.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(hold_force)
    g.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(hold_force)
    g.GetAttribute(robot_schema.Attributes.RETRY_INTERVAL.name).Set(2.0)
    g.GetAttribute(robot_schema.Attributes.STATUS.name).Set("Open")

    log(f"  gripper: {cups} suction cups on the flange face (x=+{face:.3f} in "
        f"link_6), grip range {max_grip * 100:.0f} cm, break limit "
        f"{hold_force:.0f} N -> {link6}/SurfaceGripper")
    return f"{link6}/SurfaceGripper"


def write_motion_config(log=_log):
    """Emit the URDF + RMPflow config for the RESHAPED arm, straight from the
    LINKS table -- so the IK cannot drift away from the geometry. Returns the
    path to config.json (feed it to RmpFlow)."""
    os.makedirs(os.path.join(CFG_DIR, "rmpflow"), exist_ok=True)
    origins = {jn: tuple(c - p for c, p in zip(PIVOT[child], PIVOT[parent]))
               for jn, parent, child in JOINTS
               if parent in PIVOT and child in PIVOT}
    origins["link_6-tool0"] = origins.pop("link_6_tool0")

    with open(os.path.join(_KR210_MP, "kr210.urdf")) as f:
        urdf = f.read()
    n = 0
    for jn, xyz in origins.items():
        # the origin is not always the first tag in the block -- the tool0 weld
        # puts <parent>/<child> ahead of it -- so scan to the first <origin>
        # without crossing into the next joint
        pat = re.compile(r'(<joint name="' + re.escape(jn)
                         + r'"[^>]*>(?:(?!</joint>).)*?<origin rpy="[^"]*" '
                         r'xyz=")[^"]*(")', re.S)
        urdf, k = pat.subn(
            lambda m: f"{m.group(1)}{xyz[0]:.6g} {xyz[1]:.6g} {xyz[2]:.6g}"
                      f"{m.group(2)}", urdf, count=1)
        n += k
        if not k:
            log(f"  WARNING: joint {jn} not rewritten in the URDF")
    urdf = urdf.replace('name="kr210"', 'name="kr120_r2500"', 1)
    with open(os.path.join(CFG_DIR, "kr120_r2500.urdf"), "w") as f:
        f.write(urdf)

    # the collision spheres RMPflow keeps out of obstacles ride on the links
    # that changed length, so move them too -- otherwise the arm dodges air
    with open(os.path.join(_KR210_MP, "rmpflow",
                           "kuka_kr210_robot_description.yaml")) as f:
        lines = f.readlines()
    out, link, moved = [], None, 0
    for ln in lines:
        m = re.match(r"\s*-\s*(link_\d+):", ln)
        if m:
            link = m.group(1)
        c = re.match(r'(\s*-\s*"center":\s*\[)([^\]]*)(\].*)', ln)
        if c and link in _SPHERE_FIX:
            ax, f_ = _SPHERE_FIX[link]
            v = [float(x) for x in c.group(2).split(",")]
            v[ax] *= f_
            ln = f"{c.group(1)}{v[0]:.4g}, {v[1]:.4g}, {v[2]:.4g}{c.group(3)}\n"
            moved += 1
        out.append(ln)
    with open(os.path.join(CFG_DIR, "rmpflow",
                           "kr120_robot_description.yaml"), "w") as f:
        f.writelines(out)

    shutil.copy(os.path.join(_KR210_MP, "rmpflow", "kr210_rmpflow_config.yaml"),
                os.path.join(CFG_DIR, "rmpflow", "kr120_rmpflow_config.yaml"))
    with open(RMPFLOW_CFG, "w") as f:
        json.dump({"end_effector_frame_name": "tool0",
                   "maximum_substep_size": 0.00334,
                   "ignore_robot_state_updates": False,
                   "relative_asset_paths": {
                       "robot_description_path": "kr120_robot_description.yaml",
                       "urdf_path": "../kr120_r2500.urdf",
                       "rmpflow_config_path": "kr120_rmpflow_config.yaml"}},
                  f, indent=4)
    log(f"URDF {n}/{len(origins)} joints + {moved} collision spheres reshaped "
        f"-> {RMPFLOW_CFG}")
    return RMPFLOW_CFG
