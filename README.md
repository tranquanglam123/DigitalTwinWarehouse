# Digital Twin Warehouse

A warehouse tote line in NVIDIA Isaac Sim 5.1, built and rendered entirely from Python. No GUI steps: the scene is assembled by script from NVIDIA's public sample assets, captured headless, and encoded to video with an automated black-frame audit.

Demo clip: https://martinthexr.dev/work/warehouse-tote-flow.html

## What runs in the sim

KLT totes ride a roller conveyor down a straight line to a T-junction, where a divert pushes each tote off the line into a drop bin. Around the line: a KUKA arm cell behind safety fencing, forklifts, pallet racks, and a Boston Dynamics Spot walking alongside (stock `SpotFlatTerrainPolicy`).

The parts that took actual engineering:

- The rollers are the drive surface. Every roller body carries an `IsaacConveyor` OmniGraph node with its surface velocity written in the world frame, so totes move by real PhysX contact, not animated geometry, and the belts keep working even with the conveyor extension's runtime disabled.
- Totes are rigid bodies with an invisible box collision proxy sized from the visual mesh. Convex-hulling a scanned mesh is a known PhysX crash path; the proxy sidesteps it.
- Nothing is spawned at runtime. All totes exist at build time; a `BehaviorScript` recycler parks finished totes back at the head of the line while they are kinematic, then flips them back to dynamic so they wake with zero velocity. Long renders never run dry and no physics prim is ever created or deleted while the timeline plays.
- The arm is a graft: Isaac's KR210 skeleton supplies the articulation, the sample welding cell's KR120 shell supplies every visible surface. Meshes are re-parented onto the rigid bodies and the bones reshaped to the shell, with a matching URDF emitted so RMPflow solves for the arm as built (`scripts/kuka_graft.py`).
- Camera moves and capture run headless through `lib/cine_capture_core.py`, with sub-stepped physics per video frame. The 1800-frame overview encodes with zero black frames.
- Layout decisions were measured, not eyeballed: `scripts/probe_belt.py` lists which roller bodies are actually driven and at what velocity, `scripts/probe_layout.py` sweeps candidate line positions against the warehouse geometry.

## Layout

    scripts/build_scene.py       assembles the scene USD from sample assets
    scripts/kuka_graft.py        KR120 shell on KR210 skeleton, URDF emitted
    scripts/package_recycler.py  BehaviorScript that keeps the line fed
    scripts/probe_belt.py        which rollers are driven, and how fast
    scripts/probe_layout.py      clearance sweep for the conveyor placement
    scripts/render_detached.ps1  detached render + encode, .done sentinel
    scripts/encode.sh            frames -> graded mp4 + poster + black-frame audit
    shots/tote_flow.py           overview orbit and test builders
    shots/overview_robot.py      same orbit with the Spot quadruped
    shots/robot_aisle.py         Unitree H1 walking the aisle
    lib/cine_capture_core.py     headless boot, camera rig, frame capture
    lib/cm/config_data.py        config loader, no omni imports
    assets/kuka_kr120/           URDF + RMPflow config for the grafted arm
    config/assets.example.json   asset pack paths, copy to assets.json

## Run

Requires Isaac Sim 5.1 and a local copy of the Isaac sample asset pack.

    # 1. point the config at your installs
    cp config/assets.example.json config/assets.json   # then edit paths

    # 2. build the scene
    <isaac>/python.bat scripts/build_scene.py

    # 3. render a shot (short test first, then the full overview)
    <isaac>/python.bat lib/cine_capture_core.py --builder shots/tote_flow.py:build_test --frames 180 --subframes 4 --out output/frames/_test --asset-root <asset_root>
    <isaac>/python.bat lib/cine_capture_core.py --builder shots/tote_flow.py:build_overview --frames 1800 --subframes 12 --out output/frames/overview --asset-root <asset_root>

    # 4. encode (bash, not PowerShell)
    scripts/encode.sh output/frames/overview output/videos/overview 60 300

On Windows, `scripts/render_detached.ps1` wraps steps 3 and 4 in a detached process and drops a `.done` sentinel when finished.

All machine-local paths live in `config/assets.json`, which is gitignored. Two optional keys point at extra local packs: `extra_assets_root` (hand-placed rack rows, baked walking-worker clips) and `nvidia_packs_root` (the ArchViz sample pack the safety fences, arm shell and cartons reference). Leave them empty and the build skips those layers with a warning and still produces the core running line.

## Scope and credits

One divert point, one bin. The clip shows exactly what the sim does.

All geometry is NVIDIA sample content shipped with Isaac Sim or its asset pack: Simple Warehouse environment, ConveyorBelt A05/A23 modules, KLT bin, the ArchViz Industrial welding cell, and the Spot and H1 robots with their sample locomotion policies. Spot is a Boston Dynamics model. No client data, scenes, or code.
