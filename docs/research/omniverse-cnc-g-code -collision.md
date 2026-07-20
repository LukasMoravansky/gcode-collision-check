# Omniverse for CNC G-code collision — feasible but a poor fit
 
**Verdict up front: NVIDIA Omniverse/Isaac Sim with `omni.physx` is technically capable of running a headless G-code collision check from Python, but it is the *wrong tool* for this use case.** PhysX is built for stable rigid-body dynamics, not deterministic exact-geometric interference. It has documented limitations (no triangle-mesh ↔ triangle-mesh contacts, SDF approximations that can silently miss thin features, CPU-only scene queries that negate the headline GPU advantage) and a heavy operational/licensing footprint (RTX-only GPU, ~20 GB containers, Omniverse Enterprise required for commercial redistribution). For a CNC verifier, **Python + trimesh + python-fcl remains the right primary stack**, with C++ FCL as the scale-up path. Use Omniverse only if you want photorealistic visualization on top of a separately implemented collision engine — or as a long-running pose-query service when the CAD pipeline is already inside the Omniverse ecosystem.
 
The rest of this report justifies that verdict, lays out the Omniverse architecture if you choose to use it anyway, flags showstoppers explicitly, and places Omniverse next to the previously researched stacks.
 
## Where Omniverse actually fits in the pipeline
 
Omniverse is a collection of OpenUSD-centric libraries and SDKs (the launcher was deprecated October 2025; everything now ships via GitHub + NGC + pip). The pieces relevant to a CNC checker are:
 
- **`omni.physx`** wraps the NVIDIA **PhysX 5** engine, exposes `UsdPhysics`/`PhysxSchema` USD authoring, a contact-report event stream, and a `PhysXSceneQuery` interface for overlap/sweep/raycast queries.
- **Omniverse Kit SDK** is the host runtime. Launched headless via `SimulationApp({"headless": True})` (Isaac Sim's wrapper) or `kit --no-window`. **Cold start is 5–15 minutes** the first time (shader and extension caching) and 10–60 s with warm caches — so amortizing startup with a long-lived service is mandatory.
- **`omni.kit.asset_converter`** (free, ASSIMP-based) converts **STL, OBJ, FBX, glTF, PLY** to USD. **`omni.kit.converter.cad`** (HOOPS-based) handles **STEP, IGES, JT, SLDPRT, CATIA, Parasolid, ACIS, Inventor** etc., requires 16–32 GB RAM, and — critically — is **not in NVIDIA's redistributable extensions list**.
- **Isaac Sim** is the reference framework most teams use on top: provides `isaacsim.core.prims` (`RigidPrim`, `GeometryPrim`, `XformPrim` with batched `set_world_poses`), the `World` step loop, the `ContactSensor`, URDF/MJCF importers, and the headless container `nvcr.io/nvidia/isaac-sim:5.1.0` / `6.0.0`. The Apache-2.0 source repo (`github.com/isaac-sim/IsaacSim`) is free; Kit binaries shipped with it are under a proprietary additional license.
There is **no NVIDIA-Omniverse reference project for CNC, milling, or G-code** as of June 2026. The closest paradigm is the Isaac Sim *robot self-collision detector* tutorial and the Inspire Hand *Collider Pairs* tutorial — treat the CNC as an articulated "robot" with prismatic X/Y/Z axes, group its parts via `UsdPhysics.CollisionGroup` + `FilteredPairsAPI`, and check the tool/spindle group against the workpiece/fixture group.
 
## Two viable Omniverse architectures (and why one is better)
 
There are two fundamentally different ways to run the collision check, and the choice matters more than any other decision.
 
**Architecture A — Pose-query (recommended if you go Omniverse).** Load the USD scene with colliders authored, call `play()` once so PhysX builds its actor and shape representation, then for every interpolated G-code pose place the tool with `XformPrim.set_world_poses` and call `get_physx_scene_query_interface().overlap_mesh(...)` or `.sweep_sphere_all(...)`. **No physics stepping happens.** This is fastest, deterministic, and maps cleanly to "is the tool at pose X colliding?". Catch: `overlap_mesh` internally uses a *convex* approximation of the input mesh, and **breaks if Fabric/Flatcache is enabled** (open issue `isaac-sim/IsaacSim#233`). Sweep queries between consecutive poses cover the missing-thin-feature problem without needing CCD.
 
**Architecture B — Stepping with contact reports.** Make the tool a kinematic rigid body, apply `PhysxContactReportAPI`, drive `world.step(render=False)` per pose, and subscribe to `subscribe_contact_report_events` for full pair/position/normal/separation/impulse data. **CCD must be explicitly enabled** at scene level *and* per-body, otherwise high-feed segments tunnel through fixtures silently. This is the only path to exact penetration depth and is what NVIDIA documents in the contact-reports dev guide.
 
Either way the moving tool must use a **`convexDecomposition`** or **`sdf` approximation** because PhysX forbids triangle-mesh shapes on dynamic actors. Static fixtures (vise, column, table, workpiece-as-static) can use `approximation = "none"` (raw triangle mesh).
 
## Minimal headless Python pipeline (pseudocode)
 
```python
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})  # all omni/pxr imports MUST come after
 
from pxr import UsdPhysics, PhysxSchema, PhysicsSchemaTools
import omni.usd
from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import XformPrim
 
world = World(stage_units_in_meters=0.001, physics_dt=1/1000)  # mm
stage = omni.usd.get_context().get_stage()
 
# 1. Assemble scene from converted USDs (STL/OBJ via Asset Converter; STEP via CAD Converter)
add_reference_to_stage("machine.usd",   "/World/Machine")   # spindle, column, table
add_reference_to_stage("vise.usd",      "/World/Vise")
add_reference_to_stage("workpiece.usd", "/World/Workpiece")
add_reference_to_stage("tool.usd",      "/World/Tool")
 
# 2. Apply colliders + collision groups
for path in ["/World/Machine", "/World/Vise", "/World/Workpiece"]:
    prim = stage.GetPrimAtPath(path)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("none")
 
tool = stage.GetPrimAtPath("/World/Tool")
UsdPhysics.CollisionAPI.Apply(tool)
UsdPhysics.MeshCollisionAPI.Apply(tool).CreateApproximationAttr().Set(
    PhysxSchema.Tokens.sdf)            # or "convexDecomposition"
UsdPhysics.RigidBodyAPI.Apply(tool)
PhysxSchema.PhysxRigidBodyAPI.Apply(tool).CreateKinematicEnabledAttr(True)
PhysxSchema.PhysxContactReportAPI.Apply(tool).CreateThresholdAttr(0.0)
 
# 3. Subscribe to contact events, indexed by current G-code line
report = []
current_line = {"n": None}
 
def on_contact(headers, data):
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
            c = data[i]
            report.append({
                "gcode_line": current_line["n"],
                "pair": (a0, a1),
                "position": tuple(c.position),
                "normal": tuple(c.normal),
                "separation": c.separation,
            })
 
sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
 
# 4. Drive interpolated G-code positions (each pose remembers its source line number)
world.reset()
tool_view = XformPrim(prim_paths_expr="/World/Tool")
for pose in interpolate_gcode(gcode_path, max_step_mm=0.05):
    current_line["n"] = pose.source_line
    tool_view.set_world_poses(positions=pose.xyz[None], orientations=pose.quat[None])
    world.step(render=False)
 
# 5. Persist structured report (line, XYZ, colliding pair, normal, penetration)
write_json(report, "collisions.json")
sim.close()
```
 
For the pose-query variant (Architecture A) replace step 4 with a `sweep_sphere_all` per G-code segment from pose A to pose B; you'll lose penetration depth but gain ~10× throughput.
 
## Showstoppers, ranked
 
The first three are technical and the last three are operational. Any one of the technical issues should make you reconsider Omniverse for this specific problem.
 
**PhysX does not support triangle-mesh ↔ triangle-mesh contacts.** Documented in every PhysX 5.x release and reconfirmed in GitHub Discussion `NVIDIA-Omniverse/PhysX#172`: "The PhysX SDK supports contacts between all possible combinations of shape pair *except for combinations where both shapes are a plane or a heightfield or a triangle mesh*." Your tool therefore must be approximated by SDF or convex decomposition. **For a fluted end-mill, vise jaws with grooves, or a complex spindle nose, the approximation is the collision geometry — not the CAD geometry.**
 
**SDF mesh collision can silently miss thin features.** Direct quote from the PhysX 5 docs: "Too low SDF resolution can lead to situations where very thin parts of the mesh don't collide since the SDF cannot represent/capture them." Combine that with PhysX GPU buffer overflow behavior, which the docs admit "may issue warnings and discard contacts/constraints/pairs" — and you have two silent-miss failure modes in the path your CNC verifier depends on. For a tool that exists to *prevent* machine crashes, silent misses are a catastrophic defect class.
 
**PhysX scene queries are CPU-only.** The 5.4 docs are explicit: "Scene queries are executed on the CPU and do not interact with GPU-only features." So the most natural workload for batched G-code checking (millions of overlap/sweep queries) **does not benefit from the GPU at all** — exactly the headline reason most people consider Omniverse. The actually GPU-accelerated PhysX features (broadphase, narrowphase contact gen, SDF) only matter inside the full-simulation step loop, which is slower per-pose than scene queries.
 
**Omniverse Kit cannot be redistributed under the free license.** The Isaac Sim License FAQ (current June 2026) states plainly: "If you are redistributing Isaac Sim (with Omniverse Kit) as part of an application to third parties, or delivering Isaac Sim (with Omniverse Kit) as a service to third parties" — an NVIDIA AI Enterprise license is required. Anecdotal pricing from 2025 forum discussions: **~$4,500/GPU/year**. The EULA additionally restricts use to "systems where the use or failure can reasonably be expected to threaten personal injury, death, or catastrophic loss" — a CNC crash arguably qualifies, so you'd want an explicit side-agreement.
 
**The STEP/IGES importer is not redistributable.** The CAD Converter's HOOPS Exchange backend docs say "Do not redistribute or sublicense without express permission or agreement." Internal R&D use is fine; shipping a CNC product with built-in STEP support requires negotiating both with NVIDIA and likely with Tech Soft 3D directly (typical HOOPS Exchange OEM cost: tens of thousands USD/year).
 
**Hardware and platform churn.** Isaac Sim 5.1 minimum is an RTX 4080 with 16 GB VRAM; A100/H100 are explicitly not supported (no RT cores). Containers are Linux-only, ~20 GB. Between Isaac Sim 4.x and 5.x NVIDIA renamed the `omni.isaac.*` namespace to `isaacsim.*` wholesale, and the Omniverse Launcher was killed in October 2025. Budget periodic forced upgrades for any product built on this stack.
 
## Comparison with previously researched stacks
 
The previously researched stacks (Python trimesh/FCL, C++ FCL/Bullet, C# BulletSharp) all share one decisive technical advantage over Omniverse: **they perform exact triangle-mesh ↔ triangle-mesh narrowphase via BVH + triangle-triangle intersection tests**. That is what a CNC verifier should be doing.
 
| Dimension | **Omniverse + PhysX** | **Python trimesh + python-fcl** | **C++ FCL or Bullet** | **C# BulletSharp** |
|---|---|---|---|---|
| Tri-mesh ↔ tri-mesh exact contact | **No** — SDF or convex decomp only | **Yes** (BVH + tri-tri) | **Yes** | Yes (via GImpact) |
| Distance / clearance queries | Limited (separation in contact data) | **Yes**, native | **Yes** | Partial |
| Continuous collision between poses | Speculative CCD (for sim, not exact sweep) | `continuousCollide` (conservative advancement) | Yes | Yes |
| GPU acceleration for batched queries | **No** (scene queries are CPU; SDF GPU helps full sim only) | No (multiproc scales linearly) | No | No |
| Per-query latency (typical) | ms-range (Python binding overhead) | 50–500 µs | 30–300 µs | 50–500 µs + marshalling |
| 1 M-pose batch throughput | Tens of minutes; complex setup | **Minutes single-core; <1 min 8 cores** | Sub-minute | Comparable to C++ |
| Integration code (MVP) | 100–300 LoC + USD asset prep | **~20–50 LoC** | 150–400 LoC + CMake | ~100 LoC + .NET project |
| Install footprint | ~20 GB container, RTX 4080+ required | ~200 MB; CPU only | ~50–200 MB; CPU only | ~150 MB; CPU only |
| Cold start | 5–15 min first time, 10–60 s warm | ms | ms | hundreds of ms |
| Windows install | Native installer / WSL | **Painful** (use `python-fcl-win32` or WSL) | Easy via CMake/vcpkg | Easiest |
| Cross-platform | Linux + Windows; not macOS | Linux/macOS easy, Windows fiddly | All three | All three |
| Maintenance status (2026) | Active, but churning | Active; `hpp-fcl`/`coal` fork is fastest | Active | **Effectively maintenance-mode** since 2021 |
| Commercial redistribution | **Requires Omniverse Enterprise (~$4.5K/GPU/yr)**; STEP importer not redistributable | BSD-3 / Apache-2.0; free | BSD-3 (FCL) or zlib (Bullet); free | zlib (Bullet); free |
| CAD format coverage | STL/OBJ/FBX/glTF + STEP/IGES/JT/CATIA via CAD Converter | STL/OBJ/PLY/glTF via trimesh; STEP needs external (OCCT, cadquery) | Same — STEP needs OCCT | Same |
| GUI/visualization | RTX renderer included | Add matplotlib/PyVista/Open3D | DIY (OpenGL/VTK) | DIY (Unity/WPF/OpenGL) |
| Suitable for material-removal verification? | No (PhysX has no CSG; dexel kernel needed separately) | No (FCL is collision only) | No | No |
 
The pattern is unmistakable: for the actual collision-detection workload, **the lighter stacks are technically more accurate, easier to deploy, faster on relevant batches, and licensing-clean**. Omniverse's *only* objective advantages are the rendering pipeline and an easier path to CAD interchange (STEP/IGES) — neither of which the user has identified as a requirement.
 
It is also worth noting that commercial CNC verifiers (Vericut, NCsimul, CAMplete, ModuleWorks) do not use any of these four stacks. They use proprietary **dexel/voxel + swept-volume** kernels because (a) they need to subtract material from the workpiece as the tool cuts, and (b) they need to integrate over the full sweep to catch sub-sample-rate features. If your collision report needs to reflect "is the tool gouging the *current* in-process stock", neither PhysX nor FCL gives you that — you'd plug in OpenCAMlib (open source) or ModuleWorks (commercial) for the material-removal layer and keep the collision layer separate.
 
## When Omniverse would actually make sense
 
There is a coherent argument for Omniverse, but it is narrower than the marketing implies:
 
You already have an Omniverse-based digital-twin product, your customers expect photorealistic visualization of the machining process, or you want to ship the CNC checker as one extension of a larger USD-centric platform. In that case, build it as a **Kit Service** (NVIDIA template at `github.com/NVIDIA-Omniverse/kit-app-template`) that holds a warm scene and answers G-code queries — Architecture A (scene queries, no stepping) with `convexDecomposition` colliders on the tool and `none` on the static fixtures. Pre-convert STEP at the customer site so you avoid HOOPS redistribution. Treat PhysX's results as a "physics-grade pre-check" and validate with a separate exact engine before reporting "clean" to a user.
 
For everything else — and that includes the scenario described in this task — **build the pipeline on Python + trimesh + python-fcl** (or its faster fork `hpp-fcl`/`coal`). Drop to C++ FCL only if profiling shows Python overhead dominating. Use Omniverse, if at all, as a *visualization layer* sitting on top of a collision engine you actually trust.
 
## Bottom line
 
NVIDIA Omniverse delivers a polished USD-based authoring environment, a working headless Python pipeline, and impressive marketing around GPU-accelerated physics. But under the hood, for *this specific problem*, it is built on a rigid-body dynamics engine that cannot do triangle-mesh ↔ triangle-mesh contacts, whose accuracy on non-convex CNC tooling depends on approximations with documented silent-miss modes, whose batched-query workload runs entirely on the CPU anyway, and whose redistribution requires a four-figure-per-GPU annual subscription. The previously researched stacks — particularly **Python with trimesh + python-fcl** for fast iteration and **C++ FCL** for production scale — give you exact geometry, simpler deployment, zero licensing friction, and equivalent or better throughput. **Use Omniverse for the picture, not for the answer.**