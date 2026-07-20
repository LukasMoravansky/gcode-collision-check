# Unity for G-code Collision Detection in CNC Milling Simulation
 
## Research Report — June 2026
 
---
 
## VERDICT
 
**Unity is a viable but suboptimal choice for this use case. It works, but you're fighting the engine's design assumptions at nearly every step.**
 
Unity *can* detect collisions between a tool and scene objects along a G-code toolpath, report the G-code line number, XYZ position, and colliding pair. The pieces exist: headless mode, MeshColliders, static overlap queries, C# scripting, runtime mesh import. But none of them were designed for this job, and the friction is cumulative. You will spend significant engineering effort adapting a game engine to behave as a batch geometry checker — effort that would go further with a purpose-built collision library.
 
**For a pure collision-checking pipeline (no visualization, no digital twin, no interactive simulation), the direct C#/BulletSharp or C++/FCL approaches researched previously remain superior on every axis that matters: precision, deployment simplicity, performance per watt, and licensing clarity.**
 
Unity becomes competitive only if you also need real-time 3D visualization of the simulation (operator review, training, digital twin dashboard) — then the collision checking and the rendering share the same scene graph, and Unity's overhead becomes justified.
 
---
 
## 1. UNITY PHYSICS ENGINES — SUITABILITY ANALYSIS
 
### 1.1 Built-in PhysX (Classic Unity)
 
Unity's default 3D physics is NVIDIA PhysX, exposed through `UnityEngine.Physics`. This is what you'd use in a non-DOTS project.
 
**Precision:** PhysX operates entirely in single-precision (32-bit) floating point. For CNC work where coordinates are typically within a ~1 meter work envelope, single-precision gives ~0.0001 mm resolution near the origin — adequate for collision detection (not for metrology, but sufficient for "did the tool hit the vise"). However, PhysX is tuned for game-like scenarios: its collision margins, solver iterations, and broadphase are optimized for "good enough at 60 fps," not "exact to the triangle."
 
**Static query API:** The key API for this use case is NOT the physics simulation loop. Instead, you'd use:
- `Physics.OverlapBox` / `Physics.OverlapSphere` — find all colliders touching a volume at a given position. No simulation step required.
- `Physics.ComputePenetration` — given two colliders at arbitrary poses, returns whether they overlap and the minimum separation vector. **Critical limitation:** one collider must be a primitive (Box/Sphere/Capsule) or *convex* MeshCollider. The other can be non-convex.
- `Physics.CheckBox` / `Physics.CheckSphere` — boolean overlap test.
These are stateless scene queries — you position a tool collider at each G-code point and ask "does it overlap anything?" This avoids the game loop entirely.
 
**MeshCollider constraints:**
- Non-convex MeshColliders can only participate in collision with *convex* shapes or primitives. Two non-convex MeshColliders cannot collide with each other in PhysX.
- Convex MeshColliders are limited to 255 vertices (PhysX hull limit).
- For your CNC tool (typically a cylinder/cone), a convex MeshCollider or even a CapsuleCollider is fine.
- For the vise, workpiece, and machine body (concave shapes), non-convex MeshColliders work — but only if the *tool* collider is convex.
- This is workable for your use case but constrains the collision pair topology.
### 1.2 Unity Physics (DOTS/ECS)
 
Unity Physics is a stateless, deterministic physics engine written in C# using DOTS (Data-Oriented Technology Stack) and Burst-compiled. It shares the same underlying collision detection approach as PhysX (GJK/EPA for convex, BVH for mesh) but runs on the Job System.
 
**Advantages for this use case:**
- Deterministic (same input → same output across runs, on the same architecture).
- Burst-compiled C# — significantly faster than managed C# for tight loops.
- Can be driven without the traditional MonoBehaviour game loop.
- Job System enables parallel collision checks across multiple toolpath positions.
**Disadvantages:**
- Still single-precision float.
- DOTS/ECS has a steep learning curve and the API surface has changed repeatedly (experimental → preview → production across Unity 2022–6).
- Same convex-vs-non-convex constraints as PhysX.
- Runtime MeshCollider creation in DOTS is more complex than classic Unity — you must build BlobAssets for collision geometry.
### 1.3 Havok Physics for Unity
 
Havok Physics is a stateful, high-performance physics backend that plugs into the same DOTS/ECS data model as Unity Physics. Same input data, different solver.
 
**Relevance:** Havok's advantages (sleeping, caching, simulation quality) are primarily for dynamic physics simulation — bodies interacting over time. For a batch collision-checking use case where you're doing static overlap queries, Havok provides minimal benefit over Unity Physics.
 
**Licensing change:** Starting with Unity 6.3, Havok Physics is no longer included with Pro/Enterprise/Industry subscriptions. It remains supported through Unity 2022 LTS and Unity 6.0 LTS only. This makes it a poor long-term foundation.
 
**Verdict on Havok:** Not relevant to this use case. Skip it.
 
---
 
## 2. HEADLESS PIPELINE FEASIBILITY
 
### 2.1 Batchmode / No-Graphics
 
Unity supports `-batchmode -nographics` for both Editor and built players. In this mode:
- No graphics device is initialized.
- No GPU is required.
- The engine runs on CPU only.
- Physics queries (OverlapBox, ComputePenetration, raycasts) still function — they don't require the GPU.
You can run a C# script via `-executeMethod` that:
1. Loads the scene with machine/vise/workpiece MeshColliders
2. Parses the G-code file
3. Iterates through each toolpath position
4. Positions the tool collider at each point
5. Calls `Physics.OverlapBox` or `Physics.ComputePenetration`
6. Logs collisions to a report file
7. Calls `Application.Quit()`
This works. People do it for automated testing, CI builds, and server-side simulation.
 
### 2.2 Frame/Time Coupling
 
Even in batchmode, Unity's physics scene queries depend on the physics world being synced. After moving a collider's Transform, you must call `Physics.SyncTransforms()` before queries reflect the new position. Alternatively, `ComputePenetration` takes explicit position/rotation arguments and doesn't require the collider to physically move — this is the better approach for batch checking.
 
You can disable vsync, set `targetFrameRate = -1`, and run as fast as the CPU allows. There is no hard dependency on the game loop for static queries.
 
### 2.3 Docker / CI Deployment
 
Unity headless builds run in Docker containers on Linux. The community `unityci/editor` and `game-ci` Docker images exist for this purpose. However:
- **License activation in headless/Docker is painful.** Unity Personal no longer supports manual activation via the old portal (shut down 2023). Pro/Enterprise licenses can be activated via CLI, but the process involves serial keys or floating license servers.
- **The Docker image is large** — a Unity Editor image is 3–5+ GB. A built Linux player is smaller (~100–500 MB) but still includes the entire Mono/.NET runtime and Unity engine overhead.
- **No official GPU-less physics-only runtime exists.** You're shipping an entire game engine to do geometry intersection tests.
### 2.4 Headless Verdict
 
Technically feasible. Practically heavy. The Unity runtime is ~100–500 MB of overhead for what could be a 5 MB collision library. Every CI job spins up a full game engine to check geometry overlaps. It works, but it's like using Unreal Engine to add two numbers.
 
---
 
## 3. 3D MODEL IMPORT PIPELINE
 
### 3.1 STL Import (Runtime)
 
STL is straightforward. Multiple open-source C# parsers exist:
- **pb_Stl** (GitHub: karl-/pb_Stl) — reads binary and ASCII STL, returns Unity Mesh objects. Works at runtime.
- Manual parsing is simple: STL is just triangle soup (normal + 3 vertices per facet).
Once parsed into a `Mesh` object, you call `meshCollider.sharedMesh = parsedMesh` to create the collider. Unity will "cook" the mesh (build the BVH) on assignment — this is a CPU-bound operation that takes milliseconds for simple meshes, seconds for high-poly ones.
 
### 3.2 OBJ Import (Runtime)
 
Similar story. Multiple runtime OBJ loaders exist on GitHub and the Asset Store. OBJ is text-based, well-understood.
 
### 3.3 STEP Import (Runtime) — PROBLEM
 
STEP is a B-Rep (boundary representation) CAD format. It contains NURBS surfaces, not triangles. Unity cannot read STEP natively. You need a tessellation step (NURBS → triangles) before Unity can use it.
 
**Options:**
- **Unity Asset Transformer (formerly PiXYZ):** Unity's official CAD import tool. Supports STEP, CATIA, JT, NX, and 70+ formats. Available as an Editor plugin and reportedly supports Windows runtime import. However, it's part of the Unity Industry subscription (custom pricing, typically $3,000+/seat/year), and its headless/Linux/batch capabilities are unclear.
- **CAD Exchanger SDK:** Third-party C#/C++ SDK that reads STEP and outputs tessellated meshes. Has a Unity plugin. Commercial license.
- **Open CASCADE (OCCT):** Open-source C++ library that reads STEP. Would need a C++→C# bridge (P/Invoke or a native plugin). Heavy dependency.
- **Pre-convert externally:** Convert STEP → STL/OBJ using FreeCAD, Open CASCADE, or any CAD tool before feeding into Unity. **This is the practical approach.**
### 3.4 MeshCollider Limitations with CAD Meshes
 
- **Non-manifold geometry:** PhysX's mesh cooking can fail or produce garbage on non-manifold meshes (edges shared by >2 faces, self-intersections). CAD exports often produce these. You need a mesh repair step.
- **High poly count:** MeshCollider cooking time scales with triangle count. A 500K-triangle machine body model will take several seconds to cook. At runtime, this blocks the main thread unless you use async cooking (available since Unity 2019).
- **Vertex welding and degenerate triangles:** CAD tessellation can produce near-degenerate triangles that cause PhysX to reject the mesh or produce incorrect collision geometry.
- **Practical limit:** Keep collision meshes under 100K triangles. Use decimated/simplified meshes for collision; keep the full-res mesh for display only (if you're also visualizing).
---
 
## 4. COMPARISON TABLE
 
| Criterion | Python (trimesh/FCL) | C++ (FCL/Bullet) | C# (BulletSharp) | Omniverse (PhysX/Isaac) | **Unity** |
|---|---|---|---|---|---|
| **Collision accuracy** | Exact triangle-mesh (FCL) | Exact triangle-mesh | Exact (Bullet GJK+GImpact) | Exact (PhysX 5) | Game-grade (~0.1mm margin) |
| **Floating-point precision** | double (64-bit) | double (64-bit) | double (64-bit) | float (32-bit) + double option | **float (32-bit) only** |
| **Non-convex vs non-convex** | Yes (FCL mesh-mesh) | Yes (GImpact, FCL) | Yes (GImpact) | Yes (PhysX triangle mesh) | **No** (one must be convex) |
| **Mesh-mesh penetration depth** | Yes (FCL) | Yes | Yes | Yes | **Limited** (ComputePenetration needs one convex) |
| **Headless / no-GPU** | Native | Native | Native | Needs GPU for some features | Yes (-batchmode -nographics) |
| **Docker / CI friendly** | Trivial (pip install) | Moderate (CMake) | Moderate (NuGet) | Heavy (NGC container ~10GB) | **Heavy** (~3-5 GB image) |
| **Startup overhead** | ~100 ms | ~10 ms | ~50 ms | ~5-10 s | **~3-10 s** (engine init) |
| **Per-query latency** | ~0.01–0.1 ms (FCL) | ~0.001–0.01 ms | ~0.005–0.05 ms | ~0.001 ms (GPU batch) | ~0.01–0.1 ms (PhysX query) |
| **Dense toolpath (100K points)** | 1–10 s | 0.1–1 s | 0.5–5 s | 0.01–0.1 s (GPU) | **1–10 s** |
| **STEP import** | Yes (OCP/trimesh) | Yes (Open CASCADE) | Manual | Yes (native) | **No** (needs PiXYZ/$$$, or pre-convert) |
| **STL/OBJ import** | Native | Yes | Yes | Yes | Yes (with parser lib) |
| **G-code parser** | Many (pygcode, etc.) | Custom or gcodetools | Custom | Custom | **Custom** (gsGCode C# exists) |
| **Scripting language** | Python | C++ | C# | Python + C++ | **C#** |
| **Python API** | Native | Via bindings | Via IronPython | Native | **No** (C# only, or IPC) |
| **License for industrial use** | MIT/BSD | BSD/LGPL | MIT/zlib | Free (Omniverse) + NVIDIA EULA | **$2,200+/yr** (Pro) or **$3,000+/yr** (Industry) |
| **Deployment binary size** | ~50 MB (conda env) | ~5 MB | ~10 MB | ~10 GB | **~200-500 MB** |
| **Visualization built-in** | Minimal (matplotlib) | No | No | Yes (excellent) | **Yes (excellent)** |
| **Integration complexity** | Low | Medium | Medium | High | **High** |
 
---
 
## 5. GOTCHAS AND SHOWSTOPPERS
 
### 🔴 SHOWSTOPPER: Non-convex vs. Non-convex Collision
 
Unity/PhysX cannot detect collisions between two non-convex MeshColliders. If your tool is a standard endmill (cylinder + hemisphere), a convex MeshCollider or CapsuleCollider works fine. But if you ever need to check a complex, non-convex tool geometry (form tool, dovetail cutter, T-slot cutter) against a non-convex workpiece in a concave fixture pocket, Unity cannot do it natively. You'd need to decompose the non-convex shape into multiple convex hulls — an approximation that reduces accuracy.
 
**FCL and BulletSharp can do exact non-convex mesh-mesh collision. Unity cannot.**
 
### 🟡 SIGNIFICANT: Single-Precision Float
 
Unity's physics world is 32-bit float. This gives ~7 significant digits. For a 500mm work envelope, precision is ~0.05 μm near the origin — fine for collision detection. But if your machine model is positioned at a large offset from the origin (e.g., the table surface is at Y=1500mm in world space), precision degrades. Keep the scene centered near the origin.
 
This is manageable, not a showstopper, but it's a constraint you don't have with FCL or BulletSharp (both support double precision).
 
### 🟡 SIGNIFICANT: Unity Runtime Overhead
 
To check "does cylinder A overlap mesh B at position XYZ," you need to:
1. Install Unity Editor (~5 GB)
2. Create a Unity project
3. Build a headless Linux player (~200-500 MB)
4. Deploy and run that player
Compare with BulletSharp: `dotnet add package BulletSharp`, write 100 lines of C#, compile to a 10 MB binary. Done.
 
The Unity approach carries enormous incidental complexity for a geometry intersection task.
 
### 🟡 SIGNIFICANT: Licensing
 
- **Unity Personal** (free): Available if your organization has <$200K revenue/funding. Supports batchmode and nographics. No Havok Physics. Technically usable for this purpose.
- **Unity Pro** ($2,200/seat/year): Required if >$200K revenue. Includes Havok Physics (through Unity 6.0 LTS only).
- **Unity Industry** (custom, ~$3,000+/seat/year): Required for non-game/entertainment applications if >$1M revenue. Includes Asset Transformer (PiXYZ) for STEP import.
If your company makes CNC machines or sells simulation software and has >$1M revenue, you need Unity Industry. This is a recurring per-seat cost for what might be a batch tool running on a server.
 
By contrast: FCL is BSD-licensed, BulletSharp is MIT/zlib, trimesh is MIT. Zero licensing cost, zero restrictions.
 
### 🟡 MODERATE: Physics.SyncTransforms() Gotcha
 
When you move a GameObject's Transform, the physics world doesn't update immediately. You must call `Physics.SyncTransforms()` or wait for `FixedUpdate()`. In a batch loop iterating 100K toolpath positions, forgetting this call means all your overlap queries test against stale positions. `ComputePenetration` with explicit position arguments avoids this, but `OverlapBox` does not.
 
### 🟡 MODERATE: MeshCollider Cooking Time
 
Every time you assign a new mesh to a MeshCollider, Unity "cooks" it (builds the internal BVH). For a 50K-triangle vise model, this takes ~200-500 ms. For the initial scene setup this is fine (one-time cost). But if you're modifying the workpiece mesh during simulation (material removal), re-cooking on every change is expensive. Pre-compute the workpiece mesh and don't change it during the collision pass.
 
### 🟢 MINOR: No Built-in G-code Parser
 
You'll need to write or integrate a G-code parser. The C# library **gsGCode** (GitHub: gradientspace/gsGCode) exists but is focused on 3D printing, not milling. For CNC milling G-code (G00/G01/G02/G03 with IJK arcs, canned cycles, tool changes), you'll likely need a custom parser. This is true for every stack, not Unity-specific.
 
### 🟢 MINOR: Burst Compiler for Parallel Checking
 
If you go the DOTS route, the Burst Compiler + Job System can parallelize overlap checks across CPU cores. You'd batch toolpath positions into jobs, each checking a subset. This is powerful but requires restructuring your code into the DOTS ECS pattern — a significant investment for what amounts to a parallel for-loop that could be done with `Parallel.For` in plain C# with BulletSharp.
 
---
 
## 6. CODE EXAMPLES AND STARTING POINTS
 
### Relevant GitHub Repos
 
| Resource | Description |
|---|---|
| [gradientspace/gsGCode](https://github.com/gradientspace/gsGCode) | C# G-code parsing and manipulation library (3D printing focused, but the parser handles G0/G1/G2/G3) |
| [karl-/pb_Stl](https://github.com/karl-/pb_Stl) | STL import/export for Unity, binary and ASCII, works at runtime |
| [sanukin39/UniColliderInterpolator](https://github.com/sanukin39/UniColliderInterpolator) | Approximates non-convex meshes with compound box colliders for better collision accuracy |
| [Unity DOTS Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples) | Official Unity ECS samples including physics queries |
| [unityci/docker](https://github.com/game-ci/docker) | Docker images for Unity CI (headless builds and testing) |
 
### Unity Asset Store (Commercial)
 
- **Non Convex Mesh Collider** (Asset Store) — decomposes non-convex meshes into convex sub-colliders. Useful workaround for the convex limitation.
- **Runtime OBJ Importer** — various options for runtime mesh loading.
- **Unity Asset Transformer (PiXYZ)** — STEP/CATIA/JT import, part of Unity Industry.
### CNC Simulation References (Non-Unity)
 
- **CAMotics** — open-source 3-axis G-code simulator with collision detection (C++, OpenGL). Good reference architecture.
- **Vericut** — industry-standard commercial CNC simulation with full collision checking. The benchmark for what "correct" looks like.
- **LinuxCNC/EMC2** — open-source CNC controller with simulation and 3D visualization.
### No Existing Unity CNC Collision Project Found
 
Despite searching, I found no public GitHub repository or Asset Store package that implements CNC G-code collision detection in Unity. There are scattered forum posts (a student project controlling an EdingCNC from Unity, a discussion about writing a G-code reader in Unity C#), but nothing that constitutes a reusable starting point for collision checking.
 
---
 
## 7. WHEN UNITY MAKES SENSE (AND WHEN IT DOESN'T)
 
### USE UNITY IF:
- You need **real-time 3D visualization** of the CNC simulation alongside collision checking (operator review, training tool, digital twin dashboard).
- You're building a **full CNC simulation application** with interactive controls, camera views, and visual feedback — and collision detection is one feature among many.
- Your team already has strong Unity/C# expertise and the project budget supports Unity Industry licensing.
- You plan to deploy on **multiple platforms** including VR/AR for shop floor training.
### DON'T USE UNITY IF:
- Your goal is a **batch collision-checking pipeline** that reads G-code and outputs a collision report. This is what the question describes.
- You need **exact non-convex mesh-mesh collision** without approximation.
- You want a **lightweight, deployable tool** for CI/CD or server-side batch processing.
- **Licensing cost matters** — the free alternatives (FCL, BulletSharp, trimesh) are functionally superior for this specific task.
- You need a **Python API** — Unity is C# only; bridging to Python adds IPC complexity.
---
 
## 8. RECOMMENDED APPROACH (BASED ON ALL RESEARCH)
 
For the specific requirements stated (batch G-code collision checking → report with line number, XYZ, colliding pair):
 
**Primary recommendation: C# with BulletSharp (or direct FCL via P/Invoke)**
- Native C# — same language skill as Unity, but without the engine overhead.
- Non-convex mesh-mesh collision supported (GImpact algorithm).
- Double-precision float available.
- Deploys as a ~10 MB console application.
- Trivial Docker/CI integration.
- MIT/zlib license.
**Secondary recommendation: Python with trimesh + FCL**
- Fastest prototyping.
- Excellent mesh manipulation ecosystem.
- Easy STEP import via OCP (Open CASCADE Python bindings).
**Unity: third choice**, and only if visualization requirements emerge later.
 
---
 
*Research conducted June 2026. Unity version references: Unity 6.x (2024–2026 LTS cycle). Pricing as of January 2026 adjustments.*