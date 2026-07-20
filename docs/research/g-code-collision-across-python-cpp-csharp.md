# G-code collision detection across Python, C++, and C#
 
**Bottom line up front**: For a kinematic CNC collision checker, the strongest open-source foundation is **FCL or its modern fork Coal/hpp-fcl**, which underpins MoveIt, Drake, and Pinocchio. The recommended stack-by-stack picks are **Python: `trimesh` + `python-fcl`** (best ergonomics, MIT/BSD, one-line install), **C++: `Coal` (hpp-fcl)** with **Bullet** as a runner-up when continuous collision detection is mandatory, and **C#: `geometry3Sharp` + `Assimp.Net`** for a pure-managed solution, escalating to **BulletSharpPInvoke** when you need swept-volume CCD or rich contact data. Cross-stack, the **C++ → Python path** is by far the most mature; **C# has no first-class FCL binding** and is the weakest leg. If you can pick freely, **Python with FCL** offers the best ratio of accuracy, performance, and integration effort for an open-source CNC collision verifier. No existing open-source CNC simulator (CAMotics, FreeCAD CAM, LinuxCNC) actually does tool-vs-machine-body collision — they only voxelize material removal — so you will be assembling this from collision-library and G-code-parser building blocks rather than extending a finished simulator.
 
## How existing CNC simulators handle (or don't handle) collision
 
The open-source CNC simulator landscape is **misleadingly thin on collision detection**. Every mainstream project stops at material removal and stops short of tool-vs-fixture or tool-vs-machine-body checks. **CAMotics** (C++/GPL) is the most polished open-source simulator and explicitly lists "tool collision detection / fixture collision detection" as a roadmap TODO; the project site warns "does not yet detect over/under cutting, collisions with the tool shaft or fixtures, or rapid moves in the material." **FreeCAD's Path/CAM workbench** has a voxel cut simulator but its own maintainers acknowledged in Issue #22178 (June 2025) that the workbench "doesn't provide consistent tools for detecting whether a toolpath collides with the model, machine, clamps, vises, or other workholding." **LinuxCNC's Vismach** is pure OpenGL visualization — a forum thread confirms "OpenGL does not support 3D STL interference checking" — and the VTK-vismach fork doesn't change that.
 
Only the **commercial** tools (Vericut, NCSIMUL Machine, Eyeshot) ship real machine-vs-tool collision detection, and they uniformly use a **swept-volume** approach rather than point-sampling: per CGTech, "rather than just checking points along a path, Vericut checks along the entire path of travel by sweeping through space." That's the algorithmic target you want to imitate.
 
The useful open-source byproducts are: **OpenCAMLib (OCL)** for cutter primitives (cylindrical, ball, toroidal, conical, snubnose) and drop-cutter math, **CAMotics** and **LinuxCNC/`rs274ngc`** as reference G-code interpreters with full RS-274 modal state and arc tessellation, and **`pygcode`** (Python) or **`gcode-rs` / `ngc`** (Rust) for parsing with line-number spans. None of these handle collision; you bolt a collision library onto them.
 
## Library landscape per stack
 
The collision space across all three stacks is dominated by a small family of C++ engines and their language bindings. The table below consolidates licenses, query support, and binding maturity.
 
| Library | Language | License | Mesh-vs-mesh | Distance | Pen. depth | CCD / sweep | Broadphase | Python binding | C# binding | Maintained 2025+ |
|---|---|---|---|---|---|---|---|---|---|---|
| **FCL** | C++ | BSD-3 | ✅ BVH | ✅ | ✅ | ✅ time-of-contact | Dyn-AABB | `python-fcl` (mature) | none | slow but alive |
| **Coal / hpp-fcl** | C++ | BSD-2 | ✅ BVH + convex | ✅ + lower bound | ✅ EPA | limited | Dyn-AABB | `coal` (official) | none | very active |
| **Bullet** | C++ | zlib | ⚠ via GImpact/convex | via GJK | ✅ EPA | ✅ `convexSweepTest` | Dbvt | PyBullet | **BulletSharp/PInvoke** | active |
| **CGAL** | C++ | **GPL / commercial** | ✅ exact | ✅ exact | ❌ | ❌ | AABB tree | partial | partial | very active |
| **libccd** | C | BSD-3 | convex only | ✅ | ✅ MPR/EPA | ❌ | ❌ | (via FCL) | none | stable |
| **PQP / SOLID / V-COLLIDE** | C++ | non-commercial / GPL/QPL | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | abandoned |
| **Embree** | C++ | Apache-2 | broad-phase + custom narrow | point query | ❌ | rays only | best-in-class BVH | community | none mature | very active |
| **Jolt Physics** | C++ | MIT | ✅ | ✅ | ✅ | ✅ shape cast | ✅ | community | **JoltPhysicsSharp** | active |
| **trimesh** | Python | MIT | ✅ (via FCL) | ✅ | ✅ | ❌ (wrapper) | ✅ | native | — | very active |
| **Open3D** | Py/C++ | MIT | ❌ (rays + SDF only) | ✅ | ❌ | rays | Embree-backed | ✅ | ❌ | active |
| **manifold3d** | Py/C++ | Apache-2 | wrong abstraction (booleans) | ❌ | ❌ | ❌ | internal | ✅ | ❌ | very active |
| **PyVista / VTK** | Py/C++ | MIT/BSD | ✅ `vtkCollisionDetectionFilter` (face-only) | ❌ | ❌ | ❌ | per-pair OBB | ✅ | C++ only | very active |
| **geometry3Sharp** | C# | Boost (BSL-1.0) | ✅ dual-BVH `TestIntersection` | ✅ | via SDF | ❌ | `DMeshAABBTree3` | — | native | active (dotnet8 branch) |
| **BepuPhysics 2** | C# | Apache-2 | ⚠ mesh↔convex strong, mesh↔mesh weak | ✅ | ✅ | ✅ sweep | ✅ | — | native | very active |
| **GeometRi** | C# | MIT | only `ConvexPolyhedron↔ConvexPolyhedron` | primitives only | ❌ | ❌ | ❌ | — | native | active |
 
The single most important license trap is **CGAL**: its `AABB_tree` and `Polygon_mesh_processing` packages are GPL-3 or commercial via GeometryFactory. Adopting CGAL in any closed-source CNC product without buying a commercial license effectively GPL-infects your codebase. Every other recommended library here (FCL, Coal, Bullet, trimesh, geometry3Sharp, Bepu, Jolt) is permissive (BSD/MIT/zlib/Apache/Boost) and safe for commercial use.
 
## Python: the path of least resistance
 
**`trimesh.collision.CollisionManager` over `python-fcl`** is the canonical answer and the one robotics practitioners reach for first. Trimesh handles I/O for STL, OBJ, PLY, GLTF, 3MF, OFF and even STEP via the `cascadio` companion package, exposes primitive endmill/holder shapes (`Cylinder`, `Capsule`, `Sphere`), and wraps the FCL `DynamicAABBTreeCollisionManager` broad-phase so you register the static scene once and only update the tool's transform per G-code step. The query returns contact points, **penetration depth**, and the names of the colliding objects — exactly the report fields you need. Install is a single `pip install trimesh python-fcl`, both MIT/BSD, both with prebuilt wheels for CPython 3.14 across Linux/macOS/Windows.
 
```python
from trimesh.collision import CollisionManager
scene = CollisionManager()
for name, path in [('machine','machine.stl'), ('vise','vise.stl'), ('stock','part.stl')]:
    scene.add_object(name, trimesh.load(path))
tool = trimesh.primitives.Capsule(radius=3, height=30)         # or load a holder STL
 
for line_no, (x, y, z) in interpolated_points:
    T = np.eye(4); T[:3,3] = [x, y, z]
    hit, names, contacts = scene.in_collision_single(tool, transform=T,
                                                     return_names=True, return_data=True)
    if hit:
        report.append({'line': line_no, 'xyz': (x,y,z),
                       'objects': sorted(names),
                       'depths': [c.depth for c in contacts]})
```
 
The one feature trimesh hides from you is **continuous collision detection** between two waypoints. For G0 rapids that can fly past thin fixture features between samples, drop down to **`python-fcl` directly** and call `fcl.continuousCollide(tool_old_tf, tool_new_tf, static, static_tf, req, res)`, which returns a `time_of_contact ∈ (0,1]` on the segment. MoveIt uses exactly this two-tier pattern: FCL for discrete, Bullet for CCD. If you want raw performance and a built-in `security_margin` (positive = early-warning band, perfect for "warn if the tool gets within 0.5 mm of the vise"), use **Coal** (`pip install coal`); its Nesterov-accelerated GJK is 5–15× faster than legacy FCL on convex-vs-convex narrow phase.
 
**What not to use in Python**: Open3D and PyVista's `vtkCollisionDetectionFilter` only detect face-intersection — they miss the case where the tool is fully inside a fixture without any triangle crossings, which is precisely the scenario you must catch. Manifold3d is a Boolean library and the wrong abstraction. PyMesh is stalled and a Windows-build nightmare. PyMeshLab is GPL. Direct CGAL bindings drag the GPL into your product.
 
## C++: maximum performance, weakest C# story
 
For a C++ implementation, **Coal (formerly hpp-fcl)** is the clear technical winner. It is BSD-2, very actively developed by INRIA Willow / LAAS-CNRS (3,700+ commits, 59 releases, version 3.0.2 in Sept 2025), ships a built-in Assimp-based `MeshLoader` that reads `.stl/.obj/.dae` directly, exposes box / sphere / capsule / cylinder / cone / convex / BVH / heightfield / octree primitives, and adds **swept-sphere primitives** that are an excellent envelope model for ball-end mills. Its **`security_margin` feature** maps cleanly to CNC tolerance bands. The Montaut et al. paper shows its accelerated GJK is **5–15× faster** than Bullet, legacy FCL, and libccd on convex narrow-phase queries.
 
```cpp
auto tool  = loadConvexMesh("endmill.stl");          // Coal's MeshLoader + Qhull
auto scene = loadConvexMesh("vise.stl");
coal::Transform3s T1; T1.setTranslation({x, y, z});
coal::CollisionRequest req; req.security_margin = 1e-3;   // 1 mm warning band
coal::CollisionResult res;
coal::collide(tool.get(), T1, scene.get(), coal::Transform3s::Identity(), req, res);
if (res.isCollision()) {
    auto c = res.getContact(0);
    log(line_no, c.penetration_depth, c.nearest_points[0], c.normal);
}
```
 
The **trade-off vs original FCL** is continuous collision detection: Coal trimmed FCL's CCD when forking, so for true time-of-contact between waypoints, fall back to **FCL 0.7's `continuousCollide()`** or use **Bullet's `convexSweepTest`**. Bullet (zlib license) is uniquely valuable for CNC because its sweep test answers the question "does the tool, moving from G-code point A to point B, sweep through the scene?" directly without discretization aliasing — and it's the **only C++ collision library with a mature C# binding**.
 
For STEP loading in C++ use **OpenCascade Technology (OCCT)** with `STEPControl_Reader` → `BRepMesh_IncrementalMesh` → triangle arrays into Coal/FCL. **Avoid CGAL** unless you have a commercial license; **avoid PQP/SOLID/V-COLLIDE** (non-commercial licenses and abandoned).
 
## C#: pure-managed is workable, but escalation paths are limited
 
C# is the weakest of the three ecosystems for this problem. There is **no mature .NET binding for FCL or Coal**, and no NuGet package providing a one-line `Install-Package FCL.NET`. The realistic options are: a pure-C# solution with **`geometry3Sharp`**, a native-backed solution with **`BulletSharpPInvoke`**, or rolling your own P/Invoke wrapper around a C ABI shim of FCL/Coal (estimated 1–2 weeks).
 
**`geometry3Sharp` (Boost License, gradientspace)** is the strongest pure-managed option. Its `DMeshAABBTree3.TestIntersection(IMesh testMesh, Func<Vector3d,Vector3d> transformF)` and `FindAllIntersections(otherTree)` perform proper dual-BVH descent — exactly the algorithm you would write by hand. The `transformF` callback re-poses the tool per G-code step without rebuilding the BVH, double-precision throughout (matters at CNC tolerances), and you can precompute a `MeshSignedDistanceGrid` on the workpiece for **fast clearance / penetration sampling** along dense toolpaths. Built-in `StandardMeshReader` loads STL/OBJ/OFF. The `dotnet8` branch adds GLTF support. NuGet is `geometry3Sharp 1.0.324`, and the MatterHackers fork is used in shipping products.
 
```csharp
var workBvh = new DMeshAABBTree3(StandardMeshReader.ReadMesh("workpiece.stl"), true);
var toolMesh = StandardMeshReader.ReadMesh("tool.stl");
Func<Vector3d,Vector3d> toolToWorld = v => Rotation(line) * v + Position(line);
if (workBvh.TestIntersection(toolMesh, toolToWorld)) {
    var hits = workBvh.FindAllIntersections(new DMeshAABBTree3(toolMesh, true), toolToWorld);
    foreach (var seg in hits.Segments) report.Add(line_no, seg, "tool", "workpiece");
}
```
 
The gaps are real and worth naming: geometry3Sharp gives you boolean intersection and intersection segments, but **no native penetration depth, no witness points, no swept CCD**. For those you need **`BulletSharpPInvoke`** (zlib via Bullet, MIT wrapper; the most active fork is `JAJ.Packages.BulletSharp` with native binaries for Win/Linux/macOS-ARM64). Use `btBvhTriangleMeshShape` for the static scene (machine/vise/workpiece), a `btConvexHullShape` or `btCompoundShape` for the tool, and call `ContactPairTest` for discrete checks and `convexSweepTest` for proper CCD across G-code segments. **Don't** try to do `btBvhTriangleMeshShape ↔ btBvhTriangleMeshShape` — Bullet returns no contacts for that pair; the standard workaround (which fits CNC perfectly anyway) is to keep the tool convex.
 
**BepuPhysics 2** (Apache-2) is the most modern pure-C# physics engine and is extremely fast (AVX SIMD), but its first-class collision pairs are mesh-vs-convex and mesh-vs-primitive — full mesh-vs-mesh contact manifolds aren't there yet (issue #258). For CNC, this is acceptable if your tool stays primitive/convex/compound. **GeometRi**, **MathNet.Spatial**, **HelixToolkit**, and **Triangle.NET** are not suitable: GeometRi only handles `ConvexPolyhedron↔ConvexPolyhedron` without a BVH, MathNet.Spatial is vectors/planes only, HelixToolkit is rendering-only (the maintainer confirms "no collision detection is implemented"), and Triangle.NET is 2D only despite the name. **RhinoCommon** has the best mesh-mesh API of any C# library (`Intersection.MeshMesh`, `MeshClash.Search`) but requires a Rhino license and Rhino.Inside, which makes it unsuitable for standalone redistribution. **MeshLib** advertises C# collision detection but requires a commercial license for products.
 
## Cross-language interoperability
 
The cross-language story is asymmetric. **C++ → Python is solved**: `pybind11` is the modern standard, and `python-fcl` (Cython-based) and `coal` (pybind11) already exist and are maintained. **C++ → C# is the bottleneck**: no FCL/Coal binding exists. Bullet has **`BulletSharp` (C++/CLI, Windows)** and **`BulletSharpPInvoke` (cross-platform)** under MIT, which is the only mature collision-library C# binding in the entire space. The route to use FCL/Coal from .NET is:
 
1. Wrap the templated C++ API behind a thin `extern "C"` shim (~30–60 functions for the useful subset: `BVHModel`, `CollisionObject`, `Sphere/Box/Cylinder/Capsule`, `collide()`, `distance()`, `continuousCollide()`, `DynamicAABBTreeCollisionManager`).
2. Use **`ClangSharpPInvokeGenerator`** to auto-generate the P/Invoke layer from the C headers of your shim.
3. Use .NET 7+ source-generated P/Invoke (`LibraryImportAttribute`).
4. Ship native binaries per RID under `runtimes/{rid}/native/` in a NuGet package.
This is a meaningful but bounded engineering investment. The alternative — `geometry3Sharp` + Bullet for sweeps — gets you 90% of the way there with zero binding work. **C++/CLI** is a third option but Windows-only and discouraged for new projects.
 
## A reference architecture you can implement in any of the three stacks
 
The pipeline is essentially identical across stacks; only the library calls change.
 
```
┌──────────────────────────────────────────────────────────────────────┐
│  INPUT: G-code (.nc), tool DB, scene meshes (STEP/STL), kinematics    │
├──────────────────────────────────────────────────────────────────────┤
│  PARSE + LOAD                                                         │
│   • STEP→mesh via OCCT (C++) / pythonocc (Py) / OCCT.NET (C#)         │
│   • STL/OBJ via trimesh / Assimp / Assimp.Net                         │
│   • G-code via pygcode / gcode-rs / hand-rolled RS-274 modal state    │
├──────────────────────────────────────────────────────────────────────┤
│  INTERPOLATE                                                          │
│   • G0/G1: Δs = min(0.1·r_tool, ε_chord, max_feed·servo_dt)           │
│   • G2/G3: θ_step = 2·acos(1 − ε_chord/r), recursive bisection        │
│   • canned cycles → expand to G0/G1 micro-segments                    │
├──────────────────────────────────────────────────────────────────────┤
│  KINEMATIC TRANSFORM (4/5-axis chain → world pose for each body)      │
├──────────────────────────────────────────────────────────────────────┤
│  COLLISION ENGINE                                                     │
│   • Static scene → DynamicAABBTreeCollisionManager / BVH (build once) │
│   • Tool/holder/spindle → CollisionObject with capsule+cyl or BVH     │
│   • Per (T_i, T_{i+1}): continuousCollide() → t_oc, contacts          │
│   • Optional: distance query for clearance grading                    │
├──────────────────────────────────────────────────────────────────────┤
│  REPORT                                                               │
│   CollisionEvent { line_no, motion_cmd, xyz, t_oc∈[0,1],              │
│                    pair=("tool_holder","vise_jaw_L"),                 │
│                    depth, normal, severity }                          │
│   Outputs: JSON, annotated G-code (red lines), GLB visualization      │
└──────────────────────────────────────────────────────────────────────┘
```
 
The non-obvious design decisions are: (1) **keep the workpiece separate** from the static scene if you also intend to simulate material removal — the cutting portion of the tool legitimately overlaps with stock; only the non-cutting shaft/holder must collide with the (decreasing) stock. The Dassault patent USPTO 10140395 describes this clean separation between cutting and non-cutting tool sub-meshes. (2) **CCD is non-negotiable for rapids**: G0 moves can fly tens of millimeters per servo cycle, and discrete sampling at 0.25 mm steps will still miss thin obstacles between samples. Either use `fcl.continuousCollide()` (Python/C++) or `btCollisionWorld::convexSweepTest()` (Bullet, also in C#). (3) **Model the tool as primitives where possible**: a flat endmill = cylinder + disc, ballnose = cylinder + hemisphere, bullnose = cylinder + torus. Primitive-vs-mesh queries via GJK are an order of magnitude faster than mesh-vs-mesh BVH, and exact rather than tessellated.
 
## Final cross-stack verdict
 
**If you can choose the stack freely, build this in Python.** The combination of `trimesh` (I/O, scene management, primitives), `python-fcl` (CCD, distance, contacts), and `pygcode` (or a tiny RS-274 interpreter) gives you the **shortest path to a working prototype**, the **most permissive license stack** (MIT + BSD), and a **direct upgrade path to Coal** if you later need 5–15× narrow-phase speedups. The same C++ engine (FCL/Coal) that powers MoveIt, Drake, and Pinocchio is one `pip install` away, and the broader robotics community has already solved every adjacent problem you'll hit (mesh repair, BVH tuning, broadphase, CCD-vs-discrete trade-offs).
 
**If you must use C++** (because performance is paramount or you're integrating into an existing C++ CAD/CAM kernel), use **Coal** as the collision engine with **OCCT** for STEP I/O and the **CAMotics** or **LinuxCNC `rs274ngc`** interpreter for G-code parsing. Fall back to Bullet's `convexSweepTest` for CCD. Expect best-in-class performance and the same algorithms used in cutting-edge robotics research.
 
**If you must use C#** (because of an existing WPF/Avalonia front end or .NET-only deployment constraints), the realistic plan is **geometry3Sharp + Assimp.Net for the 80% case, BulletSharpPInvoke for swept CCD and rich contact data**. Be aware you're working against the grain of the ecosystem: there is no FCL/Coal binding, and you may eventually want to invest 1–2 weeks in writing your own P/Invoke wrapper around a C shim of Coal. The good news is geometry3Sharp's `DMeshAABBTree3.TestIntersection` and `MeshSignedDistanceGrid` cover most kinematic CNC needs in pure managed code.
 
A final honest note: regardless of stack, **no existing open-source CNC simulator does what you're building**. CAMotics, FreeCAD CAM, LinuxCNC, and NCViewer all stop at material removal. The closest analogue is **robotic-arm CCD via FCL/Coal** (as in MoveIt or Pinocchio), and the techniques from that literature — broadphase + BVH narrow phase + CCD between waypoints — transfer directly to G-code verification. You're not extending a finished simulator; you're assembling a kinematic collision verifier from solid building blocks