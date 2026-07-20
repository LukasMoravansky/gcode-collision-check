# Pure-Python G-code collision checking: tools, tradeoffs, and a pipeline
 
**Use `trimesh.collision.CollisionManager` (backed by `python-fcl`) as the collision engine, `pygcode` as the G-code interpreter, and `cascadio`/`build123d` for STEP import.** This stack is the only combination that is (a) pure-Python-installable via pip, (b) gives you exactly the API shape your problem needs — "load the static scene once, move the tool through thousands of poses, report which pairs collided" — and (c) returns the contact pair names and contact points needed for a detailed report. No existing open-source project does the whole job end-to-end; you will assemble it from these pieces. The closest non-Python reference, **CAMotics**, explicitly *does not* check tool-shaft, holder, or fixture collisions (it is a material-removal verifier), so it cannot be relied on for your use case. FreeCAD Path's simulator has the same gap and is tracked in open issue #22178. The robotics motion-planning ecosystem (URDF + FCL + FK) is, in practice, the right architectural analog to copy.
 
---
 
## Ranked recommendation
 
1. **`trimesh.collision.CollisionManager` + `python-fcl`** — best fit. Drop-in match for "tool vs. N static objects" with named contact pairs, `set_transform` to move the tool without rebuilding the BVH, optional `min_distance_single` for conservative advancement. MIT/BSD, wheels for cp310–cp314 on Linux/macOS/Windows. Sub-millisecond queries on reasonable meshes.
2. **`coal` (formerly `hpp-fcl`)** — graduate here if you need a *clearance margin* report ("warn within 1 mm"), higher throughput (5–15× faster GJK), or lower-bound distance on no-collision frames. Same engine family, BSD-3, conda-forge/PyPI wheels through 2025.
3. **`python-fcl` directly** — pick when you outgrow trimesh's wrapper and need raw `CollisionRequest(num_max_contacts=N, enable_contact=True)`, custom callbacks, or `continuousCollide()` between consecutive toolpath samples (catches tunneling through thin walls).
4. **PyBullet** — only if you will later add full physics or robot-arm IK. Tool must be convex or convex-decomposed (CoACD), and meshes must come in as OBJ or vertex arrays. Heavier than FCL for a query-only workload.
5. **Don't use for this** — Open3D (no mesh-mesh primitive, only SDF/raycast), manifold3d (CSG kernel, no collision API), CAMotics/FreeCAD/LinuxCNC stock simulators (no shaft/holder/fixture collision detection), PyCAM (Python 2, dead since 2019).
---
 
## Comparison of options
 
| Component | Tool | License | Maintenance (2025–26) | Mesh-mesh concave | Contact pair info | Pure-Python install | Verdict for this use case |
|---|---|---|---|---|---|---|---|
| **Collision** | `trimesh + python-fcl` | MIT + BSD | Very active (4.x, 3.6k★) | ✅ BVH | ✅ `return_names`, `return_data` | `pip` wheels everywhere | **Pick this** |
| | `python-fcl` direct | BSD | Active, wheels through 2026 | ✅ BVH | ✅ contacts + geom IDs | `pip` wheels | More control, more boilerplate |
| | `coal` / `hpp-fcl` | BSD-3 | Very active (3.0.2 Sep 2025) | ✅ BVH + convex (GJK++) | ✅ + contact patches | conda-forge best, PyPI wheels | Best perf; clearance margin |
| | PyBullet | zlib | Maintenance mode | ✅ static; dynamic needs convex decomp | ✅ via `getClosestPoints` | `pip install pybullet` | Workable, engine weight |
| | Open3D RaycastingScene | MIT | Very active | ❌ (SDF/ray only) | Partial | `pip install open3d` | Wrong primitive |
| | manifold3d | Apache-2.0 | Active | ❌ no collision API | ✗ | wheels | Wrong tool (CSG only) |
| **G-code parsing** | `pygcode` | GPL-3 | Stalled (0.2.1, 2017) but 141★ | n/a | n/a | `pip install pygcode` | **Only Python parser with proper modal state + arc-linearize**; fork if license matters |
| | `gcodeparser` | MIT | Active | n/a | n/a | pip | Token-level only; you'd add modal state yourself |
| | `gcode-lib` (hyiger) | MIT | New (2025–26) | n/a | n/a | pip | 3D-print bias; weak on G54/G43/canned cycles |
| | `PythonicGcodeMachine` | open | Dormant | n/a | n/a | pip | Formal RS-274 grammar; useful reference |
| | `mecode` | MIT | Active | n/a | n/a | pip | **Writes** G-code, doesn't parse — exclude |
| **STEP loading** | `cascadio` (→ trimesh) | LGPL | Moderate, wheels | n/a | n/a | `pip` wheels (no MUSL) | Easiest STEP→mesh path |
| | `build123d` / `cadquery-ocp` | Apache-2.0 | Very active | n/a | n/a | `pip` wheels | Use when you also want parametric BREP |
| **Cutter geometry** | `opencamlib` | LGPL | Moderate | n/a | n/a | `pip install opencamlib` | Reference cutter shapes (cyl/ball/bull/cone); drop/push-cutter for surface gouging |
| **Convex decomposition** | `coacd` (SIGGRAPH '22) | MIT | Very active (1.0.11 May 2026) | n/a | n/a | `pip` wheels | Recommended over V-HACD (EOL) |
| **Existing simulator** | CAMotics | GPL-2+ | Active | n/a (voxel cut sim) | ✗ shaft/fixture collisions | C++ binary | Visual baseline only — **does not catch your collisions** |
| | FreeCAD Path | LGPL | Active | n/a | ✗ (issue #22178) | Python-scriptable | Useful CAD kernel; collision insufficient |
| | LinuxCNC | GPL | Active | ✗ | ✗ | `linuxcnc` Python module | No STL interference checking |
| | PyCAM | GPL-3 | Dead (2019) | n/a | ✗ | abandoned | Read for algorithms only |
 
---
 
## Why no off-the-shelf project solves this
 
CAMotics' own documentation explicitly states it does *not* detect "collisions with the tool shaft or fixtures or rapid moves in the material" — exactly the case you care about (open issue #358 since ~2020). FreeCAD Path's wiki warns "operations within the Path workbench are not aware of clamping mechanisms" (issue #22178 open). LinuxCNC's AXIS/vismach do soft-axis limits only. PyCAM did include cutter-vs-STL collision but only for *toolpath generation*, not G-code verification, and has been unmaintained since 2019. NCViewer, OpenBuildsCAM, bCNC, gcodesimulator are backplot/visualization only. The single closest community recipe — visible across Medium tutorials and the `cilynx/pygdk` ecosystem — is the do-it-yourself stack of `pygcode + trimesh + python-fcl`, which is what we recommend assembling.
 
---
 
## How the FCL-backed query loop should be structured
 
The dominant performance pattern is **two named collision managers**: one holding the static scene (machine body, vise, workpiece, fixture plate) and one holding the moving tool assembly (flute, shank, holder, spindle nose). Add objects once; per sample, only update transforms via `set_transform()` — never re-add — because re-adding tears down and rebuilds FCL's `DynamicAABBTreeCollisionManager`. Querying with `obstacles.in_collision_other(tool_grp, return_names=True, return_data=True)` returns the pair names and full `ContactData` (point, normal, penetration depth, face index), which is exactly the report format you need. For long approach moves, `min_distance_single` enables **conservative advancement** — jump forward by the queried clearance distance, which beats uniform sampling by an order of magnitude on rapids above the part. A Z-prefilter (skip when tool tip is above `Z_max(obstacles)`) often removes 60–90% of samples in real programs.
 
**Include the holder, collet, and spindle nose in the tool group.** Most real crashes are not the flutes — they are the ER collet nut or quill hitting a tall vise jaw or workholding clamp. Build each as a separate sub-mesh registered with its own name so the report can attribute the hit to `("holder", "vise_jaw_left")` rather than just "the tool". CAMotics' tool catalog (`cylindrical, conical, ballnose, spheroid, snubnose`) and FreeCAD's parametric `.fcstd` toolbits are good references for the cutter portion.
 
---
 
## Toolpath interpolation: get the chord tolerance right
 
For each G1 segment, sample with **step ≈ 0.25–0.5 × tool radius** so that any obstacle thicker than the step cannot slip between samples. For G2/G3 arcs, compute segments from a chord-error tolerance ε via `seg_angle = 2·acos(1 − ε/R_arc)`; `pygcode-norm --arc_linearize --arc_precision 0.005` already does this. Canned cycles (G73/G81–G89) should be expanded to G0/G1 before sampling — pygcode's `--canned_expand` handles the common ones. Honor modal state: `G17/G18/G19` (plane for arcs), `G20/G21` (units; normalize internally to mm), `G54–G59` (work offsets), `G43 Hn / G49` (tool-length compensation), `G90/G91` (absolute/incremental), `G53` (one-shot machine-frame override). Subprogram calls (M98/M99) and Fanuc macro vars (`#100…`) are *not* handled by pygcode — for those programs you must pre-process through LinuxCNC's `gcode` module or a controller emulator.
 
Point sampling is never provably tunnel-free: the honest claim is *"no collision detected at sampling step ε; obstacles thinner than ε may be missed."* If you need a guarantee, layer in `fcl.continuousCollide()` between consecutive sample transforms (~5–20× slower, provably tunnel-free).
 
---
 
## Minimal pipeline architecture
 
```
machine.json (WCS table, tool table, kinematic topology)
scene files (vise.step, stock.stl, machine_body.stl, fixture.stl)
program.nc
        │
        ▼
[1] Loader      trimesh.load() ; cascadio for STEP → triangulation
[2] Tool build  revolve(profile) for flute + shank + holder + spindle nose
[3] Scene       obstacles = CollisionManager(); add each static mesh once
                tool_grp  = CollisionManager(); add each tool part once
[4] Interpreter pygcode.Machine — yields (line_no, gcode_text, modal_state, segment)
[5] Sampler     chord-tol arc linearization; k·R linear step; optional conservative-advance
[6] Transforms  p_machine = WCS_offset + (X,Y,Z); Z += H (G43); build per-part 4×4
[7] Query       tool_grp.set_transform(part, T) ; obstacles.in_collision_other(...)
[8] Report      CollisionEvent(line_no, gcode, xyz_machine, xyz_wcs, wcs, pairs, depth)
```
 
```python
import numpy as np, trimesh
from pygcode import Machine, Line
from dataclasses import dataclass, field
 
@dataclass
class CollisionEvent:
    line_no: int
    gcode: str
    xyz_machine: list           # tool-tip in machine frame
    xyz_wcs: list               # commanded XYZ in active WCS
    wcs: str                    # 'G54' etc.
    pairs: list                 # [('flute', 'vise_jaw_left'), ...]
    penetration: float | None = None
 
def make_tool(diameter, flute_len, shank_d, holder_d, holder_h, kind='flat'):
    r, rs, rh = diameter/2, shank_d/2, holder_d/2
    if kind == 'flat':
        prof = [(0,0),(r,0),(r,flute_len),(rs,flute_len),(rs,flute_len*3)]
    elif kind == 'ball':
        t = np.linspace(-np.pi/2, 0, 16)
        hemi = [(r*np.cos(a), r + r*np.sin(a)) for a in t]
        prof = [(0,0)] + hemi + [(r,flute_len),(rs,flute_len),(rs,flute_len*3)]
    flute = trimesh.creation.revolve(np.array(prof), sections=48)
    holder = trimesh.creation.cylinder(radius=rh, height=holder_h, sections=48)
    holder.apply_translation([0, 0, flute_len*3 + holder_h/2])
    return {'flute': (flute, np.eye(4)), 'holder': (holder, np.eye(4))}
 
def build_scene(meshes):
    mgr = trimesh.collision.CollisionManager()
    for name, m in meshes.items():
        mgr.add_object(name, m)
    return mgr
 
def interpolate(prev_xyz, curr_xyz, tool_radius, step_k=0.4):
    L = np.linalg.norm(curr_xyz - prev_xyz)
    if L < 1e-9: return [curr_xyz]
    n = max(1, int(np.ceil(L / max(step_k*tool_radius, 0.05))))
    return [prev_xyz + (curr_xyz - prev_xyz)*(i/n) for i in range(1, n+1)]
 
def run_check(program_path, static_meshes, tool_parts, wcs_offsets,
              tool_radius=3.0, step_k=0.4):
    obstacles = build_scene(static_meshes)
    tool_grp = trimesh.collision.CollisionManager()
    for part, (mesh, _) in tool_parts.items():
        tool_grp.add_object(part, mesh)
 
    machine = Machine()
    events = []
    prev_pos = np.zeros(3)
    with open(program_path) as f:
        for i, raw in enumerate(f, 1):
            line = Line(raw)
            machine.process_block(line.block)
            curr_pos = np.array([machine.pos.X, machine.pos.Y, machine.pos.Z])
            wcs = machine.state.modal.coord_system or 'G54'
 
            # Optional fast skip: bbox/Z prefilter
            if curr_pos[2] > obstacles_zmax + 5.0 and prev_pos[2] > obstacles_zmax + 5.0:
                prev_pos = curr_pos; continue
 
            for sample in interpolate(prev_pos, curr_pos, tool_radius, step_k):
                p_machine = wcs_offsets[wcs] + sample
                T_tip = trimesh.transformations.translation_matrix(p_machine)
                for part, (_, T_local) in tool_parts.items():
                    tool_grp.set_transform(part, T_tip @ T_local)
 
                hit, pairs, data = obstacles.in_collision_other(
                    tool_grp, return_names=True, return_data=True)
                if hit:
                    depth = max((d.depth for d in data), default=None)
                    events.append(CollisionEvent(
                        line_no=i, gcode=raw.rstrip(),
                        xyz_machine=p_machine.tolist(),
                        xyz_wcs=sample.tolist(),
                        wcs=wcs, pairs=sorted(pairs), penetration=depth))
                    break  # one hit per sample is sufficient
            prev_pos = curr_pos
    return events
```
 
**Install set:** `pip install trimesh[easy] python-fcl pygcode cascadio numpy rtree`. Add `coacd` if you want convex decomposition of the holder/workpiece for higher performance, and `build123d` if you need parametric STEP. For maximum throughput, parallelize over block ranges with `multiprocessing.Pool` (each worker rebuilds its own manager — FCL collision objects are not picklable).
 
---
 
## Conclusion
 
The right pipeline today is an assemblage, not a download. `python-fcl` (via trimesh's `CollisionManager`) gives you the only mature pure-Python mesh-mesh collision primitive with named contact pairs and a broadphase tuned for "one moving object vs. many static." `pygcode` is the only Python parser that already does proper RS-274 modal state and arc linearization, and your largest semantic risks are not the collision engine but **(a)** including the holder/collet/spindle in the tool group, **(b)** correctly applying G54 offsets and G43 tool-length compensation when building per-sample transforms, and **(c)** sampling densely enough — under ~0.4 × tool radius — to avoid tunneling thin obstacles. If you later need provable tunnel-freeness, add `fcl.continuousCollide()`; if you need a clearance margin in the report ("near miss within 0.5 mm"), graduate the engine from `python-fcl` to `coal`. CAMotics and FreeCAD remain useful as visual sanity checks and for their tool-shape catalogs, but neither can be used as the actual collision checker — both projects openly state they don't check the shaft, holder, or fixtures, which is precisely the failure mode that crashes spindles in practice.