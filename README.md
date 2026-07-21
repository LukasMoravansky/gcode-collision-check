# gcode-collision-check

**Offline collision checker for CNC G-code.** Verifies the full tool assembly
(endmill + holder + collet) against your vise, stock, and fixtures —
*before* the program runs on the machine.

No open-source tool does this. CAMotics and FreeCAD Path simulate material
removal but don't check if your holder will crash into the vise jaw.
This one does.

## Quick start

```
pip install gcode-collision-check

gcode-collision-check verify program.nc --scene vise.stl
```

See [examples/](examples/) for a runnable crash/safe pair and the actual
CLI output. To see the result in an interactive 3D view instead of text,
use `gcode-collision-check visualize` (see below).

## What it checks

The tool path is sampled into thousands of XYZ positions along the program.
At each position, the *entire* tool assembly — cutter, shank, and holder —
is placed at that position and checked for overlap against the static scene
(vise, stock, fixtures, table). Most real-world crashes are the holder or
collet nut hitting a vise jaw, not the cutting edge itself — a check that
only looks at the cutter tip misses exactly those.

```
gcode-collision-check verify crash.nc --scene vise.stl
✗ COLLISION — 33 collision(s) found:
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-36.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 1.000 mm
  ...
```

## Supported G-code

Fanuc-style dialect (covers Fanuc, Haas, Mazak, and most generic
postprocessors):

- `G0`, `G1` — rapid / linear feed
- `G2`, `G3` — circular interpolation (I/J/K center format only, no R-format)
- `G17`/`G18`/`G19` — arc plane selection
- `G20`/`G21` — inch / mm (all internal units are mm)
- `G28` — home (simplified to a plain rapid to the given coordinates)
- `G40`/`G41`/`G42` — cutter compensation (parsed, not applied)
- `G43 Hn` / `G49` — tool length compensation
- `G54`–`G59` — work coordinate systems
- `G73`, `G80`–`G89` — canned cycles (expanded into rapid/feed moves;
  retract always targets the R-plane, G98/G99 not distinguished)
- `G90`/`G91` — absolute / incremental
- `M3`/`M4`/`M5` — spindle (parsed, ignored)
- `M6 Tn` — tool change
- `M30`/`M2` — program end
- `(...)` and `;...` comments, `N`-numbers, `%` program delimiters

Not supported: macro variables (`#100...`), subprograms (`M98`/`M99`,
`O`-calls), Siemens/Heidenhain conversational syntax, parametric
programming, 4th/5th axis (A/B/C).

## Tool configuration

Either build a tool from individual dimensions:

```
gcode-collision-check verify program.nc --scene vise.stl \
  --tool-diameter 10 --tool-flute-length 25 \
  --tool-shank-diameter 10 --tool-shank-length 30 \
  --tool-holder-diameter 46 --tool-holder-length 50 \
  --tool-kind flat
```

`--tool-kind` is `flat`, `ball`, or `bull` (`bull` requires
`--tool-corner-radius` > 0).

Or use a built-in preset, which overrides the individual `--tool-*` flags:

```
gcode-collision-check verify program.nc --scene vise.stl --tool-preset 6mm_ball
```

Available presets: `6mm_ball`, `10mm_flat`, `12mm_bull`.

Other options: `--wcs-offset X,Y,Z` (G54 offset from machine home),
`--output report.json` (write the full result as JSON), `--quiet`
(suppress the stdout summary).

## Visualizing a result

```
gcode-collision-check visualize program.nc --scene vise.stl
```

Runs the same check as `verify`, then opens an interactive 3D view in your
browser: the scene, the tool assembly placed at the first collision (or at
the end of the toolpath if it's safe), and the full toolpath traced in
yellow. The tool is colored red on collision, green when safe.

It takes the same `--tool-*`/`--tool-preset`/`--wcs-offset` options as
`verify`, plus:

- `--output-dir PATH` — save the GLB/HTML into `PATH` instead of a temp
  directory (useful if you want to keep or share the files)
- `--no-open` — generate the files without launching a browser

The view is a self-contained `scene.glb` + `scene.html` (using
`<model-viewer>` from a CDN) — open `scene.html` directly if the browser
didn't launch automatically. Safari blocks `file://` GLB loading via CORS;
use Chrome or Firefox if the view stays blank.

## Limitations

- Point sampling, not continuous collision detection (thin obstacles smaller
  than the sampling step may be missed)
- 3-axis only (no A/B/C rotary axes yet)
- Fanuc-style G-code only (no Siemens/Heidenhain conversational)
- Tool is modeled as cylinders (+ hemisphere/torus for ball/bull), not
  actual flute geometry
- No material removal — checks against the static stock shape, not what's
  actually left after each pass

## How it works

```
G-code text
  → parse (RS-274 tokenizer + modal state)        → GCodeSegment[]
  → sample (linear step / arc chord tolerance)     → (position, line_no)[]
  → Z-prefilter (skip segments above every obstacle)
  → collision query (tool CollisionManager vs. obstacle CollisionManager)
  → VerifyResult (safe / CollisionEvent[] with line, position, depth)
```

## Background

Built with AI-assisted coding (Claude Code) based on original research into
CNC collision detection across Python, C++, C#, NVIDIA Omniverse, and Unity
stacks. The architecture, library selection, and validation were done
against real machining programs at JIC Smart Factory. See
[docs/research/](docs/research/) for the full collision library landscape
analysis.

## License

MIT
