# Try it now

```
pip install -e .
gcode-collision-check verify examples/crash.nc --scene examples/vise.stl
```

Actual output (33 colliding samples -- the tool crashes sideways into the
jaw on the line-6 rapid, then keeps dragging through it on the retract
since X never moves back out of the jaw):

<details>
<summary>Full output</summary>

```
✗ COLLISION — 33 collision(s) found:
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-36.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 1.000 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-38.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 15.036 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-40.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 15.000 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-42.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 15.042 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-44.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 40.087 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-46.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 6.466 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-48.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 5.488 mm
  Line 6: G0 X-50          (rapid into the vise jaw = CRASH)
    Position: X=-50.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 7.444 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=5.00
    Pair: flute ↔ vise
    Depth: 7.444 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=3.12
    Pair: flute ↔ vise
    Depth: 9.798 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=1.25
    Pair: flute ↔ vise
    Depth: 6.250 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=-0.62
    Pair: flute ↔ vise
    Depth: 4.375 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=-2.50
    Pair: flute ↔ vise
    Depth: 2.902 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=-4.38
    Pair: flute ↔ vise
    Depth: 0.625 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=-6.25
    Pair: flute ↔ vise
    Depth: 1.250 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=-8.12
    Pair: flute ↔ vise
    Depth: 3.455 mm
  Line 7: G1 Z-10 F200
    Position: X=-50.00 Y=0.00 Z=-10.00
    Pair: flute ↔ vise
    Depth: 5.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=-10.00
    Pair: flute ↔ vise
    Depth: 5.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=-8.00
    Pair: flute ↔ vise
    Depth: 3.342 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=-6.00
    Pair: flute ↔ vise
    Depth: 1.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=-4.00
    Pair: flute ↔ vise
    Depth: 1.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=-2.00
    Pair: flute ↔ vise
    Depth: 3.342 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=0.00
    Pair: flute ↔ vise
    Depth: 5.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=2.00
    Pair: flute ↔ vise
    Depth: 7.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=4.00
    Pair: flute ↔ vise
    Depth: 10.535 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=6.00
    Pair: flute ↔ vise
    Depth: 7.444 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=8.00
    Pair: flute ↔ vise
    Depth: 7.449 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=10.00
    Pair: flute ↔ vise
    Depth: 7.752 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=12.00
    Pair: flute ↔ vise
    Depth: 8.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=14.00
    Pair: flute ↔ vise
    Depth: 6.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=16.00
    Pair: flute ↔ vise
    Depth: 4.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=18.00
    Pair: flute ↔ vise
    Depth: 2.000 mm
  Line 8: G0 Z50
    Position: X=-50.00 Y=0.00 Z=20.00
    Pair: flute ↔ vise
    Depth: 0.000 mm
```

</details>

Exit code: `1`.

Now run the same tool assembly against a program that never leaves the jaw
opening:

```
gcode-collision-check verify examples/safe.nc --scene examples/vise.stl
```

Actual output:

```
✓ SAFE — no collisions detected
```

Exit code: `0`.

> On Windows consoles with a non-UTF-8 code page (e.g. `cp1250`), the ✓/✗/↔
> characters above may raise a `UnicodeEncodeError`. Work around it with
> `set PYTHONIOENCODING=utf-8` (cmd) or `$env:PYTHONIOENCODING="utf-8"`
> (PowerShell) before running the command.

## Other CLI options

```
gcode-collision-check verify examples/crash.nc \
  --scene examples/vise.stl \
  --tool-preset 6mm_ball \
  --wcs-offset 10,0,0 \
  --output report.json \
  --quiet
```

- `--tool-preset` picks a built-in tool profile (`6mm_ball`, `10mm_flat`,
  `12mm_bull`) and overrides the individual `--tool-*` flags.
- `--wcs-offset X,Y,Z` sets the G54 offset (machine-frame origin of the
  active work coordinate system).
- `--output PATH` writes the full `VerifyResult` as JSON to `PATH`.
- `--quiet` suppresses the summary printed to stdout (still sets the exit
  code and still writes `--output`, if given).

## What's in here

- **`vise.stl`** -- a generic parametric vise (two jaws + base), generated by
  `generate_vise.py`. Jaws are 70mm wide, 50mm tall, 15mm thick, with a 60mm
  opening between them. The jaws stand 20mm above the machining origin
  (`Z=0`) and 30mm below it, so a straight-down plunge at safe/approach
  height can still clear the top of the jaw if it's within the opening.
- **`crash.nc`** -- cuts safely at the machining origin (inside the opening),
  then rapids sideways into the jaw and crashes.
- **`safe.nc`** -- the same program with the crashing move removed.

Regenerate the vise mesh with:

```
python examples/generate_vise.py
```
