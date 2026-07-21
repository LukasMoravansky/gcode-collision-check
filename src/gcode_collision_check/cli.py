import json
import sys
from pathlib import Path

import click
import numpy as np

from gcode_collision_check import visualize
from gcode_collision_check.collision import checker
from gcode_collision_check.tool.profiles import get_profile
from gcode_collision_check.types import ToolConfig


def _build_tool_config(
    tool_preset,
    tool_diameter,
    tool_flute_length,
    tool_shank_diameter,
    tool_shank_length,
    tool_holder_diameter,
    tool_holder_length,
    tool_kind,
    tool_corner_radius,
):
    if tool_preset is not None:
        return get_profile(tool_preset)
    return ToolConfig(
        diameter=tool_diameter,
        flute_length=tool_flute_length,
        shank_diameter=tool_shank_diameter,
        shank_length=tool_shank_length,
        holder_diameter=tool_holder_diameter,
        holder_length=tool_holder_length,
        kind=tool_kind,
        corner_radius=tool_corner_radius,
    )


def _parse_triple(value, option_name, labels="X,Y,Z"):
    parts = value.split(",")
    if len(parts) != 3:
        raise click.BadParameter(f"expected {labels} (got '{value}')", param_hint=option_name)
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        raise click.BadParameter(f"expected three numbers {labels} (got '{value}')", param_hint=option_name)


def _parse_xyz(value, option_name):
    return _parse_triple(value, option_name, labels="X,Y,Z")


def _parse_abc(value, option_name):
    """Parse an A,B,C rotation in degrees (CSN ISO 841: A/B/C around X/Y/Z)."""
    return _parse_triple(value, option_name, labels="A,B,C")


def _resolve_origin(origin, wcs_offset):
    """Parse --origin/--wcs-offset into (wcs_offsets, program_origin) for the checker.

    The two are mutually exclusive: --origin applies one machine-frame datum to
    every sample regardless of active WCS, overriding per-WCS offsets entirely.
    """
    if origin is not None and wcs_offset is not None:
        raise click.BadParameter(
            "--origin and --wcs-offset are mutually exclusive; use one or the other."
        )
    if origin is not None:
        return None, _parse_xyz(origin, "--origin")
    offset = _parse_xyz(wcs_offset if wcs_offset is not None else "0,0,0", "--wcs-offset")
    return {"G54": offset}, None


def _print_summary(result):
    stats = (
        f"{result.total_segments} segments, {result.total_samples} samples, "
        f"{result.elapsed_seconds:.2f} s"
    )

    if result.safe:
        click.echo("RESULT: SAFE   no collisions detected")
        click.echo(f"        checked {stats}")
        return

    # Each collision sample is a separate event, so one G-code move produces many
    # near-identical events. Group them by line so the report stays readable:
    # one row per line, aggregating pairs, hit count, depth range, and Z.
    groups: dict[int, dict] = {}
    order: list[int] = []
    for e in result.events:
        g = groups.get(e.line_no)
        if g is None:
            g = {
                "gcode": e.gcode_text.strip(),
                "pairs": set(),
                "hits": 0,
                "depth_max": 0.0,
                "z_values": [],
            }
            groups[e.line_no] = g
            order.append(e.line_no)
        g["hits"] += 1
        g["depth_max"] = max(g["depth_max"], e.penetration_depth)
        g["z_values"].append(e.position[2])
        for a, b in e.pairs:
            g["pairs"].add(f"{a} / {b}")

    rows = []
    for line_no in order:
        g = groups[line_no]
        z_vals = g["z_values"]
        z_lo, z_hi = min(z_vals), max(z_vals)
        z_txt = f"{z_lo:.2f}" if abs(z_hi - z_lo) < 5e-3 else f"{z_lo:.2f}..{z_hi:.2f}"
        depth_txt = "contact" if g["depth_max"] < 5e-4 else f"{g['depth_max']:.3f}"
        rows.append(
            {
                "line": str(line_no),
                "gcode": g["gcode"],
                "pair": ", ".join(sorted(g["pairs"])),
                "hits": str(g["hits"]),
                "depth": depth_txt,
                "z": z_txt,
            }
        )

    n_lines = len(rows)
    click.echo(
        f"RESULT: COLLISION   {len(result.events)} hits on "
        f"{n_lines} of {result.total_segments} program lines"
    )
    click.echo(f"        checked {stats}")
    click.echo("")

    headers = {
        "line": "LINE",
        "gcode": "G-CODE",
        "pair": "COLLIDING PAIR",
        "hits": "HITS",
        "depth": "DEPTH mm",
        "z": "Z",
    }
    cols = ["line", "gcode", "pair", "hits", "depth", "z"]
    # Cap the G-code column so a long move line can't wreck the layout.
    for r in rows:
        if len(r["gcode"]) > 32:
            r["gcode"] = r["gcode"][:29] + "..."
    widths = {c: max(len(headers[c]), *(len(r[c]) for r in rows)) for c in cols}

    def _fmt(cells):
        return "  " + "  ".join(cells[c].ljust(widths[c]) for c in cols).rstrip()

    click.echo(_fmt(headers))
    click.echo("  " + "  ".join("-" * widths[c] for c in cols))
    for r in rows:
        click.echo(_fmt(r))

    first = result.events[0]
    click.echo("")
    click.echo(
        f"        first contact: line {first.line_no}  "
        f"X{first.position[0]:.2f} Y{first.position[1]:.2f} Z{first.position[2]:.2f}"
    )


@click.group()
def main():
    """gcode-collision-check CLI."""


@main.command()
@click.argument("gcode_path", type=click.Path(exists=True, dir_okay=False))
@click.option("--scene", multiple=True, help="STL file(s) for static obstacles")
@click.option("--tool-diameter", default=10.0, type=float)
@click.option("--tool-flute-length", default=25.0, type=float)
@click.option("--tool-shank-diameter", default=10.0, type=float)
@click.option("--tool-shank-length", default=30.0, type=float)
@click.option("--tool-holder-diameter", default=46.0, type=float)
@click.option("--tool-holder-length", default=50.0, type=float)
@click.option("--tool-kind", default="flat", type=click.Choice(["flat", "ball", "bull"]))
@click.option("--tool-corner-radius", default=0.0, type=float, help="Bull-nose corner radius, mm (only used with --tool-kind bull)")
@click.option("--tool-preset", default=None, help="Named preset: 6mm_ball, 10mm_flat...")
@click.option("--wcs-offset", default=None, help="G54 offset X,Y,Z in mm (default 0,0,0)")
@click.option(
    "--origin",
    default=None,
    help="Program zero / part datum X,Y,Z in mm — applies globally, overrides per-WCS offsets",
)
@click.option(
    "--origin-rotation",
    default=None,
    help=(
        "Rotate the whole program A,B,C in degrees around the program origin "
        "(CSN ISO 841: A/B/C around X/Y/Z, right-hand rule)"
    ),
)
@click.option("--output", default=None, help="JSON report output path")
@click.option("--quiet", is_flag=True)
def verify(
    gcode_path,
    scene,
    tool_diameter,
    tool_flute_length,
    tool_shank_diameter,
    tool_shank_length,
    tool_holder_diameter,
    tool_holder_length,
    tool_kind,
    tool_corner_radius,
    tool_preset,
    wcs_offset,
    origin,
    origin_rotation,
    output,
    quiet,
):
    """Check GCODE_PATH for collisions against one or more --scene STL files."""
    tool_config = _build_tool_config(
        tool_preset,
        tool_diameter,
        tool_flute_length,
        tool_shank_diameter,
        tool_shank_length,
        tool_holder_diameter,
        tool_holder_length,
        tool_kind,
        tool_corner_radius,
    )

    scene_stls = {Path(p).stem: p for p in scene}
    wcs_offsets, program_origin = _resolve_origin(origin, wcs_offset)
    program_rotation = _parse_abc(origin_rotation, "--origin-rotation") if origin_rotation is not None else None

    result = checker.verify(
        gcode_path,
        scene_stls,
        tool_config,
        wcs_offsets=wcs_offsets,
        program_origin=program_origin,
        program_rotation=program_rotation,
    )

    if output is not None:
        Path(output).write_text(json.dumps(result.to_dict(), indent=2))

    if not quiet:
        _print_summary(result)

    sys.exit(0 if result.safe else 1)


@main.command()
@click.argument("gcode_path", type=click.Path(exists=True, dir_okay=False))
@click.option("--scene", multiple=True, help="STL file(s) for static obstacles")
@click.option("--tool-diameter", default=10.0, type=float)
@click.option("--tool-flute-length", default=25.0, type=float)
@click.option("--tool-shank-diameter", default=10.0, type=float)
@click.option("--tool-shank-length", default=30.0, type=float)
@click.option("--tool-holder-diameter", default=46.0, type=float)
@click.option("--tool-holder-length", default=50.0, type=float)
@click.option("--tool-kind", default="flat", type=click.Choice(["flat", "ball", "bull"]))
@click.option("--tool-corner-radius", default=0.0, type=float, help="Bull-nose corner radius, mm (only used with --tool-kind bull)")
@click.option("--tool-preset", default=None, help="Named preset: 6mm_ball, 10mm_flat...")
@click.option("--wcs-offset", default=None, help="G54 offset X,Y,Z in mm (default 0,0,0)")
@click.option(
    "--origin",
    default=None,
    help="Program zero / part datum X,Y,Z in mm — applies globally, overrides per-WCS offsets",
)
@click.option(
    "--origin-rotation",
    default=None,
    help=(
        "Rotate the whole program A,B,C in degrees around the program origin "
        "(CSN ISO 841: A/B/C around X/Y/Z, right-hand rule)"
    ),
)
@click.option("--output-dir", default=None, type=click.Path(file_okay=False), help="Directory to save GLB/HTML into (default: temp dir)")
@click.option("--no-open", is_flag=True, help="Generate the files but do not open a browser")
def visualize_cmd(
    gcode_path,
    scene,
    tool_diameter,
    tool_flute_length,
    tool_shank_diameter,
    tool_shank_length,
    tool_holder_diameter,
    tool_holder_length,
    tool_kind,
    tool_corner_radius,
    tool_preset,
    wcs_offset,
    origin,
    origin_rotation,
    output_dir,
    no_open,
):
    """Check GCODE_PATH for collisions and open an interactive 3D view of the result."""
    tool_config = _build_tool_config(
        tool_preset,
        tool_diameter,
        tool_flute_length,
        tool_shank_diameter,
        tool_shank_length,
        tool_holder_diameter,
        tool_holder_length,
        tool_kind,
        tool_corner_radius,
    )

    scene_stls = {Path(p).stem: p for p in scene}
    wcs_offsets, program_origin = _resolve_origin(origin, wcs_offset)
    program_rotation = _parse_abc(origin_rotation, "--origin-rotation") if origin_rotation is not None else None

    result, scene_meshes, tool_parts, toolpath_points = checker.verify_with_scene(
        gcode_path,
        scene_stls,
        tool_config,
        wcs_offsets=wcs_offsets,
        program_origin=program_origin,
        program_rotation=program_rotation,
    )

    if not result.safe:
        tool_position = np.array(result.events[0].position)
    elif toolpath_points:
        tool_position = np.array(toolpath_points[-1])
    else:
        tool_position = np.array([0.0, 0.0, 0.0])

    viz_scene = visualize.build_scene(
        scene_meshes, tool_parts, tool_position, toolpath_points, is_collision=not result.safe
    )
    saved_dir = visualize.open_in_browser(
        viz_scene, output_dir=output_dir, open_browser=not no_open
    )

    click.echo(f"Opened 3D view in browser. Files: {saved_dir}")
    _print_summary(result)

    sys.exit(0 if result.safe else 1)


main.add_command(visualize_cmd, name="visualize")


if __name__ == "__main__":
    main()
