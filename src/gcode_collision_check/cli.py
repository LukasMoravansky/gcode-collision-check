import json
import sys
from pathlib import Path

import click

from gcode_collision_check.collision import checker
from gcode_collision_check.tool.profiles import get_profile
from gcode_collision_check.types import ToolConfig


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
@click.option("--tool-preset", default=None, help="Named preset: 6mm_ball, 10mm_flat...")
@click.option("--wcs-offset", default="0,0,0", help="G54 offset X,Y,Z in mm")
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
    tool_preset,
    wcs_offset,
    output,
    quiet,
):
    """Check GCODE_PATH for collisions against one or more --scene STL files."""
    if tool_preset is not None:
        tool_config = get_profile(tool_preset)
    else:
        tool_config = ToolConfig(
            diameter=tool_diameter,
            flute_length=tool_flute_length,
            shank_diameter=tool_shank_diameter,
            shank_length=tool_shank_length,
            holder_diameter=tool_holder_diameter,
            holder_length=tool_holder_length,
            kind=tool_kind,
        )

    scene_stls = {Path(p).stem: p for p in scene}
    offset_x, offset_y, offset_z = (float(v) for v in wcs_offset.split(","))
    wcs_offsets = {"G54": (offset_x, offset_y, offset_z)}

    result = checker.verify(gcode_path, scene_stls, tool_config, wcs_offsets=wcs_offsets)

    if output is not None:
        Path(output).write_text(json.dumps(result.to_dict(), indent=2))

    if not quiet:
        if result.safe:
            click.echo("✓ SAFE — no collisions detected")
        else:
            click.echo(f"✗ COLLISION — {len(result.events)} collision(s) found:")
            for e in result.events:
                click.echo(f"  Line {e.line_no}: {e.gcode_text.strip()}")
                click.echo(
                    f"    Position: X={e.position[0]:.2f} Y={e.position[1]:.2f} Z={e.position[2]:.2f}"
                )
                click.echo(f"    Pair: {e.pairs[0][0]} ↔ {e.pairs[0][1]}")
                click.echo(f"    Depth: {e.penetration_depth:.3f} mm")

    sys.exit(0 if result.safe else 1)


if __name__ == "__main__":
    main()
