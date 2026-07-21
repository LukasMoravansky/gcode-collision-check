"""Main collision-check loop: parse G-code -> sample -> collision query -> events."""

from __future__ import annotations

import time

import numpy as np
import trimesh

from gcode_collision_check.collision.sampler import sample_segment
from gcode_collision_check.collision.scene import CollisionScene
from gcode_collision_check.parser.gcode_parser import parse_program
from gcode_collision_check.tool.assembly import build_tool_assembly
from gcode_collision_check.types import CollisionEvent, ToolConfig, VerifyResult


def verify(
    program_path: str,
    scene_stls: dict[str, str],
    tool_config: ToolConfig,
    wcs_offsets: dict[str, tuple[float, float, float]] | None = None,
    z_margin: float = 1.0,
) -> VerifyResult:
    """Check a G-code program for collisions between the tool assembly and a static scene.

    ``wcs_offsets`` maps a WCS name ("G54", ...) to its (x, y, z) offset from
    the machine origin; WCS not present default to (0, 0, 0). ``z_margin`` is
    the safety padding (mm) added to the scene's highest point for the
    Z-prefilter: segments that stay above that height on both ends are
    skipped entirely, since the tool tip (the assembly's lowest point) can
    never reach the obstacles.
    """
    result, _, _, _ = _run(program_path, scene_stls, tool_config, wcs_offsets, z_margin)
    return result


def verify_with_scene(
    program_path: str,
    scene_stls: dict[str, str],
    tool_config: ToolConfig,
    wcs_offsets: dict[str, tuple[float, float, float]] | None = None,
    z_margin: float = 1.0,
) -> tuple[VerifyResult, dict[str, trimesh.Trimesh], dict[str, trimesh.Trimesh], list[np.ndarray]]:
    """Like ``verify``, but also returns the loaded scene meshes, tool assembly
    parts, and the full toolpath (as machine-frame XYZ points) for visualization.
    """
    return _run(program_path, scene_stls, tool_config, wcs_offsets, z_margin)


def _run(
    program_path: str,
    scene_stls: dict[str, str],
    tool_config: ToolConfig,
    wcs_offsets: dict[str, tuple[float, float, float]] | None,
    z_margin: float,
) -> tuple[VerifyResult, dict[str, trimesh.Trimesh], dict[str, trimesh.Trimesh], list[np.ndarray]]:
    start_time = time.perf_counter()
    wcs_offsets = wcs_offsets or {}

    with open(program_path, encoding="utf-8") as f:
        program_text = f.read()
    segments = parse_program(program_text)

    tool_parts = build_tool_assembly(tool_config)
    tool_radius = tool_config.diameter / 2

    scene = CollisionScene()
    scene_meshes: dict[str, trimesh.Trimesh] = {}
    for name, path in scene_stls.items():
        mesh = trimesh.load(path, force="mesh")
        scene.add_obstacle(name, mesh)
        scene_meshes[name] = mesh
    scene.set_tool(tool_parts)

    z_max = scene.obstacles_z_max()

    events: list[CollisionEvent] = []
    toolpath_points: list[np.ndarray] = []
    total_samples = 0

    for segment in segments:
        if segment.start.z > z_max + z_margin and segment.end.z > z_max + z_margin:
            continue  # Z-prefilter: tool tip stays above every obstacle

        offset = wcs_offsets.get(segment.wcs, (0.0, 0.0, 0.0))

        for position, line_no in sample_segment(segment, tool_radius):
            total_samples += 1
            position_wcs = (position.x, position.y, position.z + segment.tlc_offset)
            machine_position = (
                position_wcs[0] + offset[0],
                position_wcs[1] + offset[1],
                position_wcs[2] + offset[2],
            )

            machine_position_arr = np.array(machine_position)
            toolpath_points.append(machine_position_arr)

            in_collision, pairs, contacts = scene.check_at_position(machine_position_arr)
            if in_collision:
                depth = max((c.depth for c in contacts), default=0.0)
                events.append(
                    CollisionEvent(
                        line_no=line_no,
                        gcode_text=segment.gcode_text,
                        position=machine_position,
                        position_wcs=position_wcs,
                        wcs=segment.wcs,
                        pairs=pairs,
                        penetration_depth=depth,
                    )
                )

    elapsed = time.perf_counter() - start_time
    result = VerifyResult(
        safe=not events,
        events=events,
        total_samples=total_samples,
        total_segments=len(segments),
        elapsed_seconds=elapsed,
    )
    return result, scene_meshes, tool_parts, toolpath_points
