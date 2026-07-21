"""Main collision-check loop: parse G-code -> sample -> collision query -> events."""

from __future__ import annotations

import math
import time

import numpy as np
import trimesh

from gcode_collision_check.collision.sampler import sample_segment
from gcode_collision_check.collision.scene import CollisionScene
from gcode_collision_check.parser.gcode_parser import parse_program
from gcode_collision_check.tool.assembly import build_tool_assembly
from gcode_collision_check.types import CollisionEvent, ToolConfig, VerifyResult


def _rotation_matrix(program_rotation: tuple[float, float, float] | None) -> np.ndarray:
    """Build the extrinsic machine-frame rotation matrix R = Rz(C) @ Ry(B) @ Rx(A).

    ``program_rotation`` is (A, B, C) in degrees per CSN ISO 841: A/B/C rotate
    around X/Y/Z, positive = right-hand rule around the positive half-axis.
    Returns the 3x3 identity when ``program_rotation`` is None.
    """
    if program_rotation is None:
        return np.eye(3)
    a, b, c = (math.radians(deg) for deg in program_rotation)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return rz @ ry @ rx


def verify(
    program_path: str,
    scene_stls: dict[str, str],
    tool_config: ToolConfig,
    wcs_offsets: dict[str, tuple[float, float, float]] | None = None,
    z_margin: float = 1.0,
    program_origin: tuple[float, float, float] | None = None,
    program_rotation: tuple[float, float, float] | None = None,
) -> VerifyResult:
    """Check a G-code program for collisions between the tool assembly and a static scene.

    ``wcs_offsets`` maps a WCS name ("G54", ...) to its (x, y, z) offset from
    the machine origin; WCS not present default to (0, 0, 0). ``z_margin`` is
    the safety padding (mm) added to the scene's highest point for the
    Z-prefilter: segments that stay above that height on both ends are
    skipped entirely, since the tool tip (the assembly's lowest point) can
    never reach the obstacles. ``program_origin``, if given, is the machine-frame
    position of the program's (0, 0, 0) and is applied to every sample regardless
    of the segment's active WCS, overriding ``wcs_offsets`` entirely. ``program_rotation``,
    if given, is (A, B, C) in degrees (CSN ISO 841) and rotates the whole program
    around its own origin -- i.e. around whichever translation (``program_origin``
    or the active WCS offset) applies -- before that translation is added.
    """
    result, _, _, _ = _run(
        program_path, scene_stls, tool_config, wcs_offsets, z_margin, program_origin, program_rotation
    )
    return result


def verify_with_scene(
    program_path: str,
    scene_stls: dict[str, str],
    tool_config: ToolConfig,
    wcs_offsets: dict[str, tuple[float, float, float]] | None = None,
    z_margin: float = 1.0,
    program_origin: tuple[float, float, float] | None = None,
    program_rotation: tuple[float, float, float] | None = None,
) -> tuple[VerifyResult, dict[str, trimesh.Trimesh], dict[str, trimesh.Trimesh], list[np.ndarray]]:
    """Like ``verify``, but also returns the loaded scene meshes, tool assembly
    parts, and the full toolpath (as machine-frame XYZ points) for visualization.
    """
    return _run(
        program_path, scene_stls, tool_config, wcs_offsets, z_margin, program_origin, program_rotation
    )


def _run(
    program_path: str,
    scene_stls: dict[str, str],
    tool_config: ToolConfig,
    wcs_offsets: dict[str, tuple[float, float, float]] | None,
    z_margin: float,
    program_origin: tuple[float, float, float] | None = None,
    program_rotation: tuple[float, float, float] | None = None,
) -> tuple[VerifyResult, dict[str, trimesh.Trimesh], dict[str, trimesh.Trimesh], list[np.ndarray]]:
    start_time = time.perf_counter()
    wcs_offsets = wcs_offsets or {}
    rotation = _rotation_matrix(program_rotation)

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
        translation = np.array(
            program_origin if program_origin is not None else wcs_offsets.get(segment.wcs, (0.0, 0.0, 0.0))
        )

        # Z-prefilter must use the machine-frame Z of both endpoints, not the raw
        # program Z: a rotation (or a large Z translation) can bring a segment
        # that looks high in program coordinates down into the scene.
        machine_start_z = (rotation @ np.array([segment.start.x, segment.start.y, segment.start.z]) + translation)[2]
        machine_end_z = (rotation @ np.array([segment.end.x, segment.end.y, segment.end.z]) + translation)[2]
        if machine_start_z > z_max + z_margin and machine_end_z > z_max + z_margin:
            continue  # tool tip stays above every obstacle

        for position, line_no in sample_segment(segment, tool_radius):
            total_samples += 1
            position_wcs = (position.x, position.y, position.z + segment.tlc_offset)
            machine_position_arr = rotation @ np.array(position_wcs) + translation
            machine_position = tuple(machine_position_arr)

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
