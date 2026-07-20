"""Tool proxy geometry and G-code-path linear sampling."""

from __future__ import annotations

import numpy as np
import trimesh

from poc.collision_poc.types import PathSample
from poc.swept_volume.types import ToolConfig, ToolMove


def build_tool_proxy_mesh(tool: ToolConfig) -> trimesh.Trimesh:
    """Simplified tool stand-in: a single cylinder, tip at local origin.

    This is *not* the full tool assembly (holder/collet/spindle) required by
    the target architecture -- just the cutter, approximated as a cylinder of
    ``tool.radius`` extending up (+Z) by ``tool.flute_length``. Good enough to
    detect a cutter-vs-obstacle collision; it will not catch a holder/collet
    strike.
    """
    mesh = trimesh.creation.cylinder(radius=tool.radius, height=tool.flute_length)
    mesh.apply_translation([0.0, 0.0, tool.flute_length / 2.0])
    return mesh


def sample_tool_path(moves: list[ToolMove], max_step: float) -> list[PathSample]:
    """Linearly interpolate between consecutive ``ToolMove`` waypoints.

    Each sample is tagged with the destination move's ``line_no``/``motion``/
    ``feed`` (the move that produced it). Arcs (G2/G3) are already linearized
    into short ``ToolMove`` segments by the MPF parser, so no special-casing
    is needed here beyond segment-to-segment interpolation.
    """
    if not moves:
        return []
    if max_step <= 0:
        raise ValueError("max_step must be positive")

    samples = [
        PathSample(
            line_no=moves[0].line_no,
            gcode=moves[0].motion,
            x=moves[0].x,
            y=moves[0].y,
            z=moves[0].z,
            feed=moves[0].feed,
        )
    ]

    prev = moves[0]
    for move in moves[1:]:
        start = np.array([prev.x, prev.y, prev.z])
        end = np.array([move.x, move.y, move.z])
        length = float(np.linalg.norm(end - start))
        n_steps = max(1, int(np.ceil(length / max_step)))
        for i in range(1, n_steps + 1):
            t = i / n_steps
            point = start + (end - start) * t
            samples.append(
                PathSample(
                    line_no=move.line_no,
                    gcode=move.motion,
                    x=float(point[0]),
                    y=float(point[1]),
                    z=float(point[2]),
                    feed=move.feed,
                )
            )
        prev = move

    return samples
