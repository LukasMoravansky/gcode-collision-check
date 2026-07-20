"""Samples a GCodeSegment into discrete points for collision checking."""

from __future__ import annotations

import math

from gcode_collision_check.parser.arc_linearizer import GCodeSegment, linearize_arc
from gcode_collision_check.parser.modal_state import Vector3

_EPSILON = 1e-9


def _lerp(start: Vector3, end: Vector3, t: float) -> Vector3:
    return Vector3(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
        start.z + (end.z - start.z) * t,
    )


def sample_segment(
    segment: GCodeSegment,
    tool_radius: float,
    step_factor: float = 0.4,
    chord_tol: float = 0.01,
) -> list[tuple[Vector3, int]]:
    """Sample a segment into (position, line_no) pairs, inclusive of both ends.

    Linear moves (G0/G1) are sampled at ``step_factor * tool_radius`` spacing.
    Arcs (G2/G3) are sampled via ``linearize_arc`` at ``chord_tol`` tolerance.
    """
    if segment.motion_mode in ("G2", "G3"):
        points = linearize_arc(segment, chord_tolerance=chord_tol)
    else:
        start, end = segment.start, segment.end
        length = math.dist((start.x, start.y, start.z), (end.x, end.y, end.z))
        if length < _EPSILON:
            points = [start]
        else:
            step = step_factor * tool_radius
            n_segments = max(1, math.ceil(length / step))
            points = [_lerp(start, end, i / n_segments) for i in range(n_segments + 1)]

    return [(point, segment.line_no) for point in points]
