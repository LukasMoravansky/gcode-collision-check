"""Arc linearization (G2/G3) and canned-cycle expansion (G73, G81-G89)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from gcode_collision_check.parser.modal_state import Vector3

_PLANE_AXES = {
    "G17": ("x", "y", "z"),  # XY plane, Z is the helical axis
    "G18": ("x", "z", "y"),  # XZ plane, Y is the helical axis
    "G19": ("y", "z", "x"),  # YZ plane, X is the helical axis
}

_DRILL_LIKE_CYCLES = {"G84", "G85", "G86", "G87", "G88", "G89"}
_CHIP_BREAK_CLEARANCE = 1.0  # mm; G73's partial retract above each peck

_EPSILON = 1e-9


@dataclass
class GCodeSegment:
    """One motion command, already resolved to absolute machine-frame coordinates."""

    line_no: int
    gcode_text: str
    motion_mode: str  # G0/G1/G2/G3/G73/G81-G89
    start: Vector3
    end: Vector3
    plane: str = "G17"  # G17/G18/G19, active arc plane
    arc_center: Vector3 | None = None  # absolute center, for G2/G3
    feed: float | None = None
    r_plane: float | None = None  # R word: canned-cycle rapid/retract plane
    z_depth: float | None = None  # Z word: canned-cycle final depth
    q_peck: float | None = None  # Q word: G73/G83 peck increment
    wcs: str = "G54"  # active work coordinate system when this segment ran
    tlc_offset: float = 0.0  # active tool-length compensation (mm) when this segment ran


def linearize_arc(segment: GCodeSegment, chord_tolerance: float = 0.01) -> list[Vector3]:
    """Linearize a G2/G3 arc into points, inclusive of both start and end.

    The max angular step is ``2 * acos(1 - chord_tolerance / radius)`` so the
    chord never deviates from the true arc by more than ``chord_tolerance``.
    Helical motion (a changing third axis) is linearly interpolated along the
    same angular parameter as the in-plane sweep.
    """
    if segment.arc_center is None:
        raise ValueError("linearize_arc requires an arc_center")

    axis_a, axis_b, axis_c = _PLANE_AXES[segment.plane]
    center, start, end = segment.arc_center, segment.start, segment.end

    ca, cb = getattr(center, axis_a), getattr(center, axis_b)
    sa, sb = getattr(start, axis_a) - ca, getattr(start, axis_b) - cb
    ea, eb = getattr(end, axis_a) - ca, getattr(end, axis_b) - cb

    radius = math.hypot(sa, sb)
    start_angle = math.atan2(sb, sa)
    end_angle = math.atan2(eb, ea)
    clockwise = segment.motion_mode == "G2"

    if clockwise:
        sweep = (start_angle - end_angle) % (2 * math.pi)
    else:
        sweep = (end_angle - start_angle) % (2 * math.pi)
    if sweep < _EPSILON:
        sweep = 2 * math.pi  # coincident start/end: full circle

    if radius > _EPSILON:
        ratio = max(-1.0, min(1.0, 1 - chord_tolerance / radius))
        seg_angle = 2 * math.acos(ratio)
    else:
        seg_angle = sweep
    if seg_angle < _EPSILON:
        seg_angle = sweep

    n_segments = max(1, math.ceil(sweep / seg_angle))
    direction = -1.0 if clockwise else 1.0

    start_c, end_c = getattr(start, axis_c), getattr(end, axis_c)

    points = []
    for i in range(n_segments + 1):
        t = i / n_segments
        angle = start_angle + direction * sweep * t
        point = Vector3()
        setattr(point, axis_a, ca + radius * math.cos(angle))
        setattr(point, axis_b, cb + radius * math.sin(angle))
        setattr(point, axis_c, start_c + (end_c - start_c) * t)
        points.append(point)

    return points


def expand_canned_cycle(segment: GCodeSegment) -> list[GCodeSegment]:
    """Expand a canned-cycle segment into rapid/feed moves.

    Non-motion aspects (dwell time in G82, spindle reversal in G84, boring
    dwell in G86/G89, ...) are ignored -- only the resulting tool-tip motion
    matters for collision checking.
    """
    mode = segment.motion_mode
    if mode in ("G81", "G82") or mode in _DRILL_LIKE_CYCLES:
        return _expand_simple_drill(segment)
    if mode == "G83":
        return _expand_peck_drill(segment, full_retract=True)
    if mode == "G73":
        return _expand_peck_drill(segment, full_retract=False)
    raise ValueError(f"unsupported canned cycle: {mode}")


def _at_z(segment: GCodeSegment, z: float) -> Vector3:
    return Vector3(segment.start.x, segment.start.y, z)


def _expand_simple_drill(segment: GCodeSegment) -> list[GCodeSegment]:
    r_plane, z_depth = segment.r_plane, segment.z_depth
    at_r = _at_z(segment, r_plane)
    at_depth = _at_z(segment, z_depth)
    return [
        GCodeSegment(segment.line_no, segment.gcode_text, "G0", segment.start, at_r, plane=segment.plane),
        GCodeSegment(
            segment.line_no, segment.gcode_text, "G1", at_r, at_depth, plane=segment.plane, feed=segment.feed
        ),
        GCodeSegment(segment.line_no, segment.gcode_text, "G0", at_depth, at_r, plane=segment.plane),
    ]


def _expand_peck_drill(segment: GCodeSegment, full_retract: bool) -> list[GCodeSegment]:
    r_plane, z_depth, q_peck = segment.r_plane, segment.z_depth, segment.q_peck
    if not q_peck or q_peck <= 0:
        return _expand_simple_drill(segment)

    depths = []
    current = r_plane
    while current - z_depth > _EPSILON:
        current = max(z_depth, current - q_peck)
        depths.append(current)

    segments = [
        GCodeSegment(segment.line_no, segment.gcode_text, "G0", segment.start, _at_z(segment, r_plane), plane=segment.plane)
    ]
    position_z = r_plane
    for depth in depths:
        feed_start = _at_z(segment, position_z)
        feed_end = _at_z(segment, depth)
        segments.append(
            GCodeSegment(
                segment.line_no, segment.gcode_text, "G1", feed_start, feed_end, plane=segment.plane, feed=segment.feed
            )
        )
        is_last = depth == depths[-1]
        retract_to = r_plane if (full_retract or is_last) else min(r_plane, depth + _CHIP_BREAK_CLEARANCE)
        segments.append(
            GCodeSegment(segment.line_no, segment.gcode_text, "G0", feed_end, _at_z(segment, retract_to), plane=segment.plane)
        )
        position_z = retract_to

    return segments
