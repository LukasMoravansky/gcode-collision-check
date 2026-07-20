import math

from gcode_collision_check.parser.arc_linearizer import (
    GCodeSegment,
    expand_canned_cycle,
    linearize_arc,
)
from gcode_collision_check.parser.modal_state import GCodeWord, ModalState, Vector3


def W(letter: str, value: float) -> GCodeWord:
    return GCodeWord(letter, value)


def test_default_state():
    state = ModalState()
    assert state.distance_mode == "G90"
    assert state.coord_system == "G54"
    assert state.plane == "G17"
    assert state.units == "G21"
    assert state.tlc_offset == 0.0
    assert (state.position.x, state.position.y, state.position.z) == (0.0, 0.0, 0.0)


def test_g90_to_g91_switch():
    state = ModalState()
    state.update([W("G", 91)])
    assert state.distance_mode == "G91"
    state.update([W("G", 90)])
    assert state.distance_mode == "G90"


def test_g54_to_g55_switch():
    state = ModalState()
    assert state.coord_system == "G54"
    state.update([W("G", 55)])
    assert state.coord_system == "G55"


def test_plane_switch():
    state = ModalState()
    state.update([W("G", 18)])
    assert state.plane == "G18"
    state.update([W("G", 19)])
    assert state.plane == "G19"


def test_g0_absolute_move_updates_position():
    state = ModalState()
    state.update([W("G", 0), W("X", 10), W("Y", 20)])
    assert (state.position.x, state.position.y, state.position.z) == (10.0, 20.0, 0.0)
    assert state.motion_mode == "G0"


def test_g91_incremental_move():
    state = ModalState()
    state.update([W("X", 10)])  # absolute, position.x = 10
    state.update([W("G", 91), W("G", 0), W("X", 5)])
    assert state.position.x == 15.0
    assert state.distance_mode == "G91"


def test_absolute_move_after_incremental_overrides_position():
    state = ModalState()
    state.update([W("G", 91), W("X", 5)])
    state.update([W("G", 90), W("X", 5)])
    assert state.position.x == 5.0


def test_units_g21_default_and_g20_inch_stored_as_mm():
    state = ModalState()
    assert state.units == "G21"
    state.update([W("G", 20)])
    assert state.units == "G20"
    state.update([W("X", 1)])
    assert state.position.x == 25.4  # 1 inch stored internally as mm

    state.update([W("G", 21)])
    state.update([W("X", 10)])
    assert state.position.x == 10.0  # back to mm, no conversion


def test_g43_h1_with_tool_table_sets_tlc_offset():
    state = ModalState()
    state.update([W("G", 43), W("H", 1)], tool_table={1: 50.0})
    assert state.tlc_offset == 50.0


def test_g43_unknown_h_leaves_offset_unchanged():
    state = ModalState()
    state.tlc_offset = 12.0
    state.update([W("G", 43), W("H", 2)], tool_table={1: 50.0})
    assert state.tlc_offset == 12.0


def test_g49_resets_tlc_offset():
    state = ModalState()
    state.update([W("G", 43), W("H", 1)], tool_table={1: 50.0})
    assert state.tlc_offset == 50.0
    state.update([W("G", 49)])
    assert state.tlc_offset == 0.0


def test_empty_line_does_not_change_state():
    state = ModalState()
    state.update([W("G", 91), W("G", 55)])
    before = (state.motion_mode, state.coord_system, state.distance_mode, state.plane, state.units)
    state.update([])
    after = (state.motion_mode, state.coord_system, state.distance_mode, state.plane, state.units)
    assert before == after


def test_comment_only_line_does_not_change_position():
    state = ModalState()
    state.update([W("X", 10)])
    state.update([])  # parsed comment produces no words
    assert state.position.x == 10.0


def test_motion_mode_canned_cycle():
    state = ModalState()
    state.update([W("G", 81)])
    assert state.motion_mode == "G81"


# --- arc_linearizer -------------------------------------------------------


def test_linearize_half_circle_point_count_matches_chord_tolerance():
    radius, tolerance = 10.0, 0.5
    segment = GCodeSegment(
        line_no=1,
        gcode_text="G3 X-10 Y0 I-10 J0",
        motion_mode="G3",
        start=Vector3(radius, 0, 0),
        end=Vector3(-radius, 0, 0),
        plane="G17",
        arc_center=Vector3(0, 0, 0),
    )
    points = linearize_arc(segment, chord_tolerance=tolerance)

    seg_angle = 2 * math.acos(1 - tolerance / radius)
    expected_n_segments = math.ceil(math.pi / seg_angle)
    assert len(points) == expected_n_segments + 1

    assert math.isclose(points[0].x, radius, abs_tol=1e-9)
    assert math.isclose(points[0].y, 0, abs_tol=1e-9)
    assert math.isclose(points[-1].x, -radius, abs_tol=1e-9)
    assert math.isclose(points[-1].y, 0, abs_tol=1e-9)

    # every point must lie on the circle
    for p in points:
        assert math.isclose(math.hypot(p.x, p.y), radius, abs_tol=1e-6)


def test_linearize_full_circle_is_closed():
    segment = GCodeSegment(
        line_no=1,
        gcode_text="G2 X10 Y0 I-10 J0",
        motion_mode="G2",
        start=Vector3(10, 0, 0),
        end=Vector3(10, 0, 0),
        plane="G17",
        arc_center=Vector3(0, 0, 0),
    )
    points = linearize_arc(segment, chord_tolerance=1.0)

    assert len(points) > 2
    assert math.isclose(points[0].x, points[-1].x, abs_tol=1e-6)
    assert math.isclose(points[0].y, points[-1].y, abs_tol=1e-6)


def test_linearize_helix_z_changes_linearly():
    segment = GCodeSegment(
        line_no=1,
        gcode_text="G3 X-10 Y0 Z5 I-10 J0",
        motion_mode="G3",
        start=Vector3(10, 0, 0),
        end=Vector3(-10, 0, 5),
        plane="G17",
        arc_center=Vector3(0, 0, 0),
    )
    points = linearize_arc(segment, chord_tolerance=0.5)

    assert math.isclose(points[0].z, 0.0, abs_tol=1e-9)
    assert math.isclose(points[-1].z, 5.0, abs_tol=1e-9)
    z_values = [p.z for p in points]
    assert z_values == sorted(z_values)  # monotonically increasing


def test_linearize_arc_requires_center():
    segment = GCodeSegment(
        line_no=1,
        gcode_text="G3 X-10 Y0",
        motion_mode="G3",
        start=Vector3(10, 0, 0),
        end=Vector3(-10, 0, 0),
    )
    try:
        linearize_arc(segment)
        assert False, "expected ValueError"
    except ValueError:
        pass


# --- canned cycles ----------------------------------------------------------


def test_g81_expands_to_rapid_feed_rapid():
    segment = GCodeSegment(
        line_no=10,
        gcode_text="G81 R5 Z-10",
        motion_mode="G81",
        start=Vector3(0, 0, 20),
        end=Vector3(0, 0, -10),
        r_plane=5.0,
        z_depth=-10.0,
    )
    segments = expand_canned_cycle(segment)

    assert len(segments) == 3
    rapid_in, feed_down, rapid_out = segments

    assert rapid_in.motion_mode == "G0"
    assert rapid_in.start == Vector3(0, 0, 20)
    assert rapid_in.end == Vector3(0, 0, 5)

    assert feed_down.motion_mode == "G1"
    assert feed_down.start == Vector3(0, 0, 5)
    assert feed_down.end == Vector3(0, 0, -10)

    assert rapid_out.motion_mode == "G0"
    assert rapid_out.start == Vector3(0, 0, -10)
    assert rapid_out.end == Vector3(0, 0, 5)


def test_g82_expands_same_as_g81_dwell_ignored():
    segment = GCodeSegment(
        line_no=11,
        gcode_text="G82 R5 Z-10 P500",
        motion_mode="G82",
        start=Vector3(0, 0, 20),
        end=Vector3(0, 0, -10),
        r_plane=5.0,
        z_depth=-10.0,
    )
    segments = expand_canned_cycle(segment)
    assert len(segments) == 3
    assert [s.motion_mode for s in segments] == ["G0", "G1", "G0"]


def test_g83_peck_drill_sequence():
    segment = GCodeSegment(
        line_no=20,
        gcode_text="G83 R5 Z-10 Q2",
        motion_mode="G83",
        start=Vector3(0, 0, 20),
        end=Vector3(0, 0, -10),
        r_plane=5.0,
        z_depth=-10.0,
        q_peck=2.0,
    )
    segments = expand_canned_cycle(segment)

    expected_depths = [3.0, 1.0, -1.0, -3.0, -5.0, -7.0, -9.0, -10.0]
    assert len(segments) == 1 + 2 * len(expected_depths)

    rapid_in = segments[0]
    assert rapid_in.motion_mode == "G0"
    assert rapid_in.end == Vector3(0, 0, 5)

    pecks = segments[1:]
    for i, depth in enumerate(expected_depths):
        feed, retract = pecks[2 * i], pecks[2 * i + 1]
        assert feed.motion_mode == "G1"
        assert feed.start.z == 5.0  # G83 always feeds from the R plane
        assert math.isclose(feed.end.z, depth, abs_tol=1e-9)
        assert retract.motion_mode == "G0"
        assert retract.end.z == 5.0  # G83 always retracts fully to R


def test_g73_partial_retract_between_pecks():
    segment = GCodeSegment(
        line_no=21,
        gcode_text="G73 R5 Z-10 Q2",
        motion_mode="G73",
        start=Vector3(0, 0, 20),
        end=Vector3(0, 0, -10),
        r_plane=5.0,
        z_depth=-10.0,
        q_peck=2.0,
    )
    segments = expand_canned_cycle(segment)

    pecks = segments[1:]
    first_retract = pecks[1]
    assert first_retract.motion_mode == "G0"
    assert first_retract.end.z < 5.0  # not a full retract to R, unlike G83

    last_retract = pecks[-1]
    assert last_retract.end.z == 5.0  # final retract still goes to R


def test_g84_tap_simplified_to_drill_motion():
    segment = GCodeSegment(
        line_no=30,
        gcode_text="G84 R5 Z-10",
        motion_mode="G84",
        start=Vector3(0, 0, 20),
        end=Vector3(0, 0, -10),
        r_plane=5.0,
        z_depth=-10.0,
    )
    segments = expand_canned_cycle(segment)
    assert [s.motion_mode for s in segments] == ["G0", "G1", "G0"]
