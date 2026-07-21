import math

import numpy as np
import pytest
import trimesh

from gcode_collision_check.collision.checker import _rotation_matrix, verify
from gcode_collision_check.collision.scene import CollisionScene
from gcode_collision_check.tool.assembly import build_tool_assembly, default_tool


def _vise_jaw_mesh():
    """A slab below the stock's top face, spanning Z in [-20, -5].

    Left with a 5mm gap below Z=0 so the parser's default start position
    (0, 0, 0) is safely above it (beyond the Z-prefilter's default margin)
    -- only an actual plunge triggers a collision.
    """
    box = trimesh.creation.box(extents=(60, 60, 15))
    box.apply_translation([0, 0, -12.5])
    return box


@pytest.fixture
def vise_stl(tmp_path):
    path = tmp_path / "vise.stl"
    _vise_jaw_mesh().export(path)
    return str(path)


def _write_program(tmp_path, text):
    path = tmp_path / "program.nc"
    path.write_text(text)
    return str(path)


# --- CollisionScene (unit) ---------------------------------------------------


def test_scene_detects_collision_when_tool_overlaps_obstacle():
    scene = CollisionScene()
    scene.add_obstacle("vise", _vise_jaw_mesh())
    scene.set_tool(build_tool_assembly(default_tool()))

    in_collision, pairs, contacts = scene.check_at_position(np.array([0, 0, -10]))

    assert in_collision is True
    assert any(pair[1] == "vise" for pair in pairs)
    assert contacts[0].depth > 0


def test_scene_no_collision_when_tool_clear():
    scene = CollisionScene()
    scene.add_obstacle("vise", _vise_jaw_mesh())
    scene.set_tool(build_tool_assembly(default_tool()))

    in_collision, pairs, contacts = scene.check_at_position(np.array([0, 0, 50]))

    assert in_collision is False
    assert pairs == []
    assert contacts == []


def test_obstacles_z_max():
    scene = CollisionScene()
    scene.add_obstacle("vise", _vise_jaw_mesh())
    assert scene.obstacles_z_max() == pytest.approx(-5.0)


def test_obstacles_z_max_empty_scene_is_minus_infinity():
    scene = CollisionScene()
    assert scene.obstacles_z_max() == -math.inf


# --- verify() end-to-end -----------------------------------------------------


def test_verify_known_collision(tmp_path, vise_stl):
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 Z-10 F100
        """,
    )
    result = verify(program, {"vise": vise_stl}, default_tool())

    assert result.safe is False
    assert len(result.events) > 0
    event = result.events[-1]
    assert event.pairs == [("flute", "vise")]
    assert event.penetration_depth > 0


def test_verify_known_safe(tmp_path, vise_stl):
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 X10 Y10 Z50
        G1 X-10 Y-10 Z50
        """,
    )
    result = verify(program, {"vise": vise_stl}, default_tool())

    assert result.safe is True
    assert result.events == []
    # every segment stays above the obstacles, so the Z-prefilter skips all of them
    assert result.total_samples == 0
    assert result.total_segments == 3


def test_verify_z_prefilter_skips_collision_queries(tmp_path, vise_stl, monkeypatch):
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 X10 Y10 Z50
        G1 X-10 Y-10 Z50
        """,
    )

    call_count = 0
    original = CollisionScene.check_at_position

    def counting_check(self, xyz):
        nonlocal call_count
        call_count += 1
        return original(self, xyz)

    monkeypatch.setattr(CollisionScene, "check_at_position", counting_check)

    result = verify(program, {"vise": vise_stl}, default_tool())

    assert call_count == 0
    assert result.safe is True


def test_verify_z_prefilter_still_checks_segments_that_dip_low(tmp_path, vise_stl, monkeypatch):
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 Z-10 F100
        G0 Z50
        """,
    )

    call_count = 0
    original = CollisionScene.check_at_position

    def counting_check(self, xyz):
        nonlocal call_count
        call_count += 1
        return original(self, xyz)

    monkeypatch.setattr(CollisionScene, "check_at_position", counting_check)

    result = verify(program, {"vise": vise_stl}, default_tool())

    assert call_count > 0
    assert result.safe is False


# --- program_origin -----------------------------------------------------------


def test_verify_program_origin_moves_program_away_from_obstacle(tmp_path, vise_stl):
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 Z-10 F100
        """,
    )
    result = verify(program, {"vise": vise_stl}, default_tool(), program_origin=(1000, 1000, 1000))

    assert result.safe is True
    assert result.events == []


def test_verify_program_origin_moves_program_into_obstacle(tmp_path, vise_stl):
    # dips to Z-10 (inside the vise's Z range) but far away in X/Y, so it is
    # safe at the default origin -- shifting the origin lands the dip at
    # X0/Y0, squarely inside the vise jaw.
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X500 Y500 Z50
        G1 X500 Y500 Z-10 F100
        """,
    )
    result_default = verify(program, {"vise": vise_stl}, default_tool())
    assert result_default.safe is True

    result = verify(program, {"vise": vise_stl}, default_tool(), program_origin=(-500, -500, 0))

    assert result.safe is False
    assert len(result.events) > 0


def test_verify_program_origin_applies_regardless_of_active_wcs(tmp_path, vise_stl):
    program_g54 = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 Z-10 F100
        """,
    )
    program_g55 = _write_program(
        tmp_path,
        """
        G21 G90 G55
        G0 X0 Y0 Z50
        G1 Z-10 F100
        """,
    )
    origin = (0, 0, 500)
    result_g54 = verify(program_g54, {"vise": vise_stl}, default_tool(), program_origin=origin)
    result_g55 = verify(program_g55, {"vise": vise_stl}, default_tool(), program_origin=origin)

    assert result_g54.safe == result_g55.safe is True


# --- program_rotation: math / convention ---------------------------------------


def test_rotation_matrix_c90_sends_program_x_to_machine_y():
    r = _rotation_matrix((0, 0, 90))

    np.testing.assert_allclose(r @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)


def test_rotation_matrix_none_is_identity():
    r = _rotation_matrix(None)

    np.testing.assert_array_equal(r, np.eye(3))


def test_rotation_matrix_composition_order_is_rz_ry_rx():
    # Hand-computed via the same Rx/Ry/Rz definitions (independent formulas,
    # not the implementation under test): R = Rz(60) @ Ry(45) @ Rx(30) applied to (1, 2, 3).
    a, b, c = math.radians(30), math.radians(45), math.radians(60)
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    expected = rz @ ry @ rx @ np.array([1.0, 2.0, 3.0])

    r = _rotation_matrix((30, 45, 60))

    np.testing.assert_allclose(r @ np.array([1.0, 2.0, 3.0]), expected, atol=1e-9)


def test_program_rotation_does_not_move_the_datum_point(tmp_path, vise_stl):
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z0
        """,
    )
    from gcode_collision_check.collision.checker import verify_with_scene

    _, _, _, toolpath_points = verify_with_scene(
        program,
        {"vise": vise_stl},
        default_tool(),
        program_origin=(100, 200, -10),
        program_rotation=(10, 20, 30),
    )

    assert toolpath_points
    np.testing.assert_allclose(toolpath_points[0], [100, 200, -10], atol=1e-9)


# --- program_rotation: collision behavior --------------------------------------


def test_verify_program_rotation_turns_safe_program_into_collision(tmp_path, vise_stl):
    # raw path ends at X=40,Y=0,Z=-10 -- outside the vise's |X|<=30 footprint, so safe.
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 X40 Y0 Z-10 F100
        """,
    )
    result_default = verify(program, {"vise": vise_stl}, default_tool())
    assert result_default.safe is True

    # C=45 rotates the endpoint to (28.28, 28.28, -10) -- inside the jaw.
    result = verify(program, {"vise": vise_stl}, default_tool(), program_rotation=(0, 0, 45))

    assert result.safe is False
    assert len(result.events) > 0


def test_verify_program_rotation_turns_collision_into_safe(tmp_path, vise_stl):
    # raw path ends at (28.28, 28.28, -10) -- inside the vise jaw, so it collides.
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 X28.284271 Y28.284271 Z-10 F100
        """,
    )
    result_default = verify(program, {"vise": vise_stl}, default_tool())
    assert result_default.safe is False

    # C=-45 rotates the endpoint back out to (40, 0, -10) -- outside the jaw.
    result = verify(program, {"vise": vise_stl}, default_tool(), program_rotation=(0, 0, -45))

    assert result.safe is True


def test_verify_program_rotation_z_prefilter_still_checks_rotated_dip(tmp_path, vise_stl, monkeypatch):
    # raw path never leaves Z=[10, 50] -- well above the vise, so the naive
    # (unrotated) Z-prefilter would skip every sample. A B=180 flip sends
    # this segment's Z from [10, 50] to [-50, -10], squarely into the jaw.
    program = _write_program(
        tmp_path,
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 X0 Y0 Z10 F100
        """,
    )

    call_count = 0
    original = CollisionScene.check_at_position

    def counting_check(self, xyz):
        nonlocal call_count
        call_count += 1
        return original(self, xyz)

    monkeypatch.setattr(CollisionScene, "check_at_position", counting_check)

    result = verify(program, {"vise": vise_stl}, default_tool(), program_rotation=(0, 180, 0))

    assert call_count > 0
    assert result.safe is False
