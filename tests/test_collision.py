import math

import numpy as np
import pytest
import trimesh

from gcode_collision_check.collision.checker import verify
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
