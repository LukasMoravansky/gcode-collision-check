import math

import pytest

from gcode_collision_check.tool.assembly import build_tool_assembly, default_tool
from gcode_collision_check.tool.profiles import PROFILES, get_profile
from gcode_collision_check.types import ToolConfig


def test_default_tool_is_10mm_flat_endmill_with_er32_holder():
    tool = default_tool()
    assert tool.diameter == 10.0
    assert tool.flute_length == 25.0
    assert tool.shank_diameter == 10.0
    assert tool.shank_length == 30.0
    assert tool.holder_diameter == 46.0
    assert tool.holder_length == 50.0
    assert tool.kind == "flat"


def test_flat_endmill_flute_bounding_box():
    tool = ToolConfig(
        diameter=10.0,
        flute_length=25.0,
        shank_diameter=10.0,
        shank_length=30.0,
        holder_diameter=46.0,
        holder_length=50.0,
        kind="flat",
    )
    assembly = build_tool_assembly(tool)
    flute = assembly["flute"]
    bounds = flute.bounds

    assert math.isclose(bounds[0][2], 0.0, abs_tol=1e-6)
    assert math.isclose(bounds[1][2], tool.flute_length, abs_tol=1e-6)
    assert math.isclose(bounds[1][0] - bounds[0][0], tool.diameter, abs_tol=1e-3)
    assert math.isclose(bounds[1][1] - bounds[0][1], tool.diameter, abs_tol=1e-3)


def test_ball_endmill_tip_is_rounded():
    tool = ToolConfig(
        diameter=6.0,
        flute_length=13.0,
        shank_diameter=6.0,
        shank_length=25.0,
        holder_diameter=32.0,
        holder_length=40.0,
        kind="ball",
    )
    assembly = build_tool_assembly(tool)
    flute = assembly["flute"]
    bounds = flute.bounds

    assert math.isclose(bounds[0][2], 0.0, abs_tol=1e-6)
    assert math.isclose(bounds[1][2], tool.flute_length, abs_tol=1e-6)

    # near the tip (Z close to 0), the profile must be narrower than the full
    # radius -- a rounded hemisphere, not a flat-bottomed cylinder
    radius = tool.diameter / 2
    near_tip = flute.vertices[flute.vertices[:, 2] < 0.5 * radius]
    assert len(near_tip) > 0
    max_xy_radius_near_tip = max(math.hypot(x, y) for x, y, z in near_tip)
    assert max_xy_radius_near_tip < radius - 1e-3


def test_bull_endmill_flute_bounding_box():
    tool = ToolConfig(
        diameter=12.0,
        flute_length=26.0,
        shank_diameter=12.0,
        shank_length=35.0,
        holder_diameter=46.0,
        holder_length=50.0,
        kind="bull",
        corner_radius=2.0,
    )
    assembly = build_tool_assembly(tool)
    flute = assembly["flute"]
    bounds = flute.bounds

    assert math.isclose(bounds[0][2], 0.0, abs_tol=1e-6)
    assert math.isclose(bounds[1][2], tool.flute_length, abs_tol=1e-6)
    assert math.isclose(bounds[1][0] - bounds[0][0], tool.diameter, abs_tol=1e-3)


def test_assembly_has_three_named_parts():
    assembly = build_tool_assembly(default_tool())
    assert set(assembly.keys()) == {"flute", "shank", "holder"}


def test_assembly_z_ordering_flute_shank_holder():
    tool = default_tool()
    assembly = build_tool_assembly(tool)
    flute_bounds = assembly["flute"].bounds
    shank_bounds = assembly["shank"].bounds
    holder_bounds = assembly["holder"].bounds

    assert math.isclose(flute_bounds[0][2], 0.0, abs_tol=1e-6)
    assert math.isclose(flute_bounds[1][2], shank_bounds[0][2], abs_tol=1e-6)
    assert math.isclose(shank_bounds[1][2], holder_bounds[0][2], abs_tol=1e-6)

    assert flute_bounds[1][2] <= shank_bounds[0][2] + 1e-6
    assert shank_bounds[1][2] <= holder_bounds[0][2] + 1e-6


def test_unknown_tool_kind_raises():
    tool = ToolConfig(
        diameter=6.0,
        flute_length=13.0,
        shank_diameter=6.0,
        shank_length=25.0,
        holder_diameter=32.0,
        holder_length=40.0,
        kind="unknown",
    )
    with pytest.raises(ValueError):
        build_tool_assembly(tool)


# --- profiles ---------------------------------------------------------------


def test_all_presets_are_registered():
    assert set(PROFILES.keys()) == {"6mm_ball", "10mm_flat", "12mm_bull"}


@pytest.mark.parametrize("name", ["6mm_ball", "10mm_flat", "12mm_bull"])
def test_get_profile_by_name(name):
    tool = get_profile(name)
    assert isinstance(tool, ToolConfig)
    assert tool is PROFILES[name]


def test_get_profile_builds_valid_assembly():
    for name in PROFILES:
        assembly = build_tool_assembly(get_profile(name))
        assert set(assembly.keys()) == {"flute", "shank", "holder"}


def test_get_profile_unknown_name_raises():
    with pytest.raises(ValueError):
        get_profile("does_not_exist")
