"""Builds a tool assembly (flute + shank + holder) as trimesh meshes.

Every mesh is built with its axis along Z, tip at Z=0, growing in +Z.
"""

from __future__ import annotations

import trimesh

from gcode_collision_check.types import ToolConfig


def _cylinder_from(radius: float, height: float, z_bottom: float) -> trimesh.Trimesh:
    """A Z-axis cylinder spanning [z_bottom, z_bottom + height]."""
    mesh = trimesh.creation.cylinder(radius=radius, height=height)
    mesh.apply_translation([0, 0, z_bottom + height / 2])
    return mesh


def _flat_flute(config: ToolConfig) -> trimesh.Trimesh:
    return _cylinder_from(config.diameter / 2, config.flute_length, z_bottom=0.0)


def _ball_flute(config: ToolConfig) -> trimesh.Trimesh:
    radius = config.diameter / 2
    sphere = trimesh.creation.uv_sphere(radius=radius)
    sphere.apply_translation([0, 0, radius])
    hemisphere = sphere.slice_plane(
        plane_origin=[0, 0, radius], plane_normal=[0, 0, -1], cap=True
    )

    shaft_height = config.flute_length - radius
    parts = [hemisphere]
    if shaft_height > 0:
        parts.append(_cylinder_from(radius, shaft_height, z_bottom=radius))
    return trimesh.util.concatenate(parts)


def _bull_flute(config: ToolConfig) -> trimesh.Trimesh:
    radius = config.diameter / 2
    corner = config.corner_radius
    corner_torus = trimesh.creation.torus(
        major_radius=radius - corner, minor_radius=corner
    )
    corner_torus.apply_translation([0, 0, corner])

    shaft_height = config.flute_length - corner
    parts = [corner_torus]
    if shaft_height > 0:
        parts.append(_cylinder_from(radius, shaft_height, z_bottom=corner))
    return trimesh.util.concatenate(parts)


_FLUTE_BUILDERS = {
    "flat": _flat_flute,
    "ball": _ball_flute,
    "bull": _bull_flute,
}


def build_tool_assembly(config: ToolConfig) -> dict[str, trimesh.Trimesh]:
    """Build the flute, shank, and holder meshes for a tool assembly."""
    try:
        build_flute = _FLUTE_BUILDERS[config.kind]
    except KeyError:
        raise ValueError(f"unknown tool kind: {config.kind!r}") from None

    flute = build_flute(config)
    shank = _cylinder_from(
        config.shank_diameter / 2, config.shank_length, z_bottom=config.flute_length
    )
    holder = _cylinder_from(
        config.holder_diameter / 2,
        config.holder_length,
        z_bottom=config.flute_length + config.shank_length,
    )

    return {"flute": flute, "shank": shank, "holder": holder}


def default_tool() -> ToolConfig:
    """A reasonable default: 10mm flat endmill with an ER32 holder."""
    return ToolConfig(
        diameter=10.0,
        flute_length=25.0,
        shank_diameter=10.0,
        shank_length=30.0,
        holder_diameter=46.0,
        holder_length=50.0,
        kind="flat",
        corner_radius=0,
    )
