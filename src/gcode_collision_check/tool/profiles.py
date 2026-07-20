"""Built-in tool profiles, looked up by name."""

from __future__ import annotations

from gcode_collision_check.types import ToolConfig

PROFILES: dict[str, ToolConfig] = {
    "6mm_ball": ToolConfig(
        diameter=6.0,
        flute_length=13.0,
        shank_diameter=6.0,
        shank_length=25.0,
        holder_diameter=32.0,
        holder_length=40.0,
        kind="ball",
    ),
    "10mm_flat": ToolConfig(
        diameter=10.0,
        flute_length=25.0,
        shank_diameter=10.0,
        shank_length=30.0,
        holder_diameter=46.0,
        holder_length=50.0,
        kind="flat",
    ),
    "12mm_bull": ToolConfig(
        diameter=12.0,
        flute_length=26.0,
        shank_diameter=12.0,
        shank_length=35.0,
        holder_diameter=46.0,
        holder_length=50.0,
        kind="bull",
        corner_radius=2.0,
    ),
}


def get_profile(name: str) -> ToolConfig:
    """Look up a built-in tool profile by name."""
    try:
        return PROFILES[name]
    except KeyError:
        available = ", ".join(sorted(PROFILES))
        raise ValueError(f"unknown tool profile {name!r}; available: {available}") from None
