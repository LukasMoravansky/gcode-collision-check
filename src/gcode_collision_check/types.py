"""Core dataclasses shared across the parser, tool, and collision modules."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ToolConfig:
    """Geometry of a tool assembly: cutter + shank + holder."""

    diameter: float  # mm
    flute_length: float  # mm
    shank_diameter: float  # mm
    shank_length: float  # mm
    holder_diameter: float  # mm
    holder_length: float  # mm
    kind: str = "flat"  # flat | ball | bull
    corner_radius: float = 0  # mm, pro bull endmill

    def to_dict(self) -> dict:
        return {
            "diameter": self.diameter,
            "flute_length": self.flute_length,
            "shank_diameter": self.shank_diameter,
            "shank_length": self.shank_length,
            "holder_diameter": self.holder_diameter,
            "holder_length": self.holder_length,
            "kind": self.kind,
            "corner_radius": self.corner_radius,
        }


@dataclass
class CollisionEvent:
    """A collision detected at one sample along the tool path."""

    line_no: int
    gcode_text: str
    position: tuple[float, float, float]  # XYZ v machine frame
    position_wcs: tuple[float, float, float]  # XYZ v active WCS
    wcs: str  # "G54" etc.
    pairs: list[tuple[str, str]]  # [("holder", "vise_jaw_left"), ...]
    penetration_depth: float  # mm

    def to_dict(self) -> dict:
        return {
            "line_no": self.line_no,
            "gcode_text": self.gcode_text,
            "position": list(self.position),
            "position_wcs": list(self.position_wcs),
            "wcs": self.wcs,
            "pairs": [list(pair) for pair in self.pairs],
            "penetration_depth": self.penetration_depth,
        }


@dataclass
class VerifyResult:
    """Outcome of a full G-code verification run."""

    safe: bool
    events: list[CollisionEvent]
    total_samples: int
    total_segments: int
    elapsed_seconds: float

    def to_dict(self) -> dict:
        return {
            "safe": self.safe,
            "events": [event.to_dict() for event in self.events],
            "total_samples": self.total_samples,
            "total_segments": self.total_segments,
            "elapsed_seconds": self.elapsed_seconds,
        }
