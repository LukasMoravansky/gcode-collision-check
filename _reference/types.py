"""Dataclasses for the collision PoC."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class RigidTransform:
    """Translation + Euler rotation (degrees, XYZ order, extrinsic). No scale."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def to_matrix(self) -> np.ndarray:
        from trimesh.transformations import euler_matrix

        matrix = euler_matrix(
            np.radians(self.rx), np.radians(self.ry), np.radians(self.rz), axes="sxyz"
        )
        matrix[:3, 3] = [self.x, self.y, self.z]
        return matrix

    @classmethod
    def from_csv(cls, text: str) -> "RigidTransform":
        parts = [float(p.strip()) for p in text.split(",")]
        if len(parts) not in (3, 6):
            raise ValueError(
                "Transform must be 'x,y,z' or 'x,y,z,rx,ry,rz' (degrees)"
            )
        if len(parts) == 3:
            parts = parts + [0.0, 0.0, 0.0]
        return cls(*parts)


@dataclass
class ScenePlacement:
    """Rigid transform plus a uniform scale baked into mesh vertices before add."""

    transform: RigidTransform = field(default_factory=RigidTransform)
    scale: float = 1.0

    @classmethod
    def from_args(cls, transform_csv: str | None, scale: float) -> "ScenePlacement":
        transform = RigidTransform.from_csv(transform_csv) if transform_csv else RigidTransform()
        return cls(transform=transform, scale=scale)


@dataclass
class ViseGeometry:
    """Known vise jaw geometry, read off ``models/sverak_standalone.stl``.

    Clamping axis is X (jaw faces sit at jaw_x_min/jaw_x_max); jaw width runs
    along Y. Values are hardcoded for the standalone vise mesh -- future work
    may derive these from the mesh automatically.
    """

    jaw_x_min: float = 17.0
    jaw_x_max: float = 67.0
    jaw_y_min: float = -35.0
    jaw_y_max: float = 35.0
    jaw_z_top: float = 39.5
    support_z: float = 38.0


def compute_machining_origin(vise: ViseGeometry, stock_height: float) -> np.ndarray:
    """Origin (WCS zero) at the center of the stock's top face.

    ``stock_height`` is the stock dimension along Z, measured up from the
    vise's support face (``vise.support_z``).
    """
    origin_x = (vise.jaw_x_min + vise.jaw_x_max) / 2
    origin_y = (vise.jaw_y_min + vise.jaw_y_max) / 2
    origin_z = vise.support_z + stock_height
    return np.array([origin_x, origin_y, origin_z])


# G-code/swept-volume fixtures seen so far (e.g. reference_swept.stl) are
# authored with their local X/Y swapped relative to the vise: local X spans
# the vise's jaw-width direction (Y here) and local Y spans the clamping
# direction (X here). A 90 degree rotation about Z, taken at the machining
# origin (which is also the swept mesh's own local (0,0,0)), reconciles the
# two frames without any extra translation. The sign (+90 vs -90) depends on
# which way the source G-code's X axis points and must be confirmed visually
# per G-code -- override with a negative value if the tool path comes out
# mirrored.
DEFAULT_SWEPT_RZ_DEG = 90.0


def compute_machining_transform(
    vise: ViseGeometry, stock_height: float, rz_deg: float = DEFAULT_SWEPT_RZ_DEG
) -> RigidTransform:
    """Full placement (origin + axis-mismatch correction) for the swept volume."""
    origin = compute_machining_origin(vise, stock_height)
    return RigidTransform(x=origin[0], y=origin[1], z=origin[2], rz=rz_deg)


@dataclass
class SceneCheckResult:
    in_collision: bool
    contact_pairs: list[tuple[str, str]]
    n_contacts: int

    def to_dict(self) -> dict:
        return {
            "in_collision": bool(self.in_collision),
            "contact_pairs": [list(pair) for pair in self.contact_pairs],
            "n_contacts": int(self.n_contacts),
        }


@dataclass
class PathSample:
    """One point along a linearized G-code tool path.

    ``line_no``/``gcode``/``feed`` are those of the destination move (the one
    that produced this sample) -- ordinary G-code semantics: a motion word on
    line N ends at this point.
    """

    line_no: int
    gcode: str
    x: float
    y: float
    z: float
    feed: float | None = None


@dataclass
class CollisionEvent:
    """A collision detected at one sample along the tool path."""

    line_no: int
    gcode: str
    x: float
    y: float
    z: float
    feed: float | None
    contact_pairs: list[tuple[str, str]]
    depth: float
    point: tuple[float, float, float]

    def to_dict(self) -> dict:
        return {
            "line_no": self.line_no,
            "gcode": self.gcode,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "feed": self.feed,
            "contact_pairs": [list(pair) for pair in self.contact_pairs],
            "depth": self.depth,
            "point": list(self.point),
        }
