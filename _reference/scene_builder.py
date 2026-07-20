"""Load meshes and place them in the scene: scale baked into vertices, rigid
transform applied on top (fcl set_transform is a rigid-only, absolute pose
relative to the vertices present at add_object time)."""

from __future__ import annotations

from pathlib import Path

import trimesh
from trimesh.collision import CollisionManager

from poc.collision_poc.types import ScenePlacement


def load_mesh(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(path, force="mesh")
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded
    if mesh.is_empty:
        raise ValueError(f"Mesh is empty: {path}")
    return mesh


def apply_placement(mesh: trimesh.Trimesh, placement: ScenePlacement) -> trimesh.Trimesh:
    """Bake scale into vertices; return the (unplaced) mesh for add_object.

    The rigid part of the placement is not applied here -- pass it to
    ``add_object(transform=...)`` / ``set_transform`` instead, so callers can
    cheaply re-test different offsets without rebuilding the BVH.
    """
    placed = mesh.copy()
    if placement.scale != 1.0:
        placed.apply_scale(placement.scale)
    return placed


def build_manager(name: str, mesh: trimesh.Trimesh, placement: ScenePlacement) -> CollisionManager:
    """Single-object CollisionManager with scale baked in and rigid transform applied."""
    manager = CollisionManager()
    scaled = apply_placement(mesh, placement)
    manager.add_object(name, scaled, transform=placement.transform.to_matrix())
    return manager
