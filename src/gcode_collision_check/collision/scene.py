"""Two collision managers: a static scene (obstacles) and a movable tool assembly.

Objects are added once at init time; per-sample checks only call
``set_transform`` -- never re-add, which would blow up the BVH broadphase.
"""

from __future__ import annotations

import math

import numpy as np
import trimesh
import trimesh.collision


class CollisionScene:
    def __init__(self):
        self._obstacles = trimesh.collision.CollisionManager()
        self._tool = trimesh.collision.CollisionManager()
        self._obstacle_meshes: dict[str, trimesh.Trimesh] = {}
        self._tool_parts: dict[str, trimesh.Trimesh] = {}

    def add_obstacle(self, name: str, mesh: trimesh.Trimesh) -> None:
        self._obstacles.add_object(name, mesh)
        self._obstacle_meshes[name] = mesh

    def set_tool(self, parts: dict[str, trimesh.Trimesh]) -> None:
        self._tool_parts = dict(parts)
        for name, mesh in parts.items():
            self._tool.add_object(name, mesh)

    def check_at_position(self, xyz: np.ndarray) -> tuple[bool, list, list]:
        """Move the tool assembly so its origin sits at ``xyz`` and query collisions."""
        transform = np.eye(4)
        transform[:3, 3] = xyz
        for name in self._tool_parts:
            self._tool.set_transform(name, transform)

        in_collision, names, contacts = self._tool.in_collision_other(
            self._obstacles, return_names=True, return_data=True
        )
        return in_collision, sorted(names), contacts

    def obstacles_z_max(self) -> float:
        """Highest Z among all obstacles, used for the Z-prefilter."""
        if not self._obstacle_meshes:
            return -math.inf
        return max(mesh.bounds[1][2] for mesh in self._obstacle_meshes.values())
