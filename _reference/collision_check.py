"""Collision query between two CollisionManagers (obstacles vs. tool group)."""

from __future__ import annotations

import trimesh
from trimesh.collision import CollisionManager
from trimesh.transformations import translation_matrix

from poc.collision_poc.types import CollisionEvent, PathSample, SceneCheckResult


def check_collision(obstacles: CollisionManager, tool_group: CollisionManager) -> SceneCheckResult:
    in_collision, names = obstacles.in_collision_other(tool_group, return_names=True)
    pairs = sorted(names)
    return SceneCheckResult(
        in_collision=bool(in_collision),
        contact_pairs=pairs,
        n_contacts=len(pairs),
    )


def check_gcode_path(
    samples: list[PathSample],
    tool_mesh: trimesh.Trimesh,
    obstacles: CollisionManager,
    *,
    z_max_obstacles: float,
    margin: float = 2.0,
) -> list[CollisionEvent]:
    """Step a tool proxy through sampled G-code positions, reporting collisions.

    The tool object is added once and moved per-sample via ``set_transform``
    (never re-added -- re-adding would rebuild the BVH on every sample).
    Samples whose tip is well above every obstacle (Z-prefilter) are skipped.
    """
    tool_mgr = CollisionManager()
    tool_mgr.add_object("tool", tool_mesh)

    events: list[CollisionEvent] = []
    z_cutoff = z_max_obstacles + margin

    for sample in samples:
        if sample.z > z_cutoff:
            continue

        tool_mgr.set_transform("tool", translation_matrix([sample.x, sample.y, sample.z]))
        in_collision, names, contacts = obstacles.in_collision_other(
            tool_mgr, return_names=True, return_data=True
        )
        if not in_collision:
            continue

        worst = max(contacts, key=lambda c: c.depth)
        events.append(
            CollisionEvent(
                line_no=sample.line_no,
                gcode=sample.gcode,
                x=sample.x,
                y=sample.y,
                z=sample.z,
                feed=sample.feed,
                contact_pairs=sorted(names),
                depth=float(worst.depth),
                point=tuple(float(v) for v in worst.point),
            )
        )

    return events
