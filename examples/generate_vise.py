"""Generates examples/vise.stl: a generic parametric vise (two jaws + base).

Coordinates are chosen to line up with examples/crash.nc and examples/safe.nc:
X=0 (the programs' machining origin) sits inside the jaw opening, off-center
towards the right jaw -- mirroring how a part is often clamped off-center in
a real vise rather than dead in the middle.
"""

from pathlib import Path

import trimesh

OUTPUT_PATH = Path(__file__).parent / "vise.stl"

JAW_WIDTH = 70.0  # Y extent
JAW_HEIGHT = 50.0  # Z extent
JAW_THICKNESS = 15.0  # X extent
OPENING = 60.0  # gap between the jaws' inner faces, along X

BASE_SIZE = (100.0, 80.0, 20.0)  # X, Y, Z

JAW_TOP_Z = 20.0  # jaws stand up above the machining origin (Z=0)
JAW_BOTTOM_Z = JAW_TOP_Z - JAW_HEIGHT  # -30.0
BASE_TOP_Z = JAW_BOTTOM_Z  # base sits flush under the jaws
BASE_BOTTOM_Z = BASE_TOP_Z - BASE_SIZE[2]  # -50.0

LEFT_JAW_INNER_X = -40.0
RIGHT_JAW_INNER_X = LEFT_JAW_INNER_X + OPENING  # 20.0


def build_vise() -> trimesh.Trimesh:
    left_jaw = trimesh.creation.box(extents=(JAW_THICKNESS, JAW_WIDTH, JAW_HEIGHT))
    left_jaw.apply_translation(
        [LEFT_JAW_INNER_X - JAW_THICKNESS / 2, 0, (JAW_TOP_Z + JAW_BOTTOM_Z) / 2]
    )

    right_jaw = trimesh.creation.box(extents=(JAW_THICKNESS, JAW_WIDTH, JAW_HEIGHT))
    right_jaw.apply_translation(
        [RIGHT_JAW_INNER_X + JAW_THICKNESS / 2, 0, (JAW_TOP_Z + JAW_BOTTOM_Z) / 2]
    )

    base = trimesh.creation.box(extents=BASE_SIZE)
    base_center_x = (LEFT_JAW_INNER_X - JAW_THICKNESS + RIGHT_JAW_INNER_X + JAW_THICKNESS) / 2
    base.apply_translation([base_center_x, 0, (BASE_TOP_Z + BASE_BOTTOM_Z) / 2])

    return trimesh.util.concatenate([left_jaw, right_jaw, base])


def main() -> None:
    vise = build_vise()
    vise.export(OUTPUT_PATH)
    print(f"wrote {OUTPUT_PATH} ({len(vise.vertices)} vertices, {len(vise.faces)} faces)")
    print(f"bounds: {vise.bounds.tolist()}")


if __name__ == "__main__":
    main()
