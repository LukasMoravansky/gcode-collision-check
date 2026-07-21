"""Builds an interactive 3D view (GLB + HTML) of a collision-check run."""

from __future__ import annotations

import base64
import tempfile
import webbrowser
from pathlib import Path

import numpy as np
import trimesh


def build_scene(
    scene_meshes: dict[str, trimesh.Trimesh],
    tool_parts: dict[str, trimesh.Trimesh],
    tool_position: np.ndarray,
    toolpath_points: list[np.ndarray],
    is_collision: bool,
) -> trimesh.Scene:
    """Assemble the visualization scene."""
    viz = trimesh.Scene()

    for name, mesh in scene_meshes.items():
        m = mesh.copy()
        m.visual.face_colors = [180, 180, 180, 180]
        viz.add_geometry(m, node_name=f"obstacle_{name}")

    color = [220, 40, 40, 255] if is_collision else [40, 180, 40, 255]
    transform = np.eye(4)
    transform[:3, 3] = tool_position
    for part_name, mesh in tool_parts.items():
        m = mesh.copy()
        m.apply_transform(transform)
        m.visual.face_colors = color
        viz.add_geometry(m, node_name=f"tool_{part_name}")

    if len(toolpath_points) >= 2:
        points = np.array(toolpath_points)
        path = trimesh.load_path(points)
        path.colors = [[255, 200, 0, 255]] * len(path.entities)
        viz.add_geometry(path, node_name="toolpath")

    return viz


def export_html(scene: trimesh.Scene, output_dir: Path) -> tuple[Path, Path]:
    """Export the scene as GLB + an HTML file with a <model-viewer> tag.

    The GLB is embedded into the HTML as a base64 data URI rather than
    referenced by relative path: <model-viewer> loads its ``src`` via
    ``fetch()``, and browsers block that fetch for ``file://`` pages (not
    just Safari -- Chrome and Firefox do too). A data URI needs no network
    fetch, so the page works when opened directly from disk.
    """
    glb_path = output_dir / "scene.glb"
    html_path = output_dir / "scene.html"

    scene.export(str(glb_path), file_type="glb")
    glb_b64 = base64.b64encode(glb_path.read_bytes()).decode("ascii")
    glb_data_uri = f"data:model/gltf-binary;base64,{glb_b64}"

    html_content = f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>gcode-collision-check — 3D view</title>
  <script type="module"
    src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js">
  </script>
  <style>
    body {{ margin: 0; background: #1a1a2e; }}
    model-viewer {{
      width: 100vw; height: 100vh;
      --poster-color: #1a1a2e;
    }}
    .info {{
      position: fixed; top: 16px; left: 16px;
      color: #e0e0e0; font-family: monospace; font-size: 14px;
      background: rgba(0,0,0,0.6); padding: 8px 12px; border-radius: 4px;
    }}
  </style>
</head>
<body>
  <model-viewer
    src="{glb_data_uri}"
    camera-controls
    auto-rotate
    shadow-intensity="0.5"
    environment-image="neutral"
    camera-orbit="45deg 55deg auto"
    min-camera-orbit="auto auto auto"
    max-camera-orbit="auto auto auto"
    interaction-prompt="auto">
  </model-viewer>
  <div class="info">gcode-collision-check &middot; drag to rotate &middot; scroll to zoom</div>
</body>
</html>"""

    html_path.write_text(html_content, encoding="utf-8")
    return glb_path, html_path


def open_in_browser(
    scene: trimesh.Scene, output_dir: Path | None = None, open_browser: bool = True
) -> Path:
    """Export the scene to ``output_dir`` (or a fresh temp dir) and optionally open it."""
    target_dir = Path(output_dir) if output_dir is not None else Path(tempfile.mkdtemp(prefix="gcc_viz_"))
    target_dir.mkdir(parents=True, exist_ok=True)
    _glb_path, html_path = export_html(scene, target_dir)
    if open_browser:
        webbrowser.open(f"file://{html_path}")
    return target_dir
