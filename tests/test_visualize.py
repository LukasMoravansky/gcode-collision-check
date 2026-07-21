import numpy as np
import trimesh

from gcode_collision_check import visualize


def _box_scene():
    scene_meshes = {"vise": trimesh.creation.box(extents=[20, 20, 20])}
    tool_parts = {
        "flute": trimesh.creation.cylinder(radius=5, height=10),
        "shank": trimesh.creation.cylinder(radius=5, height=10),
    }
    tool_position = np.array([0.0, 0.0, 30.0])
    toolpath_points = [np.array([0.0, 0.0, 30.0]), np.array([10.0, 0.0, 30.0])]
    return scene_meshes, tool_parts, tool_position, toolpath_points


def test_build_scene_has_expected_node_names():
    scene_meshes, tool_parts, tool_position, toolpath_points = _box_scene()

    viz = visualize.build_scene(
        scene_meshes, tool_parts, tool_position, toolpath_points, is_collision=False
    )

    assert "obstacle_vise" in viz.graph.nodes_geometry
    assert "tool_flute" in viz.graph.nodes_geometry
    assert "tool_shank" in viz.graph.nodes_geometry
    assert "toolpath" in viz.graph.nodes_geometry


def test_export_html_writes_glb_and_html(tmp_path):
    scene_meshes, tool_parts, tool_position, toolpath_points = _box_scene()
    viz = visualize.build_scene(
        scene_meshes, tool_parts, tool_position, toolpath_points, is_collision=True
    )

    glb_path, html_path = visualize.export_html(viz, tmp_path)

    assert glb_path.exists()
    assert html_path.exists()

    html_content = html_path.read_text(encoding="utf-8")
    assert "<model-viewer" in html_content
    assert 'src="data:model/gltf-binary;base64,' in html_content

    loaded = trimesh.load(str(glb_path))
    assert loaded is not None


def test_open_in_browser_does_not_open_when_flag_off(tmp_path, monkeypatch):
    scene_meshes, tool_parts, tool_position, toolpath_points = _box_scene()
    viz = visualize.build_scene(
        scene_meshes, tool_parts, tool_position, toolpath_points, is_collision=False
    )

    opened = []
    monkeypatch.setattr(visualize.webbrowser, "open", lambda url: opened.append(url))

    output_dir = tmp_path / "viz_out"
    result_dir = visualize.open_in_browser(viz, output_dir=output_dir, open_browser=False)

    assert result_dir == output_dir
    assert (output_dir / "scene.glb").exists()
    assert (output_dir / "scene.html").exists()
    assert opened == []
