import json
from dataclasses import asdict

from gcode_collision_check.types import CollisionEvent, ToolConfig, VerifyResult


def make_tool_config(**overrides) -> ToolConfig:
    defaults = dict(
        diameter=6.0,
        flute_length=20.0,
        shank_diameter=6.0,
        shank_length=30.0,
        holder_diameter=25.0,
        holder_length=50.0,
    )
    defaults.update(overrides)
    return ToolConfig(**defaults)


def make_collision_event(**overrides) -> CollisionEvent:
    defaults = dict(
        line_no=42,
        gcode_text="G1 X10 Y20 Z-5 F300",
        position=(10.0, 20.0, -5.0),
        position_wcs=(10.0, 20.0, -5.0),
        wcs="G54",
        pairs=[("holder", "vise_jaw_left")],
        penetration_depth=0.35,
    )
    defaults.update(overrides)
    return CollisionEvent(**defaults)


def test_tool_config_defaults():
    tool = make_tool_config()
    assert tool.kind == "flat"
    assert tool.corner_radius == 0


def test_tool_config_bull_endmill():
    tool = make_tool_config(kind="bull", corner_radius=1.5)
    assert tool.kind == "bull"
    assert tool.corner_radius == 1.5


def test_tool_config_to_dict():
    tool = make_tool_config()
    data = tool.to_dict()
    assert data == {
        "diameter": 6.0,
        "flute_length": 20.0,
        "shank_diameter": 6.0,
        "shank_length": 30.0,
        "holder_diameter": 25.0,
        "holder_length": 50.0,
        "kind": "flat",
        "corner_radius": 0,
    }
    json.dumps(data)


def test_tool_config_asdict():
    tool = make_tool_config()
    assert asdict(tool)["diameter"] == 6.0


def test_collision_event_creation():
    event = make_collision_event()
    assert event.line_no == 42
    assert event.wcs == "G54"
    assert event.pairs == [("holder", "vise_jaw_left")]


def test_collision_event_to_dict():
    event = make_collision_event()
    data = event.to_dict()
    assert data == {
        "line_no": 42,
        "gcode_text": "G1 X10 Y20 Z-5 F300",
        "position": [10.0, 20.0, -5.0],
        "position_wcs": [10.0, 20.0, -5.0],
        "wcs": "G54",
        "pairs": [["holder", "vise_jaw_left"]],
        "penetration_depth": 0.35,
    }
    json.dumps(data)


def test_verify_result_safe_no_events():
    result = VerifyResult(
        safe=True,
        events=[],
        total_samples=100,
        total_segments=10,
        elapsed_seconds=0.5,
    )
    assert result.safe is True
    assert result.events == []


def test_verify_result_with_events():
    event = make_collision_event()
    result = VerifyResult(
        safe=False,
        events=[event],
        total_samples=100,
        total_segments=10,
        elapsed_seconds=1.25,
    )
    assert result.safe is False
    assert len(result.events) == 1
    assert result.events[0] is event


def test_verify_result_to_dict():
    event = make_collision_event()
    result = VerifyResult(
        safe=False,
        events=[event],
        total_samples=100,
        total_segments=10,
        elapsed_seconds=1.25,
    )
    data = result.to_dict()
    assert data["safe"] is False
    assert data["events"] == [event.to_dict()]
    assert data["total_samples"] == 100
    assert data["total_segments"] == 10
    assert data["elapsed_seconds"] == 1.25
    json.dumps(data)
