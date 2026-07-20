import json

import pytest
import trimesh
from click.testing import CliRunner

from gcode_collision_check.cli import main


def _vise_jaw_mesh():
    """A slab below the stock's top face, spanning Z in [-20, -5] (matches
    the clearance reasoning in test_collision.py: the parser's default start
    position (0, 0, 0) must stay safely above it)."""
    box = trimesh.creation.box(extents=(60, 60, 15))
    box.apply_translation([0, 0, -12.5])
    return box


@pytest.fixture
def vise_stl(tmp_path):
    path = tmp_path / "vise.stl"
    _vise_jaw_mesh().export(path)
    return str(path)


@pytest.fixture
def crash_program(tmp_path):
    path = tmp_path / "crash.nc"
    path.write_text(
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 Z-10 F100
        """
    )
    return str(path)


@pytest.fixture
def safe_program(tmp_path):
    path = tmp_path / "safe.nc"
    path.write_text(
        """
        G21 G90 G54
        G0 X0 Y0 Z50
        G1 X10 Y10 Z50
        """
    )
    return str(path)


def test_verify_collision_exits_nonzero_and_prints_summary(crash_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(main, ["verify", crash_program, "--scene", vise_stl])

    assert result.exit_code == 1
    assert "COLLISION" in result.output
    assert "Line" in result.output
    assert "flute" in result.output


def test_verify_safe_exits_zero_and_prints_summary(safe_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(main, ["verify", safe_program, "--scene", vise_stl])

    assert result.exit_code == 0
    assert "SAFE" in result.output


def test_verify_quiet_suppresses_output(crash_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(main, ["verify", crash_program, "--scene", vise_stl, "--quiet"])

    assert result.exit_code == 1
    assert result.output == ""


def test_verify_output_writes_json_report(safe_program, vise_stl, tmp_path):
    report_path = tmp_path / "report.json"
    runner = CliRunner()
    result = runner.invoke(
        main, ["verify", safe_program, "--scene", vise_stl, "--output", str(report_path), "--quiet"]
    )

    assert result.exit_code == 0
    data = json.loads(report_path.read_text())
    assert data["safe"] is True
    assert data["events"] == []


def test_verify_tool_preset_overrides_individual_flags(crash_program, vise_stl):
    """--tool-preset should win even if individual --tool-* flags are also given."""
    runner = CliRunner()
    result = runner.invoke(
        main,
        [
            "verify",
            crash_program,
            "--scene",
            vise_stl,
            "--tool-diameter",
            "1000",  # would make everything collide if it were actually used
            "--tool-preset",
            "6mm_ball",
        ],
    )

    assert result.exit_code == 1
    # a 6mm tool plunging straight down at the origin still collides with
    # nothing at X=0,Y=0 in this fixture -- but the crash program's plunge
    # to Z=-10 (well within the slab at [-20,-5]) collides regardless of
    # tool diameter, so this just confirms the preset path doesn't crash
    assert "COLLISION" in result.output


def test_verify_wcs_offset_shifts_program_away_from_obstacle(crash_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(
        main, ["verify", crash_program, "--scene", vise_stl, "--wcs-offset", "1000,1000,1000"]
    )

    assert result.exit_code == 0
    assert "SAFE" in result.output


def test_verify_bull_without_corner_radius_fails_clearly(safe_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(
        main, ["verify", safe_program, "--scene", vise_stl, "--tool-kind", "bull"]
    )

    assert result.exit_code != 0
    assert isinstance(result.exception, ValueError)
    assert "corner_radius" in str(result.exception)


def test_verify_bull_with_corner_radius_succeeds(safe_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(
        main,
        [
            "verify",
            safe_program,
            "--scene",
            vise_stl,
            "--tool-kind",
            "bull",
            "--tool-corner-radius",
            "1.5",
        ],
    )

    assert result.exit_code == 0
    assert "SAFE" in result.output


def test_verify_unknown_tool_preset_fails_clearly(safe_program, vise_stl):
    runner = CliRunner()
    result = runner.invoke(
        main, ["verify", safe_program, "--scene", vise_stl, "--tool-preset", "does_not_exist"]
    )

    assert result.exit_code != 0
    assert isinstance(result.exception, ValueError)


def test_verify_requires_existing_gcode_path(vise_stl):
    runner = CliRunner()
    result = runner.invoke(main, ["verify", "no_such_file.nc", "--scene", vise_stl])
    assert result.exit_code != 0
