"""Tests for the lens cap builder."""

import sys
from pathlib import Path

import cadquery as cq
import pytest

from lens_cap_model.main import build_part, main


def test_build_part_has_cap_dimensions() -> None:
    """The cap spans the outer diameter and the given height."""
    cap = build_part(74.0, 2.0, 20.0).val()
    assert isinstance(cap, cq.Shape)
    box = cap.BoundingBox()

    assert box.xlen == pytest.approx(78.0)
    assert box.ylen == pytest.approx(78.0)
    assert box.zlen == pytest.approx(20.0)


def test_build_part_rejects_thin_wall() -> None:
    """A wall too thin for the rim fillets is refused."""
    with pytest.raises(ValueError, match="wall thickness"):
        build_part(74.0, 0.5, 20.0)


def test_main_exports_step_and_stl(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The command writes a STEP and an STL file named after the diameter."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sys, "argv", ["build-lens-cap", "--id", "60.5"])

    main()

    assert (tmp_path / "slip-on-cap-60_5.step").is_file()
    assert (tmp_path / "slip-on-cap-60_5.stl").is_file()
