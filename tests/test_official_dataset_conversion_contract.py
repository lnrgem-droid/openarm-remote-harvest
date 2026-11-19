from pathlib import Path


def test_converter_uses_openarm_bridge_left_then_right_order():
    source = (Path(__file__).parents[1] / "scripts" /
              "convert_recording_to_openarm_dataset.py").read_text(encoding="utf-8")

    assert 'sides = ("left",) if left_only else ("left", "right")' in source
    assert 'for index, side in enumerate(sides):' in source
    assert 'values = vector[:, index * 8:(index + 1) * 8]' in source
