from pathlib import Path


def test_converter_uses_openarm_bridge_left_then_right_order():
    source = (Path(__file__).parents[1] / "scripts" /
              "convert_recording_to_openarm_dataset.py").read_text(encoding="utf-8")

    assert '("left", vector[:, :8])' in source
    assert '("right", vector[:, 8:])' in source
    assert '("right", vector[:, :8])' not in source
