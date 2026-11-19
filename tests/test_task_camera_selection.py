"""Exercise real camera writes, task metadata, and two-camera conversion contracts."""
import ast
import importlib.util
import json
import logging
import os
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"
spec = importlib.util.spec_from_file_location("camera_task_manager", SCRIPTS / "jetson_record_manager.py")
manager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager)
ALL = ("left_wrist", "right_wrist", "chest")


def spool_class(marker, error):
    tree = ast.parse((SCRIPTS / "jetson_orbbec_rgbd_service.py").read_text())
    ns = dict(json=json, os=os, queue=queue, threading=threading, time=time,
              Path=Path, LOG=logging.getLogger(__name__), ACTIVE_MARKER=marker, ERROR_MARKER=error)
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)
                                 and n.name == "DepthSpooler"], type_ignores=[]), "spool", "exec"), ns)
    return ns["DepthSpooler"]


@pytest.mark.parametrize("task,expected", [
    ("LEFT_PICK_ONE", ["left_wrist", "chest"]),
    ("RIGHT_PULL_LOG", ["right_wrist", "chest"]),
    ("RIGHT_PUSH_LOG", ["right_wrist", "chest"]),
])
def test_task_through_spool_and_finalization(tmp_path, task, expected):
    r = manager.Recorder(tmp_path, runtime_dir=tmp_path / "runtime", storage_root=tmp_path)
    r.active_marker = tmp_path / "active"
    r.error_marker = tmp_path / "error"
    r._free_gb = lambda: 100
    r._camera_health = lambda: {"ok": True}
    r._acquire_motion_lock = lambda *_: {}
    r._release_motion_lock = lambda: None
    r.start_session("new")
    r.start = lambda **_: {"ok": True}
    assert r.start_episode(task)["ok"]
    assert r.active_episode["camera_roles"] == expected
    dataset = Path(r.active_episode["lerobot_root"])
    dataset.mkdir()
    r.process = SimpleNamespace(poll=lambda: None)
    r._activate_depth_spool(dataset, r._generation)
    assert json.loads((dataset / "capture-config.json").read_text())["camera_roles"] == expected
    spool = spool_class(r.active_marker, r.error_marker)(ALL)
    for sequence in range(32):
        spool.update({role: ({"frame_sequence": sequence}, np.full((2, 2, 3), sequence, np.uint8),
                            np.full((2, 2, 1), 1000 + sequence, np.uint16)) for role in ALL})
        for q in spool.queues.values():
            while not q.empty():
                time.sleep(.001)
    r.active_marker.unlink()
    assert spool._close()
    assert {p.stem for p in (dataset / "rgb_raw").iterdir()} == set(expected)
    assert {p.stem for p in (dataset / "depth_raw").glob("*.u16le")} == set(expected)
    for role in expected:
        assert (dataset / f"depth_raw/{role}.u16le").stat().st_size == 32 * 2 * 2 * 2
    r.active_episode["requested_result"] = "success"
    r._finalize_episode(0)
    assert r.last_episode["valid"] is True
    assert r.last_episode["camera_roles"] == expected
    assert r.last_episode["rgbd_spool_frame_count"] == dict.fromkeys(expected, 32)


def test_same_spool_switches_sides_then_supports_legacy(tmp_path):
    marker, error = tmp_path / "marker", tmp_path / "error"
    spool = spool_class(marker, error)(ALL)
    for i, roles in enumerate((["right_wrist", "chest"], ["left_wrist", "chest"], list(ALL))):
        root = tmp_path / str(i)
        root.mkdir()
        if i < 2:
            (root / "capture-config.json").write_text(json.dumps({"camera_roles": roles}))
        marker.write_text(str(root))
        spool.update({role: ({"frame_sequence": i}, np.zeros((2, 2, 3), np.uint8),
                            np.zeros((2, 2, 1), np.uint16)) for role in ALL})
        marker.unlink()
        assert spool._close()
        receipt = json.loads((root / "rgbd-complete.json").read_text())
        assert receipt["complete"] and set(receipt["written"]) == set(roles)


@pytest.mark.parametrize("roles", [[], ["invalid"], ["chest", "chest"], "chest"])
def test_invalid_camera_config_does_not_save_all_cameras(tmp_path, roles):
    marker, error = tmp_path / "marker", tmp_path / "error"
    (tmp_path / "capture-config.json").write_text(json.dumps({"camera_roles": roles}))
    marker.write_text(str(tmp_path))
    spool = spool_class(marker, error)(ALL)
    spool.update({})
    assert error.exists() and not marker.exists()
    assert not (tmp_path / "rgb_raw").exists()


def test_two_camera_metadata_keeps_bimanual_state_action(tmp_path):
    tree = ast.parse((SCRIPTS / "convert_recording_to_openarm_dataset.py").read_text())
    ns = dict(Path=Path, json=json, ROLES=ALL,
              CAMERA_NAMES={"left_wrist": "wrist_left", "right_wrist": "wrist_right", "chest": "chest"})
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
        and n.name in {"recording_camera_roles", "metadata"}], type_ignores=[]), "converter", "exec"), ns)
    for roles in (["left_wrist", "chest"], ["right_wrist", "chest"]):
        (tmp_path / "capture-config.json").write_text(json.dumps({"camera_roles": roles}))
        selected = ns["recording_camera_roles"](tmp_path)
        value = ns["metadata"]("task", 30., True, camera_roles=selected)
        assert set(value["equipment"]["perceptions"]["cameras"]) == {ns["CAMERA_NAMES"][r] for r in roles}
        assert set(value["frequencies"]["obs"]["arms"]) == {"left", "right"}
    (tmp_path / "capture-config.json").unlink()
    assert ns["recording_camera_roles"](tmp_path) == ALL
