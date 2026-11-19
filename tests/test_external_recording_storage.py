"""Removable recording disks must never silently fall back to the system disk."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

spec = importlib.util.spec_from_file_location(
    "external_storage_manager", Path(__file__).parents[1] / "scripts/jetson_record_manager.py")
manager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager)


def configured(tmp_path):
    mount = tmp_path / "usb"
    mount.mkdir()
    repo = tmp_path / "repo"
    (repo / "config").mkdir(parents=True)
    storage = mount / "datasets"
    (repo / "config/recording_storage.json").write_text(json.dumps({
        "storage_root": str(storage), "mountpoint": str(mount), "uuid": "ABCD-1234"}))
    recorder = manager.Recorder(repo, runtime_dir=tmp_path / "runtime")
    recorder.active_marker = tmp_path / "recording.active"
    recorder.camera_status_path = tmp_path / "camera-status.json"
    return recorder, mount, storage


def mounted(target, uuid):
    return SimpleNamespace(stdout=json.dumps({"filesystems": [{"target": str(target), "uuid": uuid}]}))


def test_correct_disk_allows_batch_and_restores_external_session(tmp_path):
    recorder, mount, storage = configured(tmp_path)
    with patch.object(manager.subprocess, "run", return_value=mounted(mount, "ABCD-1234")):
        result = recorder.start_session("new")
        batch = Path(result["session_root"])
        assert storage in batch.parents
        assert (batch / "session.json").exists()
        restored = manager.Recorder(recorder.root, runtime_dir=tmp_path / "runtime")
        assert restored.session_root == batch
        assert set(restored.next_episode_by_task.values()) == {1}


@pytest.mark.parametrize("wrong_disk", [False, True])
def test_missing_or_replaced_disk_refuses_creation_recording_and_metadata(tmp_path, wrong_disk):
    recorder, mount, storage = configured(tmp_path)
    response = mounted(mount if wrong_disk else tmp_path, "OTHER-UUID")
    with patch.object(manager.subprocess, "run", return_value=response):
        assert recorder._free_gb() == 0
        for action in (
            lambda: recorder.start_session("new"),
            lambda: recorder.start_episode("RIGHT_PULL_LOG"),
            lambda: recorder.start(),
            lambda: recorder._write_json(storage / "session.json", {}),
        ):
            with pytest.raises(ValueError, match="采集硬盘未就绪"):
                action()
        assert not storage.exists()


def test_disk_removal_requests_recording_stop(tmp_path):
    recorder, mount, storage = configured(tmp_path)
    recorder._generation = 1
    process = SimpleNamespace(poll=lambda: None)
    with patch.object(manager.subprocess, "run", return_value=mounted(tmp_path, "SYSTEM")), \
            patch.object(recorder, "_request_stop") as stop:
        recorder._monitor(process, storage / "episode/lerobot", 1)
        stop.assert_called_once()
        assert "采集硬盘未就绪" in stop.call_args.args[0]
