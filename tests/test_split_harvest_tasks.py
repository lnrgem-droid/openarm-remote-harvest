"""Offline recording-to-motion contract for the three independent harvest tasks."""
import importlib.util
import json
import sys
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TELEOP = ROOT.parent / "openarm-remote-harvest/ros2_robot/src"
for package in ("remote_teleop_runtime", "remote_teleop_protocol"):
    sys.path.insert(0, str(TELEOP / package))
from remote_teleop_runtime.collection_motion import CollectionMotion

spec = importlib.util.spec_from_file_location("split_task_manager", ROOT / "scripts/jetson_record_manager.py")
manager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager)
Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2
TASKS = [("LEFT_PICK_ONE", "left_pick", "left"),
         ("RIGHT_PULL_LOG", "right_pull", "right"),
         ("RIGHT_PUSH_LOG", "right_push", "right")]


@pytest.fixture
def recorder(tmp_path):
    r = manager.Recorder(tmp_path, runtime_dir=tmp_path / "runtime", storage_root=tmp_path)
    r.active_marker = tmp_path / "recording.active"
    r.error_marker = tmp_path / "recording.error"
    r._free_gb = lambda: 100.
    r._camera_health = lambda: {"ok": True}
    r.start_session("new")
    r.start = lambda **kwargs: {"ok": True}
    return r


def motion_link(r, m, actual=Q):
    def request(command, **fields):
        if command != "status":
            m.command(command, fields, actual, actual, actual, [0.] * 16, 1., True)
        return {"collection": m.status(actual, actual)}
    r._motion_request = request


def seal(r, result="success"):
    active = r.active_episode
    dataset = Path(active["lerobot_root"])
    dataset.mkdir(parents=True)
    (dataset / "rgbd-complete.json").write_text(json.dumps({
        "complete": True, "dataset_root": str(dataset),
        "written": dict.fromkeys(active["camera_roles"], 30), "dropped": {}}))
    active["requested_result"] = result
    r._finalize_episode(0)
    return json.loads((Path(active["episode_root"]) / "episode.json").read_text())


def test_all_tasks_route_to_physical_arm_and_have_independent_durable_counts(recorder, tmp_path):
    r = recorder
    for task, group, side in [TASKS[1], TASKS[0], TASKS[2], TASKS[1]]:
        m = CollectionMotion(tmp_path / "pose.json")
        if side == "right":
            m.command("left_lock", {}, Q, Q, Q, [0.] * 16, 1., True)
        motion_link(r, m)
        response = r.start_episode(task)
        assert response["ok"], response
        assert m.recording["side"] == side
        assert m.recording["task"] == task
        assert r.active_episode["task_group"] == group
        assert r.active_episode["arm_side"] == side
        assert Path(r.active_episode["episode_root"]).parent.name == group
        assert not r.start_episode("RIGHT_PUSH_LOG")["ok"]
        sealed = seal(r)
        assert sealed["valid"] is True
        assert m.recording is None
    assert {key: r.next_episode_by_task[key] for key in ("left_pick", "right_pull", "right_push")} == {
        "left_pick": 2, "right_pull": 3, "right_push": 2}
    stats = r._task_statistics()
    assert stats["left_pick"]["valid_success"] == 1
    assert stats["right_pull"]["valid_success"] == 2
    assert stats["right_push"]["valid_success"] == 1
    restored = manager.Recorder(tmp_path, runtime_dir=tmp_path / "runtime", storage_root=tmp_path)
    assert restored.next_episode_by_task == r.next_episode_by_task
    assert restored._task_statistics() == stats
    counts = restored.session_catalog()["sessions"][0]["episode_counts"]
    assert (counts["left_pick"], counts["right_pull"], counts["right_push"]) == (1, 2, 1)


@pytest.mark.parametrize("task,group,side", TASKS)
def test_pending_episode_recovers_with_task_identity(recorder, tmp_path, task, group, side):
    m = CollectionMotion(tmp_path / "pose.json")
    if side == "right":
        m.command("left_lock", {}, Q, Q, Q, [0.] * 16, 1., True)
    motion_link(recorder, m)
    assert recorder.start_episode(task)["ok"]
    restored = manager.Recorder(tmp_path, runtime_dir=tmp_path / "runtime", storage_root=tmp_path)
    assert restored.phase == "awaiting_result"
    assert restored.active_episode["task"] == task
    assert restored.active_episode["task_group"] == group
    assert restored.active_episode["arm_side"] == side
    assert restored.next_episode_by_task[group] == 1
    assert not restored.start_episode(task)["ok"]


def test_deleted_or_archived_episodes_do_not_recycle_numbers(recorder, tmp_path):
    r = recorder
    motion_link(r, CollectionMotion(tmp_path / "pose.json"))
    for _ in range(3):
        assert r.start_episode("LEFT_PICK_ONE")["ok"]
        seal(r)
    batch = r.session_root
    uid = json.loads((batch / ".episode-sequence.json").read_text())["batch_uid"]
    r.close_session()
    shutil.rmtree(batch / "episodes/left_pick/episode_0002")
    shutil.rmtree(batch / "episodes/left_pick/episode_0003")
    # Archive cleanup may remove metadata entries as well as the directories.
    manifest = json.loads((batch / "session.json").read_text())
    manifest["episodes"] = []
    (batch / "session.json").write_text(json.dumps(manifest))
    restored = manager.Recorder(tmp_path, runtime_dir=tmp_path / "runtime", storage_root=tmp_path)
    restored._load_session(batch)
    assert restored.next_episode_by_task["left_pick"] == 4
    assert (batch / "episodes/left_pick/episode_0001").is_dir()
    assert json.loads((batch / ".episode-sequence.json").read_text())["batch_uid"] == uid
    restored.close_session()
    shutil.rmtree(batch / "episodes/left_pick/episode_0001")
    restored._load_session(batch)
    assert restored.next_episode_by_task["left_pick"] == 4


def test_reserved_number_survives_removed_pending_directory(recorder, tmp_path):
    motion_link(recorder, CollectionMotion(tmp_path / "pose.json"))
    assert recorder.start_episode("LEFT_PICK_ONE")["ok"]
    shutil.rmtree(recorder.active_episode["episode_root"])
    restored = manager.Recorder(tmp_path, runtime_dir=tmp_path / "runtime", storage_root=tmp_path)
    assert restored.active_episode is None
    assert restored.next_episode_by_task["left_pick"] == 2


def test_corrupt_number_ledger_blocks_reopen(recorder):
    root = recorder.session_root
    recorder.close_session()
    (root / ".episode-sequence.json").write_text('{broken')
    with pytest.raises(ValueError):
        recorder._load_session(root)


@pytest.mark.parametrize("task,group,side", TASKS[1:])
@pytest.mark.parametrize("left_hold", [False, True])
def test_log_phases_allow_left_follow_or_hold_and_independent_ready(recorder, tmp_path, task, group, side, left_hold):
    m = CollectionMotion(tmp_path / "pose.json")
    if left_hold:
        m.command("left_lock", {}, Q, Q, Q, [0.] * 16, 1., True)
    left_before = m.left
    displaced = Q.copy(); displaced[14] = .2
    motion_link(recorder, m, displaced)
    m.right = tuple(Q[8:])
    assert not recorder.start_episode(task)["ok"]  # Active right arm must follow.
    m.right = None
    assert recorder.start_episode(task)["ok"]  # Neither left HOLD nor a saved pose is required.
    assert m.left == left_before
    assert m.saved is None
    assert seal(recorder, "failure")["result"] == "failure"
    assert recorder._task_statistics()[group]["failure"] == 1
    assert not recorder.start_episode("RIGHT_PICK_ONE")["ok"]  # Legacy task contract unchanged.


def test_left_pick_preserves_right_log_hold_and_gripper(recorder, tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    m.right = tuple(Q[8:])
    held = m.right
    motion_link(recorder, m)
    assert recorder.start_episode("LEFT_PICK_ONE")["ok"]
    assert m.right == held and m.leader_target is None
    seal(recorder)
    assert m.right == held


def test_old_controller_never_silently_accepts_new_task(recorder):
    calls = []
    def old_controller(command, **fields):
        calls.append(command)
        return {"collection": {}}
    recorder._motion_request = old_controller
    assert not recorder.start_episode("RIGHT_PUSH_LOG")["ok"]
    assert "collection_begin" not in calls
    assert not list((recorder.session_root / "episodes/right_push").iterdir())
    assert recorder.next_episode_by_task["right_push"] == 1


def test_new_tasks_do_not_relabel_legacy_data(recorder, tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    motion_link(recorder, m)
    assert recorder.start_episode("LEFT_GRASP_LOG")["ok"]
    original = seal(recorder)
    metadata = Path(original["episode_root"]) / "episode.json"
    before = metadata.read_bytes()
    assert recorder.start_episode("LEFT_PICK_ONE")["ok"]
    seal(recorder)
    assert metadata.read_bytes() == before
    assert recorder._task_statistics()["left"]["valid_success"] == 1
    assert recorder._task_statistics()["left_pick"]["valid_success"] == 1


@pytest.mark.parametrize("task,side", [("LEFT_PICK_ONE", "right"), ("RIGHT_PULL_LOG", "left"), ("RIGHT_PUSH_LOG", "left")])
def test_motion_rejects_task_arm_mismatch(tmp_path, task, side):
    m = CollectionMotion(tmp_path / "pose.json")
    with pytest.raises(ValueError, match="不匹配"):
        m.command("collection_begin", {"task": task, "side": side, "token": "test"},
                  Q, Q, Q, [0.] * 16, 1., True)
    assert m.recording is None
