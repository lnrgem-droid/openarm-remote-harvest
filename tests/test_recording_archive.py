"""Exercise archive deletion guards with disposable recordings, never robot data."""
import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT=Path(__file__).parents[1]/'scripts/archive_recordings.py'
spec=importlib.util.spec_from_file_location('archive_recordings',SCRIPT)
archive=importlib.util.module_from_spec(spec);spec.loader.exec_module(archive)


@pytest.fixture
def recording(tmp_path,monkeypatch):
    data=tmp_path/'datasets';source=data/'batch';disk=tmp_path/'disk';disk.mkdir()
    episode=source/'episodes/right_pull/episode_0007'
    (episode/'lerobot/rgb_raw').mkdir(parents=True)
    (episode/'episode.json').write_text(json.dumps({'result':'success','episode_number':7}))
    (episode/'lerobot/rgbd-complete.json').write_text('{"complete":true}')
    (episode/'lerobot/rgb_raw/chest.rgb24').write_bytes(bytes(range(256))*1024)
    (source/'session.json').write_text(json.dumps({'session_ended_unix_s':123,'episodes':[{'episode_number':7}]}))
    monkeypatch.setattr(archive,'DATA_ROOT',data)
    monkeypatch.setattr(archive,'recorder_state',lambda:{'ok':True,'running':False,'active_episode':None,'session_root':None})
    # A temporary filesystem substitutes for the USB mount only; copy and hashes are real.
    monkeypatch.setattr(archive,'check_disk',lambda disk,source,expected_device=None:disk.stat().st_dev)
    return source,disk,episode


def test_verified_archive_retains_names_and_high_water_mark(recording):
    source,disk,episode=recording
    original=(episode/'lerobot/rgb_raw/chest.rgb24').read_bytes()
    result=archive.archive_batch(source,disk)
    target=Path(result['archive'])
    assert (target/'episodes/right_pull/episode_0007/lerobot/rgb_raw/chest.rgb24').read_bytes()==original
    assert not episode.exists()
    ledger=json.loads((source/'.episode-sequence.json').read_text())
    assert ledger['next_episode_by_task']['right_pull']==8
    assert json.loads((source/'session.json').read_text())['episodes']==[]
    manifest=json.loads((target/'archive-manifest.json').read_text())
    assert all(archive.digest(target/name)==value['sha256'] for name,value in manifest['files'].items())


def test_corrupt_destination_never_deletes_source(recording,monkeypatch):
    source,disk,episode=recording
    monkeypatch.setattr(archive,'digest',lambda path:'incorrect checksum')
    with pytest.raises(RuntimeError,match='校验失败'):archive.archive_batch(source,disk)
    assert (episode/'lerobot/rgb_raw/chest.rgb24').is_file()
    assert not list(disk.rglob('archive-manifest.json'))


def test_source_mutation_during_copy_never_deletes_source(recording):
    source,disk,episode=recording
    def mutate(_):
        (episode/'added-during-copy.txt').write_text('new file')
    with pytest.raises(RuntimeError,match='发生变化'):archive.archive_batch(source,disk,mutate)
    assert (episode/'added-during-copy.txt').is_file()


def test_unfinished_recording_is_preserved(recording):
    source,disk,episode=recording
    (episode/'lerobot/rgbd-complete.json').write_text('{"complete":false}')
    with pytest.raises(ValueError,match='未写完'):archive.archive_batch(source,disk)
    assert episode.is_dir()


def test_reopened_session_during_detach_restores_source(recording,monkeypatch):
    source,disk,episode=recording
    def status():
        return {'ok':True,'running':False,'active_episode':None,
                'session_root':str(source) if not (source/'episodes').exists() else None}
    monkeypatch.setattr(archive,'recorder_state',status)
    with pytest.raises(RuntimeError,match='打开状态'):archive.archive_batch(source,disk)
    assert (episode/'lerobot/rgb_raw/chest.rgb24').is_file()
    assert list(disk.rglob('archive-manifest.json'))


def test_second_archive_keeps_batch_identity_and_separate_transfer(recording):
    source,disk,episode=recording
    first=archive.archive_batch(source,disk)
    next_episode=episode.with_name('episode_0008')
    (next_episode/'lerobot').mkdir(parents=True)
    (next_episode/'episode.json').write_text('{"result":"success","episode_number":8}')
    (next_episode/'lerobot/rgbd-complete.json').write_text('{"complete":true}')
    second=archive.archive_batch(source,disk)
    assert Path(first['archive']).parent==Path(second['archive']).parent
    assert first['archive']!=second['archive']
    assert Path(first['archive']).exists()
    assert second['next_episode_by_task']['right_pull']==9


def test_active_recorder_blocks_all_copy(recording,monkeypatch):
    source,disk,episode=recording
    monkeypatch.setattr(archive,'recorder_state',lambda:{'ok':True,'running':True})
    with pytest.raises(RuntimeError,match='停止录制'):archive.archive_batch(source,disk)
    assert not list(disk.iterdir())
    assert episode.exists()


def test_same_filesystem_rejected(tmp_path):
    with pytest.raises(ValueError,match='同一个分区'):
        archive.check_disk(Path('/'),tmp_path)
