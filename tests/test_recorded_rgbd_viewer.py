"""Offline player validation; synthetic recordings only, no robot connection."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

PATH = Path(__file__).parents[1]/'scripts/view_recorded_rgbd.py'
spec = importlib.util.spec_from_file_location('recorded_rgbd', PATH)
viewer = importlib.util.module_from_spec(spec); spec.loader.exec_module(viewer)


def recording(tmp_path, role='chest', times=(1000000000, 1100000000, 1500000000)):
    root = tmp_path/'episode_0001/lerobot'
    (root/'rgb_raw').mkdir(parents=True, exist_ok=True)
    (root/'depth_raw').mkdir(exist_ok=True)
    rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    depth = np.array([[0, 500, 1000], [1500, 2000, 3000]], dtype='<u2')
    (root/'rgb_raw'/f'{role}.rgb24').write_bytes(rgb.tobytes()*len(times))
    (root/'depth_raw'/f'{role}.u16le').write_bytes(depth.tobytes()*len(times))
    entries = [dict(role=role, aligned_depth_to_rgb=True, host_realtime_ns=t,
                    width=3, height=2, rgb_offset_bytes=i*18, rgb_nbytes=18,
                    offset_bytes=i*12, nbytes=12, depth_scale_mm_per_unit=.5)
               for i,t in enumerate(times)]
    (root/'depth_raw'/f'{role}.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in entries))
    (root/'rgbd-complete.json').write_text('{}')
    return root, entries, rgb, depth


def test_raw_frames_offsets_units_and_source_unchanged(tmp_path):
    root, entries, rgb, depth = recording(tmp_path)
    before = {str(p): p.read_bytes() for p in root.rglob('*') if p.is_file()}
    player = viewer.Recording(root.parent)
    assert np.array_equal(player.read_frame('chest', entries[1]), rgb)
    assert np.array_equal(player.read_frame('chest', entries[2], True), depth)
    colors = viewer.depth_colors(depth, .5, 0, 1.5)
    assert list(colors[0,0]) == [0,0,0]
    assert list(colors[1,2]) == [0,0,255]
    assert player.frame(.09)['images']['chest']['frame'] == 2
    assert player.frame(.39)['images']['chest']['frame'] == 3
    assert player.duration == .5
    assert before == {str(p): p.read_bytes() for p in root.rglob('*') if p.is_file()}


def test_camera_absolute_timestamps_and_missing_time_ranges(tmp_path):
    root, *_ = recording(tmp_path)
    recording(tmp_path, 'left_wrist', (1200000000, 1400000000))
    player = viewer.Recording(root)
    assert 'unavailable' in player.frame(0)['images']['left_wrist']
    assert player.frame(.2)['images']['left_wrist']['frame'] == 1
    assert 'unavailable' in player.frame(.5)['images']['left_wrist']


def test_step_visits_every_camera_frame_on_union_timeline(tmp_path):
    root, *_ = recording(tmp_path)
    recording(tmp_path, 'left_wrist', (1200000000, 1400000000))
    player = viewer.Recording(root)
    assert player.metadata()['timeline'] == [0., .1, .2, .4, .5]
    seen = {role: set() for role in player.cameras}
    seconds = 0.
    for _ in player.timeline:
        for role, item in player.frame(seconds)['images'].items():
            if 'frame' in item: seen[role].add(item['frame'])
        seconds = viewer.neighbor_time(player.timeline, seconds, 1)
    assert seen == {'chest': {1, 2, 3}, 'left_wrist': {1, 2}}
    assert seconds == .5
    assert viewer.neighbor_time(player.timeline, 0., -1) == 0.
    assert viewer.neighbor_time(player.timeline, .3, -1) == .2
    assert viewer.neighbor_time(player.timeline, .3, 1) == .4


def test_truncated_frame_rejected_and_incomplete_index_reported(tmp_path):
    root, *_ = recording(tmp_path)
    with (root/'depth_raw/chest.jsonl').open('a') as f: f.write('{bad')
    player = viewer.Recording(root)
    assert player.metadata()['warnings']
    (root/'depth_raw/chest.u16le').write_bytes(b'\x00')
    with pytest.raises(ValueError, match='不完整'): player.frame(0)


@pytest.mark.parametrize('near,far', [(1,1), (2,1), (-1,2), (0,float('nan'))])
def test_bad_depth_range_rejected(near,far):
    with pytest.raises(ValueError): viewer.depth_colors(np.zeros((2,2)),1,near,far)


def test_discovery_and_worker_protocol(tmp_path):
    from argparse import Namespace
    root, *_ = recording(tmp_path)
    (root.parent/'episode.json').write_text(json.dumps({'task':'RIGHT_PUSH_LOG','result':'success'}))
    found=viewer.discover(tmp_path)
    assert len(found)==1 and found[0]['task']=='RIGHT_PUSH_LOG'
    client=viewer.Transport(Namespace(ssh=None))
    try:
        assert len(client.exchange({'action':'list','root':str(tmp_path)})['recordings'])==1
        assert client.exchange({'action':'open','path':str(root)})['roles']==['chest']
        assert client.exchange({'action':'frame','time':0})['images']['chest']['rgb']
        with pytest.raises(ValueError): client.exchange({'action':'open','path':str(tmp_path/'missing')})
    finally: client.close()
