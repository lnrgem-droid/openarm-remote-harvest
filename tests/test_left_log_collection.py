"""Single-left gates and real recorder subprocess on synthetic ROS/status input only."""
import copy
import importlib.util
import json
import shlex
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import left_log_common as common
import left_log_record_manager as manager
import record_left_log as capture


def healthy():
    return dict(mode='left_only',enabled_arms=['left'],state='RUNNING',fault_bits=0,
                motor_health={'healthy':True},feedback_fresh_for_control=True,
                action_age_ms=2.,feedback_age_ms=3.,leader_session_id=123,
                applied_axes=[.1]*7+[-.5236]+[0.]*8,left_actual_rad=[.2]*7)


class ContractTests(unittest.TestCase):
    def test_health_rejects_fault_stale_or_wrong_arm(self):
        self.assertEqual(common.ready_reason(healthy()),'')
        for fields in ({'enabled_arms':['left','right']},{'state':'FAULT'},
                       {'fault_bits':4},{'feedback_age_ms':101},{'action_age_ms':float('nan')},
                       {'motor_health':{'healthy':False}},{'applied_axes':[0]*8},
                       {'leader_session_id':0},{'feedback_fresh_for_control':False}):
            with self.subTest(fields=fields):self.assertTrue(common.ready_reason(healthy()|fields))

    def test_real_feedback_and_applied_action_have_same_gripper_units(self):
        msg=NS(header=NS(stamp=NS(sec=1,nanosec=0)),
               name=[f'openarm_left_joint{i}' for i in range(1,8)]+['openarm_left_finger_joint1'],
               position=[.2]*7+[.022])
        self.assertEqual(capture.joint_vector(msg,1_010_000_000),[.2]*7+[.5])
        self.assertEqual(common.action_vector(healthy()),[.1]*7+[.5])
        with self.assertRaises(ValueError):capture.joint_vector(msg,1_100_000_001)
        msg.name.pop()
        with self.assertRaises(KeyError):capture.joint_vector(msg,1_010_000_000)

    def recorder(self,root):
        r=manager.LeftLogRecorder(root,runtime_dir=root/'runtime',storage_root=root/'datasets')
        r.active_marker=root/'active';r.error_marker=root/'error';r.camera_status_path=root/'cameras.json'
        r._free_gb=lambda:100.
        r.camera_status_path.write_text(json.dumps({'updated_unix_s':time.time(),
          'cameras':{role:{'healthy':True} for role in common.ROLES}}))
        return r

    def test_two_camera_contract_and_separate_task_directories(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.recorder(Path(d));self.assertTrue(r._camera_health()['ok'])
            self.assertTrue(r.start_session('new')['ok'])
            info=json.loads((r.session_root/'session.json').read_text())
            self.assertEqual(info['active_arms'],['left']);self.assertEqual(info['vector_size'],8)
            self.assertEqual(set(p.name for p in (r.session_root/'episodes').iterdir()),{'left_pull','left_push'})
            self.assertFalse(r.start_episode('RIGHT_PULL_LOG','x')['ok'])

    def write_sealed(self,r,group,number):
        directory=r.session_root/'episodes'/group/f'episode_{number:04d}'
        directory.mkdir(parents=True)
        task=next(task for task,g in common.TASKS.items() if g==group)
        (directory/'episode.json').write_text(json.dumps(dict(
            episode_id=f'{group}_episode_{number:04d}',episode_number=number,
            episode_root=str(directory),task=task,task_group=group,result='success',valid=True)))
        return directory

    def test_fixed_storage_reopens_without_new_batches_and_reserves_incomplete_dirs(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);r=self.recorder(root);r.prepare_storage()
            fixed=r.session_root
            self.assertEqual(fixed,root/'datasets')
            self.write_sealed(r,'left_pull',2)
            (fixed/'episodes/left_push/episode_0004').mkdir()
            r.prepare_storage();r.start_session('new')
            self.assertEqual(r.session_root,fixed)
            self.assertEqual(r.status()['next_episode_by_task'],{'left_pull':3,'left_push':5})
            restored=self.recorder(root);restored.prepare_storage()
            self.assertEqual(restored.session_root,fixed)
            self.assertEqual(restored.status()['next_episode_by_task'],{'left_pull':3,'left_push':5})
            self.assertFalse(list(fixed.glob('left_log_*')))

    def test_delete_all_resets_only_that_task_and_actual_next_record_uses_one(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);r=self.recorder(root);r.prepare_storage()
            a=self.write_sealed(r,'left_pull',1);b=self.write_sealed(r,'left_pull',2)
            other=self.write_sealed(r,'left_push',2)
            self.assertEqual(r.status()['next_episode_by_task']['left_pull'],3)
            shutil.rmtree(a);shutil.rmtree(b)
            self.assertEqual(r.status()['next_episode_by_task'],{'left_pull':1,'left_push':3})
            self.assertEqual(r.status()['task_statistics']['left_pull']['valid_success'],0)
            captured={}
            r.start=lambda **kw:captured.update(kw) or {'ok':True}
            with patch.object(manager,'runtime_status',return_value=healthy()):
                self.assertTrue(r.start_episode('LEFT_PULL_LOG','LOG-001')['ok'])
            self.assertEqual(r.active_episode['episode_number'],1)
            self.assertIn('left_pull/episode_0001/lerobot',captured['dataset_root'])
            self.assertTrue(other.is_dir())
            # Deleting an active directory must never reset its ownership or allow a second start.
            shutil.rmtree(Path(r.active_episode['episode_root']))
            self.assertTrue(r.status()['running'])
            self.assertFalse(r.start_episode('LEFT_PULL_LOG','LOG-002')['ok'])

    def test_deleting_lower_numbers_does_not_overwrite_remaining_data(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.recorder(Path(d));r.prepare_storage()
            a=self.write_sealed(r,'left_pull',1);self.write_sealed(r,'left_pull',3)
            shutil.rmtree(a)
            self.assertEqual(r.status()['next_episode_by_task']['left_pull'],4)

    def test_record_without_session_click_prepares_fixed_storage(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.recorder(Path(d));r.start=lambda **kw:{'ok':True}
            with patch.object(manager,'runtime_status',return_value=healthy()):
                result=r.start_episode('LEFT_PUSH_LOG')
            self.assertTrue(result['ok'])
            self.assertEqual(r.session_root,Path(d)/'datasets')
            self.assertEqual(r.active_episode['task_group'],'left_push')

    def test_pending_episode_survives_restart_without_number_reset(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);r=self.recorder(root);r.start=lambda **kw:{'ok':True}
            with patch.object(manager,'runtime_status',return_value=healthy()):r.start_episode('LEFT_PULL_LOG','LOG-001')
            restored=self.recorder(root);restored.prepare_storage()
            self.assertEqual(restored.phase,'awaiting_result')
            self.assertEqual(restored.active_episode['episode_number'],1)
            self.assertTrue(restored.status()['running'])
            self.assertFalse(restored.start_episode('LEFT_PUSH_LOG','LOG-002')['ok'])

    def test_old_pending_batch_is_finished_before_switching_to_fixed_storage(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);r=self.recorder(root)
            manager.base.Recorder.start_session(r,'new')
            old_root=r.session_root;r.start=lambda **kw:{'ok':True}
            with patch.object(manager,'runtime_status',return_value=healthy()):
                manager.base.Recorder.start_episode(r,'LEFT_PULL_LOG','OLD-001')
            pending=Path(r.active_episode['episode_root'])
            restored=self.recorder(root);restored.prepare_storage()
            self.assertEqual(restored.session_root,old_root)
            self.assertEqual(restored.phase,'awaiting_result')
            self.assertTrue(restored.stop_episode('aborted')['ok'])
            self.assertTrue((pending/'episode.json').is_file())
            restored.prepare_storage()
            self.assertEqual(restored.session_root,root/'datasets')
            self.assertTrue((pending/'episode.json').is_file())
            self.assertEqual(restored.status()['next_episode_by_task'],{'left_pull':1,'left_push':1})

    def test_recorder_never_enables_motor_and_rejects_changed_leader(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.recorder(Path(d))
            with patch.object(manager,'runtime_status',return_value=healthy()) as rpc:
                r._acquire_motion_lock('left_pull','token');rpc.assert_called_once_with()
            self.assertTrue(r._recording_motion_valid(healthy()))
            self.assertFalse(r._recording_motion_valid(healthy()|{'leader_session_id':124}))
            for command in ('enable','start','hold','collection_begin'):
                with self.assertRaises(RuntimeError):r._motion_request(command)

    def test_fault_does_not_claim_episode_number(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.recorder(Path(d));r.start_session('new')
            with patch.object(manager,'runtime_status',return_value=healthy()|{'state':'FAULT'}):
                response=r.start_episode('LEFT_PULL_LOG','TEST')
            self.assertFalse(response['ok']);self.assertIsNone(r.active_episode)
            self.assertEqual(r.next_episode_by_task,{'left_pull':1,'left_push':1})
            self.assertEqual(list((r.session_root/'episodes/left_pull').iterdir()),[])

    def test_synthetic_record_save_and_interruption(self):
        import pyarrow.parquet as pq
        for task,result,fault in [('LEFT_PULL_LOG','success',False),('LEFT_PUSH_LOG','failure',False),
                                  ('LEFT_PULL_LOG','success',True)]:
            with self.subTest(task=task,result=result,fault=fault),tempfile.TemporaryDirectory() as d:
                root=Path(d);r=self.recorder(root);r._camera_health=lambda:{'ok':True}
                script=root/'synthetic.py'
                # Exercise production sampler main and ParquetWriter, but replace
                # both inputs before main imports ROS. Never accesses ROS/CAN/socket.
                script.write_text('''import sys,time,types
sys.path.insert(0,SCRIPT_DIR)
import record_left_log as recorder
started=time.monotonic()
class Node:
    def __init__(self,*a):pass
    def create_subscription(self,t,topic,callback,qos):self.callback=callback
    def destroy_node(self):pass
def spin(node,timeout_sec):
    time.sleep(timeout_sec)
    now=time.time_ns(); elapsed=time.monotonic()-started
    node.callback(types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=now//1000000000,nanosec=now%1000000000)),name=[f'openarm_left_joint{i}' for i in range(1,8)]+['openarm_left_finger_joint1'],position=[elapsed*.01]*7+[.022]))
sys.modules['rclpy']=types.SimpleNamespace(init=lambda:None,shutdown=lambda:None,spin_once=spin)
sys.modules['rclpy.node']=types.SimpleNamespace(Node=Node)
sys.modules['sensor_msgs']=types.ModuleType('sensor_msgs')
sys.modules['sensor_msgs.msg']=types.SimpleNamespace(JointState=object)
def status(*a):
    s=HEALTH.copy()
    if FAIL and time.monotonic()-started>1.3:s['state']='FAULT'
    return s
recorder.runtime_status=status
raise SystemExit(recorder.main())
'''.replace('SCRIPT_DIR',repr(str(ROOT/'scripts'))).replace('HEALTH',repr(healthy())).replace('FAIL',repr(fault)))
                r._recorder_command=lambda:[sys.executable,str(script),'--root',str(r.dataset_root),'--task',task]
                r._motion_request=lambda *a,**k:healthy()
                with patch.object(manager,'runtime_status',return_value=healthy()):
                    r.start_session('new');self.assertTrue(r.start_episode(task)['ok'])
                def wait(predicate):
                    end=time.monotonic()+10
                    while time.monotonic()<end:
                        if predicate():return
                        time.sleep(.03)
                    self.fail(f'timeout: {r.phase} {r.last_log}')
                try:
                    wait(lambda:r.phase=='recording')
                    episode=dict(r.active_episode);dataset=Path(episode['lerobot_root'])
                    (dataset/'rgbd-complete.json').write_text(json.dumps(dict(dataset_root=str(dataset),complete=True,
                        written={'left_wrist':90,'chest':90},dropped={})))
                    if fault:
                        wait(lambda:r.phase=='awaiting_result')
                        self.assertEqual(r.next_episode_by_task[common.TASKS[task]],1)
                    else:time.sleep(1.4)
                    self.assertTrue(r.stop_episode(result)['ok']);wait(lambda:not r.status()['running'])
                    self.assertEqual(r.last_episode['valid'],result=='success' and not fault)
                    self.assertEqual(r.last_episode['arm_side'],'left')
                    self.assertEqual(r.next_episode_by_task[common.TASKS[task]],2)
                    table=pq.read_table(dataset/'data/chunk-000/file-000.parquet')
                    self.assertGreater(table.num_rows,15)
                    states=table['observation.state'].to_pylist();actions=table['action'].to_pylist()
                    self.assertTrue(all(len(v)==8 for v in states+actions))
                    self.assertGreater(states[-1][0],states[0][0]);self.assertAlmostEqual(states[-1][7],.5)
                    info=json.loads((dataset/'meta/info.json').read_text())
                    self.assertEqual(info['camera_roles'],['left_wrist','chest'])
                    self.assertEqual(bool(info['recording_error']),fault)
                    r.stop_episode(result);self.assertEqual(r.next_episode_by_task[common.TASKS[task]],2)
                finally:
                    if r.process and r.process.poll() is None:r.process.kill();r.process.wait(timeout=3)


class ConversionTests(unittest.TestCase):
    def test_eight_dimensions_convert_to_left_only_with_two_cameras(self):
        import numpy as np
        import pyarrow as pa
        import pyarrow.parquet as pq
        import yaml
        import convert_recording_to_openarm_dataset as converter
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'source';output=Path(d)/'converted'
            for path in ('meta','data/chunk-000','depth_raw','rgb_raw'):(source/path).mkdir(parents=True)
            origin=1_790_000_000_000_000_000
            (source/'meta/info.json').write_text(json.dumps(dict(format='openarm_left_rgbd_staging_v1',
                active_arms=['left'],vector_size=8,started_unix_ns=origin,task='LEFT_PULL_LOG')))
            pq.write_table(pa.table({'timestamp':[0.], 'observation.state':[[.1]*7+[.5]],'action':[[.2]*7+[.5]]}),source/'data/chunk-000/file-000.parquet')
            for role in common.ROLES:
                (source/f'rgb_raw/{role}.rgb24').write_bytes(np.zeros((2,2,3),dtype=np.uint8).tobytes())
                (source/f'depth_raw/{role}.jsonl').write_text(json.dumps(dict(role=role,aligned_depth_to_rgb=True,
                    host_realtime_ns=origin,height=2,width=2,rgb_offset_bytes=0,rgb_nbytes=12))+'\n')
            with patch.object(sys,'argv',['convert',str(source),str(output)]):converter.main()
            self.assertFalse((output/'episodes/0/obs/arms/right').exists())
            table=pq.read_table(output/'episodes/0/obs/arms/left/state.parquet')
            self.assertEqual(len(table['qpos'].to_pylist()[0]),8)
            meta=yaml.safe_load((output/'metadata.yaml').read_text())
            self.assertEqual(meta['frequencies']['obs']['arms'],{'left':30.})
            self.assertEqual(set(meta['frequencies']['cameras']),{'wrist_left','chest'})
            self.assertEqual(meta['tasks'][0]['prompt'],'LEFT_PULL_LOG')
            with patch.object(sys,'argv',['convert',str(source),str(output)]),self.assertRaises(SystemExit):converter.main()

if __name__=='__main__':unittest.main()
