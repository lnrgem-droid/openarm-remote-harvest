#!/usr/bin/env python3
"""Read ROS joint state + applied left action at 30 Hz. No CAN or ROS publishers."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import termios
import time
import tty
from left_log_common import runtime_status,action_vector,finite_vector,TASKS


def joint_vector(message,now_ns):
    stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
    if not 0<=now_ns-stamp<100_000_000: raise ValueError('left ROS source timestamp stale')
    q=dict(zip(message.name,message.position))
    values=[q[f'openarm_left_joint{i}'] for i in range(1,8)]+[q['openarm_left_finger_joint1']]
    if not finite_vector(values,8): raise ValueError('left ROS feedback incomplete')
    values[7]=max(0.,min(1.,values[7]/.044))
    return values


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import pyarrow as pa
    import pyarrow.parquet as pq
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--task',choices=TASKS,required=True)
    a=p.parse_args()
    if a.root.exists(): raise RuntimeError('refusing to overwrite dataset')
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    if os.isatty(0): tty.setcbreak(0,termios.TCSANOW)
    rclpy.init(); node=Node('left_log_read_only_recorder')
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    latest=None; received=0.
    def receive(message):
        nonlocal latest,received
        latest=message;received=time.monotonic()
    node.create_subscription(JointState,'/follower/joint_states',receive,1)
    schema=pa.schema([('timestamp',pa.float64()),('observation.state',pa.list_(pa.float32(),8)),
                      ('action',pa.list_(pa.float32(),8)),('frame_index',pa.int64()),
                      ('episode_index',pa.int64()),('task_index',pa.int64()),('index',pa.int64()),
                      ('host_realtime_ns',pa.int64()),('action_age_ms',pa.float64()),('feedback_age_ms',pa.float64())])
    writer=None;rows=[];count=0;origin=None;origin_mono=None;session=None;failure=None;duration=0.;last_sample=None
    deadline=time.monotonic()+20;next_at=time.monotonic()
    try:
        while not stopped:
            rclpy.spin_once(node,timeout_sec=.003)
            if select.select([0],[],[],0)[0] and b'q' in os.read(0,64).lower(): break
            now=time.monotonic()
            if now<next_at: continue
            if last_sample is not None and now-last_sample>.15:
                raise RuntimeError('left recording sample gap exceeded 150 ms')
            next_at=max(next_at+1/30,now)
            try:
                if latest is None or now-received>.1: raise ValueError('left ROS feedback missing')
                state=joint_vector(latest,time.time_ns())
                status=runtime_status(.08);action=action_vector(status)
                if time.monotonic()-received>.1: raise ValueError('left ROS feedback stale during status read')
                if session is not None and status['leader_session_id']!=session: raise ValueError('leader session changed')
            except (OSError,ValueError,KeyError) as exc:
                if writer is not None or time.monotonic()>deadline: raise RuntimeError(str(exc)) from exc
                continue
            if writer is None:
                a.root.mkdir(parents=True,exist_ok=False)
                (a.root/'data/chunk-000').mkdir(parents=True);(a.root/'meta').mkdir()
                writer=pq.ParquetWriter(a.root/'data/chunk-000/file-000.parquet',schema,compression='zstd')
                origin=time.time_ns();origin_mono=time.monotonic();session=status['leader_session_id']
            now=time.monotonic();duration=now-origin_mono
            rows.append({'timestamp':duration,'observation.state':state,'action':action,
                         'frame_index':count,'episode_index':0,'task_index':0,'index':count,
                         'host_realtime_ns':time.time_ns(),'action_age_ms':status['action_age_ms'],
                         'feedback_age_ms':status['feedback_age_ms']})
            count+=1
            last_sample=now
            if len(rows)>=300: writer.write_table(pa.Table.from_pylist(rows,schema));rows=[]
    except Exception as exc: failure=f'{type(exc).__name__}: {exc}';print(failure,flush=True)
    finally:
        if writer:
            if count<30 and failure is None:failure='fewer than 30 robot samples; episode needs review'
            if rows: writer.write_table(pa.Table.from_pylist(rows,schema))
            writer.close()
            info={'format':'openarm_left_rgbd_staging_v1','active_arms':['left'],'vector_size':8,
                  'vector_order':'left_joint1..7,left_gripper','gripper_unit':'opening_fraction_0_to_1',
                  'joint_unit':'radian','camera_roles':['left_wrist','chest'],'fps':30,'task':a.task,
                  'total_frames':count,'started_unix_ns':origin,'duration_s':duration,
                  'leader_session_id':session,'recording_error':failure}
            (a.root/'meta/info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 3 if failure else (0 if count else 2)

if __name__=='__main__': raise SystemExit(main())
