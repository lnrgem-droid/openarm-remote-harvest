#!/usr/bin/env python3
"""Single-left log collection UI using the mushroom console's native Tk visual template."""
import argparse
import fcntl
import json
import queue
import subprocess
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox
from PIL import Image,ImageDraw,ImageTk,ImageOps,ImageGrab
import mushroom_collection_console as template
from left_log_common import ROLES,TITLES,TASKS,ready_reason

template.ROLES=ROLES  # Private UI process; original mushroom launcher remains unchanged.
BG,PANEL,BORDER,TEXT,MUTED=template.BG,template.PANEL,template.BORDER,template.TEXT,template.MUTED
FONT='Noto Sans CJK SC'


class Backend:
    def __init__(self,endpoint):
        self.endpoint=endpoint;self.lock=threading.Lock();self.jobs=queue.Queue();self.stop=threading.Event()
        self.value={};self.error='正在连接录制服务';self.received=0.;self.pending=False;self.connection_error=True
        threading.Thread(target=self.loop,daemon=True).start()
    def request(self,command,**fields):
        with self.lock:
            if self.pending:return False
            self.pending=True
        self.jobs.put({'command':command,**fields});return True
    def loop(self):
        while not self.stop.is_set():
            try: request=self.jobs.get(timeout=.8)
            except queue.Empty:request={'command':'status'}
            try:
                value=template.recorder_request(self.endpoint,request,3000)
                if value.get('profile')!='left_log_v1':raise ValueError('连接的不是单左臂菌棒录制服务')
                with self.lock:
                    self.value=value;self.received=time.monotonic()
                    if request['command']!='status' or not value.get('ok') or self.connection_error:
                        self.error='' if value.get('ok') else str(value.get('error'))
                    self.connection_error=False
            except Exception as exc:
                with self.lock:self.received=0.;self.error=str(exc);self.connection_error=True
            finally:
                if request['command']!='status':
                    with self.lock:self.pending=False
    def snapshot(self):
        with self.lock:return dict(self.value),self.error,self.pending,time.monotonic()-self.received<4


class Teleop:
    def __init__(self,root):
        self.root=root;self.stop=threading.Event();self.lock=threading.Lock();self.value={};self.error='正在读取左臂状态';self.received=0.
        threading.Thread(target=self.loop,daemon=True).start()
    def loop(self):
        while not self.stop.is_set():
            try:
                result=subprocess.run(['/usr/bin/python3',str(self.root/'scripts/left_only_session.py'),'status'],capture_output=True,text=True,timeout=7)
                if result.returncode:raise RuntimeError(result.stdout.strip() or result.stderr.strip())
                value=json.loads(result.stdout)
                with self.lock:self.value=value;self.error='';self.received=time.monotonic()
            except Exception as exc:
                with self.lock:self.value={};self.error=str(exc)[-220:];self.received=0.
            self.stop.wait(.8)
    def snapshot(self):
        with self.lock:return dict(self.value),self.error,time.monotonic()-self.received<3


class App:
    def __init__(self,root,args):
        self.root=root;self.args=args;self.busy=False;self.photos={};self.notice='固定目录已自动准备；检查相机和左臂，确认 READY 后开始录制。'
        self.events=queue.Queue();self.ready=tk.BooleanVar(value=False)
        self.status=tk.StringVar(value='正在读取状态…');self.message=tk.StringVar(value=self.notice)
        self.storage=tk.StringVar(value='正在读取 Jetson 固定保存目录…')
        self.metrics={r:tk.StringVar(value='等待相机') for r in ROLES}
        self.counts={t:tk.StringVar(value='有效成功 0　失败 0　中止 0') for t in TASKS}
        self.task_paths={t:tk.StringVar(value='正在读取保存目录…') for t in TASKS}
        self.buttons={};self.camera_labels={};self.last_episode=None
        self.build()
        if args.demo:
            self.backend=None;self.teleop=None;self.preview=None
        else:
            self.backend=Backend(f'tcp://{args.jetson}:5597');self.teleop=Teleop(args.teleop_root)
            self.preview=template.PreviewReceiver(f'tcp://{args.jetson}:5596');self.preview.start()
            self.ensure_services()
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(100,self.tick)
        if args.screenshot:
            if not args.demo:raise ValueError('screenshot requires --demo')
            root.after(1800,self.screenshot)

    def button(self,parent,text,command,color='#34434c',**pack):
        b=tk.Button(parent,text=text,command=command,bg=color,fg='white',activebackground=color,
                    activeforeground='white',relief='flat',font=(FONT,10,'bold'),padx=12,pady=7)
        b.pack(**pack);return b

    def build(self):
        r=self.root;r.title('OpenArm｜单左臂菌棒采集控制台');r.configure(bg=BG)
        w=min(1500,r.winfo_screenwidth()-40);h=min(940,r.winfo_screenheight()-80)
        r.geometry(f'{w}x{h}+20+35');r.minsize(1050,740)
        top=tk.Frame(r,bg=BG);top.pack(fill='x',padx=22,pady=(12,8))
        tk.Label(top,text='OpenArm｜单左臂菌棒采集',font=(FONT,20,'bold'),bg=BG,fg=TEXT).pack(side='left')
        tk.Label(top,text='仅左臂 · 右臂不参与',font=(FONT,11),bg=BG,fg='#72c94c').pack(side='right')
        tk.Label(r,textvariable=self.status,bg=BG,fg='#79cfe8',font=(FONT,11),anchor='w').pack(fill='x',padx=24)
        row=tk.Frame(r,bg=BG);row.pack(fill='x',padx=22,pady=8)
        for key,title,color in [('start','开始左臂遥操…','#217e49'),('check','仅预检','#34434c'),
                                ('hold','暂停跟随','#866526'),('disable','支撑后失能停止…','#933a35')]:
            self.buttons[key]=self.button(row,title,lambda k=key:self.motion(k),color,side='left',padx=3)
        self.button(row,'连接采集服务',self.ensure_services,side='right',padx=3)
        cams=tk.Frame(r,bg=BG);cams.pack(fill='both',expand=True,padx=22,pady=4)
        for column,role in enumerate(('chest','left_wrist')):
            cams.grid_columnconfigure(column,weight=1,uniform='camera');cams.grid_rowconfigure(0,weight=1)
            card=tk.Frame(cams,bg=PANEL,highlightbackground=BORDER,highlightthickness=1)
            card.grid(row=0,column=column,sticky='nsew',padx=4)
            tk.Label(card,text=template.CAMERA_TITLES[role],font=(FONT,12,'bold'),bg=PANEL,fg=TEXT).pack(anchor='w',padx=12,pady=5)
            label=tk.Label(card,bg='#070a0c',fg=MUTED,text='等待实时画面',font=(FONT,15),width=1,height=1);label.pack(fill='both',expand=True,padx=8)
            self.camera_labels[role]=label
            tk.Label(card,textvariable=self.metrics[role],bg=PANEL,fg=MUTED,font=(FONT,9)).pack(anchor='w',padx=12,pady=4)
        batch=tk.Frame(r,bg=BG);batch.pack(fill='x',padx=24,pady=5)
        tk.Label(batch,text='固定目录 · 无需新建批次 · 编号按现有文件自动接续，删空后从 1 开始',
                 bg=BG,fg=MUTED,font=(FONT,10)).pack(side='left')
        self.button(batch,'退出界面',self.close,side='right')
        tk.Label(r,textvariable=self.storage,bg=BG,fg='#79cfe8',font=(FONT,9),anchor='w',wraplength=1400).pack(fill='x',padx=28)
        ready_row=tk.Frame(r,bg=BG);ready_row.pack(fill='x',padx=28,pady=6)
        tk.Checkbutton(ready_row,text='已将左从臂调至本阶段 READY',variable=self.ready,
                       bg=BG,fg=TEXT,selectcolor=PANEL,activebackground=BG,activeforeground=TEXT,font=(FONT,10)).pack(side='left')
        tasks=tk.Frame(r,bg=BG);tasks.pack(fill='x',padx=22,pady=4)
        for col,task in enumerate(TASKS):
            tasks.grid_columnconfigure(col,weight=1,uniform='tasks')
            color='#72c94c' if col==0 else '#58a9ef'
            card=tk.Frame(tasks,bg=PANEL,highlightbackground=BORDER,highlightthickness=1);card.grid(row=0,column=col,sticky='nsew',padx=4)
            tk.Label(card,text=f'{col+1:02d}  左臂 · {TITLES[task]}',bg=PANEL,fg=color,font=(FONT,15,'bold')).pack(anchor='w',padx=16,pady=(8,2))
            tk.Label(card,textvariable=self.counts[task],bg=PANEL,fg=TEXT,font=(FONT,11)).pack(anchor='w',padx=16)
            tk.Label(card,textvariable=self.task_paths[task],bg=PANEL,fg='#79cfe8',
                     font=(FONT,9),wraplength=650,justify='left').pack(anchor='w',padx=16,pady=3)
            self.buttons[task]=self.button(card,f'开始录制{TITLES[task]}',lambda t=task:self.record(t),color,fill='x',padx=16,pady=8)
        footer=tk.Frame(r,bg=PANEL);footer.pack(fill='x',padx=26,pady=(6,12))
        tk.Label(footer,textvariable=self.message,bg=PANEL,fg='#5be1e6',font=(FONT,10),wraplength=1350,anchor='w').pack(fill='x',padx=12,pady=7)
        controls=tk.Frame(footer,bg=PANEL);controls.pack(fill='x',padx=10,pady=(0,8))
        for result,label,color in [('success','成功并保存','#18883e'),('failure','失败并保存','#a46620'),('aborted','中止本条并保存','#525d62')]:
            self.buttons[result]=self.button(controls,label,lambda v=result:self.result(v),color,side='left',expand=True,fill='x',padx=4)
        tk.Label(footer,text='保存只结束本条录制，遥操继续；关闭界面不会使机械臂失能。',bg=PANEL,fg=MUTED,font=(FONT,9)).pack(pady=(0,7))

    def notify(self,text):self.notice=text;self.message.set(text)
    def backend_state(self):
        return self.backend.snapshot() if self.backend else ({},'',False,False)
    def request(self,command,**fields):
        if self.args.demo:return
        if not self.backend.request(command,**fields):self.notify('上一个操作尚未完成，请稍候。')
        else:self.notify('正在处理…')
    def ensure_services(self):
        if self.args.demo:return
        def work():
            p=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=3',self.args.ssh,
                              'systemctl --user start openarm-left-log-recorder.service openarm-left-log-camera.service'],capture_output=True,text=True,timeout=20)
            if p.returncode:raise RuntimeError(p.stderr or p.stdout)
            return '采集服务已连接；相机画面就绪后可录制。'
        self.work(work)
    def work(self,fn):
        if self.busy:self.notify('上一个操作仍在处理。');return
        self.busy=True
        def run():
            try:self.events.put(str(fn()))
            except Exception as exc:self.events.put('操作失败：'+str(exc))
            finally:self.events.put(None)
        threading.Thread(target=run,daemon=True).start()
    def motion(self,command,arms_supported=False):
        if self.args.demo:return
        state,_,pending,_=self.backend_state()
        if command in ('start','disable') and (state.get('running') or pending):
            self.notify('请先保存或中止本条录制，再操作遥操启动/失能。');return
        flags=[]
        if command=='start':
            if arms_supported:
                if not messagebox.askokcancel('支撑后重新启动','旧会话可能仍带力矩。\n确认左主臂和左从臂均已可靠支撑，且可以先失能停止，再对齐到共同垂落初始位？',parent=self.root):return
            elif not messagebox.askokcancel('启动左主从臂','将使能左主臂和左从臂，缓慢对齐到已保存的共同垂落初始位。\n请确认主从垂落姿态对应、右臂已隔离，活动范围无人。',parent=self.root):return
            flags=['--workspace-clear']
            if arms_supported:flags.append('--arms-supported')
        if command=='disable':
            if not messagebox.askokcancel('支撑后停止','失能后机械臂可能下落。\n确认左主臂和左从臂都已可靠支撑？',parent=self.root):return
            flags=['--arms-supported']
        def run():
            p=subprocess.run(['/usr/bin/python3',str(self.args.teleop_root/'scripts/left_only_session.py'),command,*flags],capture_output=True,text=True)
            log=Path.home()/'.local/state/openarm-left-only/gui-last-operation.log';log.parent.mkdir(parents=True,exist_ok=True);log.write_text(p.stdout+p.stderr)
            if command=='start' and p.returncode==3 and not arms_supported:
                self.events.put(('startup_support_required',));return '旧会话需要支撑后重新启动。'
            if p.returncode:raise RuntimeError((p.stdout+p.stderr)[-900:])
            return {'start':'左臂遥操启动检查通过。','check':'预检通过，尚未启动电机。','hold':'已请求保持；重新跟随需支撑停机后重新启动。','disable':'左主从臂失能停止已确认。'}[command]
        self.work(run)
    def record(self,task):
        if self.args.demo:return
        s,e,live=self.teleop.snapshot();state,_,pending,connected=self.backend_state()
        reason=ready_reason(s) if live else '左臂状态连接不可用'
        if reason or not connected or not (state.get('camera_health') or {}).get('ok'):
            self.notify(reason or '相机或录制服务未就绪。');return
        if state.get('storage_mode') != 'fixed' or not state.get('storage_ready'):
            self.notify('固定保存目录未就绪，请先连接更新后的单左臂录制服务。');return
        if not self.ready.get():self.notify('请先勾选本阶段 READY。');return
        self.request('episode_start',task=task)
    def result(self,result):
        self.ready.set(False)
        self.request('episode_stop',result=result,failure_code='operator_failure' if result=='failure' else '')
    def close(self):
        state,_,pending,_=self.backend_state()
        if self.busy or pending or state.get('running'):
            messagebox.showwarning('尚有操作未完成','请先保存/中止本条，并等待封口完成后再关闭。',parent=self.root);return
        if self.preview:self.preview.stop.set()
        if self.backend:self.backend.stop.set()
        if self.teleop:self.teleop.stop.set()
        self.root.destroy()
    def tick(self):
        try:self.update()
        except Exception as exc:self.notify('界面刷新失败：'+str(exc))
        if self.root.winfo_exists():self.root.after(150,self.tick)
    def update(self):
        while True:
            try:item=self.events.get_nowait()
            except queue.Empty:break
            if item is None:self.busy=False
            elif item == ('startup_support_required',):
                self.pending_start_support = True
            else:self.notify(item)
        if getattr(self,'pending_start_support',False) and not self.busy:
            self.pending_start_support=False
            self.root.after_idle(lambda:self.motion('start',arms_supported=True))
        if self.args.demo:
            s={'state':'RUNNING'};connected=True;pending=False
            state={'phase':'idle','session_id':'left_log_fixed','session_root':'/home/nvidia/datasets/openarm_left_log_sessions','storage_mode':'fixed','storage_ready':True,'camera_health':{'ok':True},'task_statistics':{'left_pull':{'valid_success':12,'failure':1},'left_push':{'valid_success':11,'failure':0}}}
            why='';live=True;images={}
            for role in ROLES:
                image=Image.new('RGB',(640,480),'#1c2930');d=ImageDraw.Draw(image)
                for x in range(0,640,40):d.line((x,0,x,480),fill='#30424a')
                for y in range(0,480,40):d.line((0,y,640,y),fill='#30424a')
                d.rounded_rectangle((200,140,440,340),radius=55,fill='#84755b',outline='#bca17b',width=3)
                images[role]=image;self.metrics[role].set('界面预览 · 模拟画面（未连接设备）')
        else:
            state,error,pending,connected=self.backend.snapshot();s,err,live=self.teleop.snapshot()
            why=ready_reason(s) if live else err
            packet=self.preview.latest();images=packet[0] if packet else {}
            if packet:
                for role in ROLES:self.metrics[role].set(f"{packet[1].get(role+'_fps',0):.1f} FPS · 延迟 {packet[1].get(role+'_age_ms',0):.0f} ms")
            if self.preview.age_s()>2:
                for role in ROLES:self.metrics[role].set('预览未连接或画面已过期')
            if error:self.message.set(error)
        for role,image in images.items():
            label=self.camera_labels[role];w=max(10,label.winfo_width());h=max(10,label.winfo_height())
            photo=ImageTk.PhotoImage(ImageOps.contain(image,(w,h),template.LANCZOS));self.photos[role]=photo;label.configure(image=photo,text='')
        phase=state.get('phase','offline');active=bool(state.get('running'));camera=(state.get('camera_health') or {}).get('ok') is True
        phase_text={'idle':'未录制','starting':'准备录制，请稍候','recording':'正在录制','stopping':'正在封口保存','awaiting_result':'录制中断，等待标记保存','error':'上条录制异常'}.get(phase,'录制服务未连接')
        if not connected:phase_text='录制服务断连，请重连确认状态'
        self.status.set(f"左臂：{'跟随正常' if not why and live else (s.get('state') or '未连接')}　｜　{phase_text}　｜　相机：{'就绪' if camera else '未就绪'}")
        if why and not active:self.message.set('禁止录制：'+why)
        fixed_ready = state.get('storage_mode') == 'fixed' and state.get('storage_ready') is True
        if fixed_ready:
            self.storage.set('固定保存位置：'+str(state.get('session_root', '')))
        elif active:
            self.storage.set('先保存上次未结束的数据：'+str(state.get('session_root', '')))
        else:
            self.storage.set('固定目录未就绪，请确认 Jetson 单左臂录制服务已更新并连接。')
        if phase in ('recording','stopping','awaiting_result'):
            episode=state.get('active_episode') or {};task=episode.get('task','')
            self.message.set(f"{phase_text} · {TITLES.get(task,task)} · {episode.get('episode_id','')}"+((' · '+str(episode.get('capture_interrupted'))) if episode.get('capture_interrupted') else ''))
        saved = state.get('last_episode') or {}
        last=(saved.get('episode_id'), saved.get('finalized_unix_s')) if saved else None
        if last and last!=self.last_episode:
            self.last_episode=last;self.ready.set(False)
            episode=state['last_episode'];self.notify(f"已保存 {episode.get('episode_id')} · 结果 {episode.get('result')} · 数据有效：{'是' if episode.get('valid') else '否，需复核'}")
        for task,group in TASKS.items():
            stats=(state.get('task_statistics') or {}).get(group,{})
            self.counts[task].set(f"有效成功 {stats.get('valid_success',0)}　失败 {stats.get('failure',0)}　中止 {stats.get('aborted',0)}")
            directory=(state.get('task_directories') or {}).get(task)
            if not directory and state.get('session_root'):
                directory=str(Path(state['session_root'])/'episodes'/group)
            number=int((state.get('next_episode_by_task') or {}).get(group,1))
            self.task_paths[task].set('保存到：'+str(directory or '等待服务确认'))
            self.buttons[task].configure(text=f'开始录制{TITLES[task]} · 第 {number:04d} 条')
            enabled=connected and fixed_ready and not active and not pending and not self.busy and camera and live and not why and self.ready.get()
            self.buttons[task].configure(state='normal' if enabled else 'disabled')
        for key in ('success','failure','aborted'):
            enabled=connected and not pending and (phase in ('recording','awaiting_result') or (key=='aborted' and phase=='starting'))
            self.buttons[key].configure(state='normal' if enabled else 'disabled')
        for key in ('start','check','disable'):
            enabled=not self.busy and not active and not pending
            if key=='start' and s.get('state')=='RUNNING':enabled=False
            self.buttons[key].configure(state='normal' if enabled else 'disabled')
        self.buttons['hold'].configure(state='normal' if not self.busy and live and s.get('state')=='RUNNING' else 'disabled')
    def screenshot(self):
        self.root.update_idletasks();x=self.root.winfo_rootx();y=self.root.winfo_rooty()
        ImageGrab.grab(bbox=(x,y,x+self.root.winfo_width(),y+self.root.winfo_height())).save(self.args.screenshot)
        self.root.destroy()


def main():
    p=argparse.ArgumentParser();p.add_argument('--jetson',default='192.168.50.2');p.add_argument('--ssh',default='openarm-jetson')
    default=Path(json.loads((Path.home()/'.local/share/openarm-left-only/desktop.json').read_text())['release_root'])
    p.add_argument('--teleop-root',type=Path,default=default);p.add_argument('--demo',action='store_true');p.add_argument('--screenshot',type=Path)
    args=p.parse_args()
    if args.screenshot and not args.demo:p.error('--screenshot requires --demo')
    root=tk.Tk()
    lock=None
    if not args.demo:
        lock_path=Path.home()/'.local/state/openarm-left-only/collection-ui.lock'
        lock_path.parent.mkdir(parents=True,exist_ok=True);lock=lock_path.open('a')
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            root.withdraw();messagebox.showinfo('单左臂菌棒采集','控制台已经打开，请切换到已有窗口。');root.destroy();return
    App(root,args);root.mainloop()

if __name__=='__main__':main()
