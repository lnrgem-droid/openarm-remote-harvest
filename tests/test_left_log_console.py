"""Real Tk controls with all robot/network clients replaced; run under a test display."""
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import left_log_collection_console as ui


@unittest.skipUnless(os.environ.get('DISPLAY'),'needs a test X display')
class LeftConsoleTests(unittest.TestCase):
    def setUp(self):
        self.root=ui.tk.Tk()
        self.root.withdraw()
        self.status=dict(mode='left_only',enabled_arms=['left'],state='RUNNING',fault_bits=0,
            motor_health={'healthy':True},feedback_fresh_for_control=True,
            action_age_ms=2.,feedback_age_ms=3.,leader_session_id=1,
            applied_axes=[0.]*16,left_actual_rad=[0.]*7)
        self.state=dict(phase='idle',running=False,session_id='left_log_fixed',
            session_root='/home/nvidia/datasets/openarm_left_log_sessions',storage_mode='fixed',
            storage_ready=True,camera_health={'ok':True})
        self.backend=Mock();self.backend.snapshot.side_effect=lambda:(self.state,'',False,True)
        self.teleop=Mock();self.teleop.snapshot.side_effect=lambda:(self.status,'',True)
        self.preview=Mock();self.preview.latest.return_value=None;self.preview.age_s.return_value=0.
        with patch.object(ui,'Backend',return_value=self.backend),patch.object(ui,'Teleop',return_value=self.teleop),\
             patch.object(ui.template,'PreviewReceiver',return_value=self.preview),patch.object(ui.App,'ensure_services'):
            self.app=ui.App(self.root,NS(demo=False,screenshot=None,jetson='test',ssh='test',teleop_root=Path('/test')))
        self.root.update_idletasks()

    def tearDown(self):
        for task in self.root.tk.call('after','info'):self.root.after_cancel(task)
        self.root.destroy()

    def test_opening_ui_never_starts_motion_or_recording(self):
        self.backend.request.assert_not_called()
        self.app.update()
        self.assertEqual(str(self.app.buttons['LEFT_PULL_LOG']['state']),'disabled')
        self.app.ready.set(True);self.app.update()
        self.assertEqual(str(self.app.buttons['LEFT_PULL_LOG']['state']),'normal')
        self.app.record('LEFT_PULL_LOG')
        self.backend.request.assert_called_once_with('episode_start',task='LEFT_PULL_LOG')

    def test_fixed_paths_and_next_numbers_are_visible_without_batch_buttons(self):
        self.state['next_episode_by_task']={'left_pull':3,'left_push':8}
        self.app.update()
        for label in ('新建批次','继续上次批次','选择已有批次'):
            self.assertNotIn(label,self.app.buttons)
        self.assertIn('/episodes/left_pull',self.app.task_paths['LEFT_PULL_LOG'].get())
        self.assertIn('/episodes/left_push',self.app.task_paths['LEFT_PUSH_LOG'].get())
        self.assertIn('0003',self.app.buttons['LEFT_PULL_LOG']['text'])
        self.state['next_episode_by_task']['left_pull']=1
        self.app.update()
        self.assertIn('0001',self.app.buttons['LEFT_PULL_LOG']['text'])
        self.assertIn('0008',self.app.buttons['LEFT_PUSH_LOG']['text'])

    def test_old_recorder_cannot_enable_recording_without_fixed_storage(self):
        self.state.pop('storage_mode')
        self.app.ready.set(True);self.app.update()
        self.assertEqual(str(self.app.buttons['LEFT_PULL_LOG']['state']),'disabled')
        self.app.record('LEFT_PULL_LOG');self.backend.request.assert_not_called()

    def test_reused_episode_id_still_clears_ready_and_reports_new_save(self):
        self.state['last_episode']={'episode_id':'left_pull_episode_0001',
            'finalized_unix_s':1.,'result':'success','valid':True}
        self.app.update();self.app.ready.set(True)
        self.state['last_episode']['finalized_unix_s']=2.
        self.state['last_episode']['result']='failure'
        self.app.update()
        self.assertFalse(self.app.ready.get())
        self.assertIn('failure',self.app.message.get())

    def test_fault_disables_both_phase_buttons(self):
        self.app.ready.set(True)
        self.status.update(state='FAULT',fault_bits=2,reason='missing motor feedback')
        self.app.update()
        for task in ui.TASKS:self.assertEqual(str(self.app.buttons[task]['state']),'disabled')
        self.app.record('LEFT_PUSH_LOG');self.backend.request.assert_not_called()
        self.assertIn('missing motor feedback',self.app.message.get())

    def test_user_cancel_cannot_enable_or_disable_motors(self):
        with patch.object(ui.messagebox,'askokcancel',return_value=False),patch.object(ui.subprocess,'run') as run:
            self.app.motion('start');self.app.motion('disable');run.assert_not_called()

    def test_recovery_support_dialog_cancel_does_not_run(self):
        with patch.object(ui.messagebox,'askokcancel',return_value=False),patch.object(ui.subprocess,'run') as run:
            self.app.motion('start',arms_supported=True)
            run.assert_not_called()

    def test_recovery_confirmation_passes_support_flag(self):
        with patch.object(ui.messagebox,'askokcancel',return_value=True),patch.object(ui.subprocess,'run',return_value=NS(returncode=0,stdout='',stderr='')) as run,patch.object(ui.App,'work',side_effect=lambda fn:fn()),patch.object(ui.Path,'write_text'):
            self.app.motion('start',arms_supported=True)
            self.assertIn('--arms-supported',run.call_args.args[0])

    def test_recovery_prompt_waits_for_worker_completion(self):
        self.app.busy=True
        self.app.events.put(('startup_support_required',))
        with patch.object(self.app,'motion') as motion:
            self.app.update();self.root.update_idletasks();motion.assert_not_called()
            self.app.events.put(None)
            self.app.update();self.root.update_idletasks()
            motion.assert_called_once_with('start',arms_supported=True)

    def test_busy_recording_blocks_start_and_disable(self):
        self.state.update(running=True,phase='recording')
        self.app.update()
        for key in ('start','disable','LEFT_PULL_LOG','LEFT_PUSH_LOG'):
            self.assertEqual(str(self.app.buttons[key]['state']),'disabled')
        with patch.object(ui.messagebox,'askokcancel') as dialog,patch.object(ui.subprocess,'run') as run:
            self.app.motion('start');self.app.motion('disable');dialog.assert_not_called();run.assert_not_called()
        self.app.result('failure')
        self.backend.request.assert_called_once_with('episode_stop',result='failure',failure_code='operator_failure')
        self.assertFalse(self.app.ready.get())

if __name__=='__main__':unittest.main()
