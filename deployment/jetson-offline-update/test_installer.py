"""Hardware-free tests for offline installer failure and archive handling."""
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location("installer", Path(__file__).with_name("install_on_jetson.py"))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
client_spec = importlib.util.spec_from_file_location(
    "update_client", Path(__file__).with_name("update-jetson.py"))
client = importlib.util.module_from_spec(client_spec)
client_spec.loader.exec_module(client)


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.job = self.root / "job"
        self.job.mkdir()

    def tearDown(self):
        self.temp.cleanup()

    def fixture(self):
        dest, candidate = self.root / "old", self.root / "new"
        dest.mkdir()
        (dest / "value").write_text("old")
        candidate.mkdir()
        (candidate / "value").write_text("new")
        return dest, candidate

    def test_success_keeps_backup(self):
        dest, candidate = self.fixture()
        tx = m.Transaction(self.job)
        tx.replace(dest, candidate)
        tx.commit()
        self.assertEqual((dest / "value").read_text(), "new")
        self.assertEqual((self.job / "backup/0/value").read_text(), "old")

    def test_rollback_restores_original(self):
        dest, candidate = self.fixture()
        tx = m.Transaction(self.job)
        tx.replace(dest, candidate)
        tx.rollback()
        self.assertEqual((dest / "value").read_text(), "old")

    def test_failure_between_two_renames(self):
        dest, candidate = self.fixture()
        tx = m.Transaction(self.job)
        original = m.os.replace

        def replace(a, b):
            if Path(a) == candidate:
                raise OSError("injected failure")
            return original(a, b)

        with mock.patch.object(m.os, "replace", side_effect=replace):
            with self.assertRaises(OSError):
                tx.replace(dest, candidate)
        m.Transaction(self.job).load().rollback()
        self.assertEqual((dest / "value").read_text(), "old")

    def test_rollback_new_path(self):
        dest, candidate = self.fixture()
        m.remove(dest)
        tx = m.Transaction(self.job)
        tx.replace(dest, candidate)
        tx.rollback()
        self.assertFalse(dest.exists())

    def test_rollback_symlink_does_not_modify_target(self):
        original = self.root / "original"
        original.mkdir()
        (original / "value").write_text("original")
        dest = self.root / "link"
        dest.symlink_to(original)
        candidate = self.root / "new-link"
        candidate.symlink_to(self.root / "new-target")
        tx = m.Transaction(self.job)
        tx.replace(dest, candidate)
        tx.rollback()
        self.assertEqual(dest.resolve(), original)
        self.assertEqual((dest / "value").read_text(), "original")

    def test_restart_recovers_uncommitted_transaction(self):
        dest, candidate = self.fixture()
        job = self.root / "job-123"
        job.mkdir()
        tx = m.Transaction(job)
        tx.replace(dest, candidate)
        with mock.patch.object(m, "STATE", self.root), mock.patch.object(m, "require_idle"):
            m.recover_incomplete()
        self.assertEqual((dest / "value").read_text(), "old")
        self.assertEqual(json.loads(tx.journal.read_text())["status"], "rolled_back")

    def test_idle_guard_blocks(self):
        with mock.patch.object(m, "active_processes", return_value=[{"pid": 123}]):
            with self.assertRaisesRegex(RuntimeError, "检测到"):
                m.require_idle()

    def test_wrong_architecture_refused(self):
        with mock.patch.object(m.platform, "machine", return_value="x86_64"):
            with self.assertRaisesRegex(RuntimeError, "仅允许"):
                m.preflight()

    def test_new_capture_worker_blocks(self):
        proc = self.root / "proc"
        process = proc / "1234"
        process.mkdir(parents=True)
        (process / "cmdline").write_bytes(
            b"/home/nvidia/miniconda3/envs/lerobot/bin/python\0/x/record_openarm_fast_staging.py\0")
        self.assertEqual(m.active_processes(proc)[0]["pid"], 1234)

    def test_process_detection_matches_arguments(self):
        proc = self.root / "proc"
        proc.mkdir()
        cases = {101: [b"/usr/bin/python3", b"/x/remote-teleop-follower-watchdog"],
                 102: [b"/usr/bin/python3", b"/x/jetson_record_manager.py"],
                 103: [b"bash", b"-c", b"pgrep -f remote-teleop-follower"]}
        for pid, args in cases.items():
            path = proc / str(pid)
            path.mkdir()
            (path / "cmdline").write_bytes(b"\0".join(args) + b"\0")
        self.assertEqual({p["pid"] for p in m.active_processes(proc)}, {101, 102})

    def archive(self, name, link=False):
        archive = self.root / "input.tar.gz"
        with tarfile.open(archive, "w:gz") as stream:
            member = tarfile.TarInfo(name)
            if link:
                member.type = tarfile.SYMTYPE
                member.linkname = "/tmp/unsafe"
                stream.addfile(member)
            else:
                member.size = 2
                stream.addfile(member, io.BytesIO(b"ok"))
        return archive

    def test_path_traversal_rejected(self):
        for name in ("remote-control/../../escape", "/absolute", "wrong/file"):
            with self.subTest(name=name):
                with self.assertRaises(RuntimeError):
                    m.safe_extract(self.archive(name), self.root / "out", "remote-control")

    def test_archive_symlink_rejected(self):
        with self.assertRaises(RuntimeError):
            m.safe_extract(self.archive("remote-control/link", link=True),
                           self.root / "out", "remote-control")

    def test_safe_archive_extracts(self):
        m.safe_extract(self.archive("remote-control/file"), self.root / "out", "remote-control")
        self.assertEqual((self.root / "out/remote-control/file").read_text(), "ok")

    def test_overlay_replaces_link_without_writing_target(self):
        source, target = self.root / "src", self.root / "dst"
        source.mkdir()
        target.mkdir()
        outside = self.root / "outside"
        outside.write_text("keep")
        (source / "script.py").write_text("new")
        (target / "script.py").symlink_to(outside)
        m.overlay_files(source, target)
        self.assertEqual(outside.read_text(), "keep")
        self.assertEqual((target / "script.py").read_text(), "new")

    def test_completed_remote_job_clears_pending(self):
        pending = self.root / "pending.json"
        pending.write_text("{}")
        reply = json.dumps({"text": "", "offset": 0, "alive": False,
                            "result": {"returncode": 0, "finished_unix": 1}})
        with mock.patch.object(client, "STATE", self.root), \
                mock.patch.object(client, "PENDING", pending), \
                mock.patch.object(client, "remote", return_value=reply):
            self.assertEqual(client.follow("/job/example"), 0)
        self.assertFalse(pending.exists())

    def test_interrupted_remote_job_is_not_reported_success(self):
        pending = self.root / "pending.json"
        pending.write_text("{}")
        reply = json.dumps({"text": "", "offset": 0, "alive": False, "result": None})
        with mock.patch.object(client, "PENDING", pending), \
                mock.patch.object(client, "remote", return_value=reply):
            with self.assertRaisesRegex(RuntimeError, "中断"):
                client.follow("/job/example")
        self.assertFalse(pending.exists())

    def test_network_failure_retains_job_identity(self):
        pending = self.root / "pending.json"
        pending.write_text('{"job":"/job/example"}')
        with mock.patch.object(client, "PENDING", pending), \
                mock.patch.object(client, "remote", side_effect=RuntimeError("offline")), \
                mock.patch.object(client.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "同一任务"):
                client.follow("/job/example")
        self.assertTrue(pending.exists())


if __name__ == "__main__":
    unittest.main()
