"""Pipe-input tests for the DTE options (no FireWire camera needed).

    python3 tests/pipe_test.py            # tests ./dvgrab
    DVGRAB=/path/to/dvgrab python3 tests/pipe_test.py

Needs ffmpeg to generate a 12 s PAL DV test clip.
"""
import atexit, json, os, shutil, signal, subprocess, sys, tempfile, threading, time, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
DVGRAB = os.path.abspath(os.environ.get("DVGRAB", os.path.join(HERE, "..", "dvgrab")))
SP = tempfile.mkdtemp(prefix="dvgrab-pipe-test-")
atexit.register(shutil.rmtree, SP, True)
SAMPLE = os.path.join(SP, "sample.dv")
FRAME = 144000  # PAL DV


def make_sample():
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y",
                    "-f", "lavfi", "-i", "testsrc=size=720x576:rate=25",
                    "-f", "lavfi", "-i", "sine=frequency=1000:sample_rate=48000",
                    "-t", "12", "-target", "pal-dv", "-ac", "2", SAMPLE], check=True)


class Run:
    def __init__(self, args, pre="", fps=25, directory=None):
        self.dir = directory or tempfile.mkdtemp(dir=SP, prefix="t")
        if pre:
            cmd = ["bash", "-c", "{} exec {} -input - {} {}/Cam-".format(pre, DVGRAB, args, self.dir)]
        else:
            cmd = [DVGRAB, "-input", "-"] + args.split() + [self.dir + "/Cam-"]
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.lines, self.events = [], []
        self.fps = fps
        threading.Thread(target=self._feed, daemon=True).start()
        threading.Thread(target=self._read, daemon=True).start()
        self.err = []
        threading.Thread(target=lambda: self.err.extend(self.p.stderr), daemon=True).start()

    def _feed(self):
        try:
            with open(SAMPLE, "rb") as f:
                while True:
                    b = f.read(FRAME)
                    if not b:
                        break
                    self.p.stdin.write(b); self.p.stdin.flush()
                    time.sleep(1.0 / self.fps)
            self.p.stdin.close()
        except (BrokenPipeError, ValueError):
            pass

    def _read(self):
        for raw in self.p.stdout:
            self.lines.append(raw.decode())
            self.events.append(json.loads(raw))

    def wait_type(self, t, timeout=5):
        end = time.time() + timeout
        while time.time() < end:
            if any(e["type"] == t for e in self.events):
                return
            time.sleep(0.02)
        raise AssertionError("no {} event; got {}".format(t, [e["type"] for e in self.events]))

    def sig(self, s):
        os.kill(self.p.pid, s)

    def finish(self, timeout=10):
        rc = self.p.wait(timeout=timeout)
        time.sleep(0.2)
        return rc

    def types(self, t):
        return [e for e in self.events if e["type"] == t]

    def files(self):
        return sorted(f for f in os.listdir(self.dir))


class PipeTests(unittest.TestCase):
    def test_trigger_takes(self):
        r = Run("--json --trigger --sync-interval 1 --timestamp --size 0")
        r.wait_type("ready")
        time.sleep(0.5)
        r.sig(signal.SIGUSR1); time.sleep(1.5)
        r.sig(signal.SIGUSR2); time.sleep(1.0)
        r.sig(signal.SIGUSR1); time.sleep(1.0)
        # quick disarm + re-arm between frames must still split the take
        r.sig(signal.SIGUSR2); r.sig(signal.SIGUSR1); time.sleep(1.0)
        r.sig(signal.SIGINT)
        self.assertEqual(r.finish(), 0)
        opens, closes = r.types("file_open"), r.types("file_closed")
        self.assertEqual(len(opens), 3, r.files())
        self.assertEqual([c["reason"] for c in closes], ["take_end", "take_end", "stop"])
        self.assertEqual(sorted(os.path.basename(o["file"]) for o in opens), r.files())
        self.assertEqual(len(r.files()), 3)
        for name in r.files():
            self.assertRegex(name, r"^Cam-\d{2}-\d{2}-\d{4}_\d{2}-\d{2}-\d{2}(-\d+)?\.dv$")
        for c in closes:
            size = os.path.getsize(c["file"])
            self.assertEqual(size, c["frames"] * FRAME)
        self.assertEqual(r.events[-1], {"type": "exit", "code": 0})
        statuses = [e["status"] for e in r.types("status")]
        self.assertIn("REC", statuses); self.assertIn("STBY", statuses)
        rec = [e for e in r.types("status") if e["status"] == "REC"]
        self.assertTrue(all(e["file"] for e in rec))

    def test_existing_name_is_not_overwritten(self):
        first = Run("--json --timestamp --size 0", fps=500)
        self.assertEqual(first.finish(timeout=20), 0)
        original = os.path.join(first.dir, first.files()[0])
        size = os.path.getsize(original)
        second = Run("--json --timestamp --size 0", fps=500, directory=first.dir)
        self.assertEqual(second.finish(timeout=20), 0)
        names = second.files()
        self.assertEqual(len(names), 2, names)
        self.assertTrue(any(n.endswith("-2.dv") for n in names), names)
        self.assertEqual(os.path.getsize(original), size)

    def test_early_sigusr1_does_not_kill(self):
        # Signals that arrive once main() is running, while DVgrab is still
        # being constructed, must not kill the process. (Before main() runs no
        # program can catch them; the app waits for "ready".)
        r = Run("--json --trigger --timestamp --size 0")
        time.sleep(0.05)
        for _ in range(5):
            r.sig(signal.SIGUSR1); time.sleep(0.01)
        r.wait_type("ready")
        r.wait_type("file_open")
        r.sig(signal.SIGINT)
        self.assertEqual(r.finish(), 0)

    def test_no_trigger_writes_everything(self):
        r = Run("--json --size 0", fps=500)
        self.assertEqual(r.finish(timeout=20), 0)
        closes = r.types("file_closed")
        self.assertEqual(len(closes), 1)
        self.assertGreater(closes[0]["frames"], 280)

    def test_size_split_reports_both_files(self):
        r = Run("--json --size 10", fps=500)
        self.assertEqual(r.finish(timeout=20), 0)
        closes = r.types("file_closed")
        self.assertGreaterEqual(len(closes), 4)
        self.assertEqual(closes[0]["reason"], "split")
        self.assertEqual(closes[-1]["reason"], "stop")
        self.assertEqual(len(r.types("file_open")), len(closes))

    def test_write_failure_reports_error_and_exits_cleanly(self):
        # Ignore SIGXFSZ so exceeding the file size limit gives EFBIG on write.
        r = Run("--json --size 0", pre="trap '' XFSZ; ulimit -f 4000;", fps=500)
        rc = r.finish(timeout=20)
        self.assertNotEqual(rc, -signal.SIGABRT)
        errors = r.types("error")
        self.assertEqual(len(errors), 1, r.events[-5:])
        self.assertEqual(errors[0]["code"], "write_failed")
        self.assertEqual(r.types("file_closed")[-1]["reason"], "error")
        self.assertEqual(r.events[-1]["type"], "exit")

    def test_sigint_mid_stream_does_not_hang(self):
        r = Run("--json --size 0")
        r.wait_type("file_open")
        time.sleep(0.5)
        r.sig(signal.SIGINT)
        self.assertEqual(r.finish(timeout=5), 0)

    def test_dot_in_folder_name(self):
        d = tempfile.mkdtemp(dir=SP, prefix="dot.dir")
        p = subprocess.run([DVGRAB, "-input", "-", "--size", "0", d + "/x-"], stdin=open(SAMPLE, "rb"),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
        self.assertEqual(p.returncode, 0, p.stderr[-300:])
        self.assertEqual(os.listdir(d), ["x-001.dv"])

    def test_json_with_raw_pipe_rejected(self):
        p = subprocess.run([DVGRAB, "-input", "-", "--json", "-"], stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        self.assertNotEqual(p.returncode, 0)

    def test_version(self):
        p = subprocess.run([DVGRAB, "--version"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertIn(b"-dte", p.stdout)


if __name__ == "__main__":
    make_sample()
    unittest.main(verbosity=2)
