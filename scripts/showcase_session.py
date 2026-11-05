#!/usr/bin/env python3
"""Keep a showcase launch and its simulator children in one disposable session."""
import os
import signal
import subprocess
import sys
import time


def main():
    if len(sys.argv) < 2:
        raise SystemExit('Supply the showcase launch command')
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)
    interrupted = False

    def stop(signum, _frame):
        nonlocal interrupted
        interrupted = True
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while child.poll() is None:
            if interrupted:
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    break
            else:
                time.sleep(.1)
    finally:
        # xvfb-run can exit before its Gazebo child. I clean up the whole
        # isolated group even when ros2 launch itself has already stopped.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + (3 if sig != signal.SIGKILL else 1)
            while time.monotonic() < deadline:
                child.poll()
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(.05)
            else:
                continue
            break
        child.wait()
    return 130 if interrupted else child.returncode


if __name__ == '__main__':
    raise SystemExit(main())
