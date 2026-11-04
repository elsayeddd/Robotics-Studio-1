#!/usr/bin/env python3
"""Verify the Qt console starts and shuts down cleanly with both vehicle panels."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('executable')
    args = parser.parse_args()
    for termination in (signal.SIGINT, signal.SIGTERM):
        primary = termination.name
        with tempfile.TemporaryDirectory(prefix='embertrace_qt_') as directory:
            environment = dict(os.environ, QT_QPA_PLATFORM='offscreen', XDG_RUNTIME_DIR=directory)
            with (Path(directory) / 'console.log').open('w+') as log:
                process = subprocess.Popen([args.executable],
                    stdout=log, stderr=subprocess.STDOUT, env=environment)
                try:
                    deadline = time.monotonic() + 10
                    while 'EmberTrace dashboard ready' not in (Path(directory) / 'console.log').read_text():
                        assert process.poll() is None, f'{primary} dashboard exited during startup'
                        assert time.monotonic() < deadline, 'Dashboard did not become ready'
                        time.sleep(.05)
                    time.sleep(.2)
                    process.send_signal(termination)
                    code = process.wait(timeout=5)
                    assert code == 0, f'{primary} dashboard shutdown failed: {code}'
                finally:
                    if process.poll() is None:
                        process.kill(); process.wait(timeout=5)
                    log.seek(0); print(log.read(), end='')
    print('Qt dashboard startup/shutdown passed with both vehicle panels for SIGINT and SIGTERM')


if __name__ == '__main__':
    main()
