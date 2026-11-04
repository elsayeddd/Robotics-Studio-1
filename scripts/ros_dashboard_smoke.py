#!/usr/bin/env python3
"""Verify the Qt console starts and shuts down cleanly for each primary vehicle."""
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
    for primary in ('rover', 'drone'):
        with tempfile.TemporaryDirectory(prefix='embertrace_qt_') as directory:
            environment = dict(os.environ, QT_QPA_PLATFORM='offscreen', XDG_RUNTIME_DIR=directory)
            with (Path(directory) / 'console.log').open('w+') as log:
                process = subprocess.Popen([args.executable, '--ros-args', '-p', f'primary_vehicle:={primary}'],
                    stdout=log, stderr=subprocess.STDOUT, env=environment)
                try:
                    deadline = time.monotonic() + 2
                    while time.monotonic() < deadline:
                        assert process.poll() is None, f'{primary} dashboard exited during startup'
                        time.sleep(.05)
                    process.send_signal(signal.SIGINT)
                    assert process.wait(timeout=5) == 0, f'{primary} dashboard shutdown failed'
                finally:
                    if process.poll() is None:
                        process.kill(); process.wait(timeout=5)
                    log.seek(0); print(log.read(), end='')
    print('Qt dashboard startup/shutdown passed for rover and drone')


if __name__ == '__main__':
    main()
