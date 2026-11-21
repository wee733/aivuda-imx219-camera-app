#!/usr/bin/python3
"""Stop the ROS process group if the foreground host process or AivudaOS exits."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys


def main():
    owner, expected_start, *command = sys.argv[1:]
    owner_fd = os.pidfd_open(int(owner))
    start_ticks = Path(f'/proc/{owner}/stat').read_text().rsplit(')', 1)[1].split()[19]
    if start_ticks != expected_start:
        raise SystemExit('AivudaOS owner exited before sensor startup')
    owner_exit = select.poll()
    owner_exit.register(owner_fd, select.POLLIN)
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    child = subprocess.Popen(command, start_new_session=True)
    try:
        while child.poll() is None and not stopping:
            if owner_exit.poll(250):
                stopping = True
    finally:
        # Always terminate any components left behind by a launch parent.
        try:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
    return 0 if stopping else child.returncode


if __name__ == '__main__':
    sys.exit(main())
