#!/usr/bin/python3
"""Run one local Isaac ROS 5 workload in a foreground, Aivuda-owned container."""
import fcntl
import grp
import os
from pathlib import Path
import subprocess
import sys


IMAGE = 'airborne/isaac-ros:5.0-imx219-preview-20261022'
DEVICE_PATTERNS = ('nvhost-gpu', 'nvhost-as-gpu', 'nvhost-ctrl-gpu', 'nvhost-power-gpu',
                   'nvhost-tsg-gpu', 'nvgpu/igpu0/as', 'nvgpu/igpu0/channel',
                   'nvgpu/igpu0/ctrl', 'nvgpu/igpu0/power', 'nvgpu/igpu0/tsg',
                   'nvmap', 'nvsciipc', 'dri/renderD*', 'dri/card*')


def resolve_discovery_env():
    """Mirror AivudaOS global runtime_environment into the ROS 2 Lyrical container."""
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '').strip()
    explicit_range = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').strip()
    static_peers = os.environ.get('ROS_STATIC_PEERS', '').strip()
    if localhost_only == '1':
        discovery_range = 'LOCALHOST'
    elif explicit_range:
        discovery_range = explicit_range
    elif localhost_only == '0':
        discovery_range = 'SUBNET'
    else:
        discovery_range = 'LOCALHOST'
    env = ['--env', f'ROS_AUTOMATIC_DISCOVERY_RANGE={discovery_range}']
    if static_peers:
        env.extend(['--env', f'ROS_STATIC_PEERS={static_peers}'])
    return env


def cleanup_stale_container(container_name):
    """Remove a leftover stopped container with the same name after an unclean shutdown."""
    inspect = subprocess.run(
        ['docker', 'inspect', '--format', '{{.State.Running}}', container_name],
        capture_output=True, text=True, check=False,
    )
    if inspect.returncode != 0:
        return
    if inspect.stdout.strip().lower() == 'true':
        raise SystemExit(f'Container {container_name} is already running')
    subprocess.run(['docker', 'rm', '-f', container_name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def main():
    mode, root, domain, *command = sys.argv[1:]
    if mode not in ('camera', 'apriltag') or not command or not 0 <= int(domain) <= 232:
        raise ValueError('Expected camera|apriltag APP_ROOT ROS_DOMAIN_ID COMMAND ...')
    root = Path(root).resolve()
    runtime = Path(__file__).resolve().parent
    # The old airborne worker uses the same inode for its per-service flock.
    legacy = Path.home() / 'workspaces/airborne/control/run' / f'{mode}.lock'
    if legacy.parent.is_dir():
        guard = os.open(legacy, os.O_RDWR | os.O_CREAT, 0o660)
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(f'{mode} already has an airborne owner ({legacy})')
        os.set_inheritable(guard, True)

    container_name = f'aivuda_isaac_{mode}'
    cleanup_stale_container(container_name)

    runtime_data_env = os.environ.get('AIVUDA_APP_RUNTIME_DATA_PATH', '').strip()
    runtime_data = (Path(runtime_data_env).resolve() if runtime_data_env
                    else Path(f'/tmp/aivuda-isaac-{mode}'))
    runtime_data.mkdir(parents=True, exist_ok=True)

    # The same PID becomes the foreground Docker client at exec. The worker
    # watches that process even if AivudaOS ultimately sends it SIGKILL.
    owner = os.getpid()
    start_ticks = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]
    args = ['docker', 'run', '--rm', '--pull=never', '--init', '--name', container_name,
            '--network=host', '--ipc=host', '--pid=host', '--runtime=nvidia',
            '--user', f'{os.getuid()}:{os.getgid()}', '--cap-drop=ALL',
            '--tmpfs', '/home/app:rw,mode=1777', '--workdir', '/home/app',
            '--env', 'HOME=/home/app', '--env', 'ROS_HOME=/home/app/.ros',
            '--env', 'AIVUDA_DATA_DIR=/data',
            '--env', f'ROS_DOMAIN_ID={int(domain)}', *resolve_discovery_env(),
            '--env', 'NVIDIA_VISIBLE_DEVICES=all', '--env', 'NVIDIA_DRIVER_CAPABILITIES=all',
            '--volume', '/tmp:/tmp', '--volume', f'{root}:/app:ro',
            '--volume', f'{runtime}:/opt/aivuda-isaac:ro',
            '--volume', f'{runtime_data}:/data:rw',
            '--entrypoint', '/bin/bash']
    for name in ('video', 'render'):
        args.extend(['--group-add', str(grp.getgrnam(name).gr_gid)])
    for pattern in DEVICE_PATTERNS:
        for path in sorted(Path('/dev').glob(pattern)):
            if path.is_char_device():
                args.extend(['--device', str(path)])
    args += [IMAGE, '/opt/aivuda-isaac/environment.sh', str(owner), start_ticks, *command]
    os.execvp(args[0], args)


if __name__ == '__main__':
    main()
