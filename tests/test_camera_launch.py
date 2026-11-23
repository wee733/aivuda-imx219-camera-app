import json
import os
from pathlib import Path
import select
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest
import yaml


APP = Path(__file__).resolve().parents[1] / 'app'


@pytest.fixture
def camera(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(APP / 'scripts'))
    import camera_config
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.setenv('AIVUDAOS_WS_ROOT', str(tmp_path))
    return camera_config


def write_config(path, config):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(config))
    return path


def defaults():
    return yaml.safe_load((APP / 'config/default_config.yaml').read_text())


def test_saved_domain_and_override(camera, tmp_path):
    import jsonschema
    schema = yaml.safe_load((APP / 'config/config_schema.yaml').read_text())
    cfg = defaults()
    cfg['stream'].update(mode='off', width=640)
    for expected in (17, 0):
        cfg['camera']['ros_domain_id'] = expected
        jsonschema.validate(cfg, schema)
        path = write_config(tmp_path / 'config.yaml', cfg)
        domain, args = camera.prepare(path, tmp_path)
        assert domain == expected
        assert args == ['/app/scripts/in_container.sh', 'off', '/data/app_config.json']
        settings = json.loads((tmp_path / 'app_config.json').read_text())
        assert settings['ros_domain_id'] == expected
        assert settings['stream']['width'] == 640
    assert not jsonschema.Draft7Validator(schema).is_valid({**cfg, 'ros': {'domain_id': 17}})


def test_domain_sources(camera, tmp_path, monkeypatch):
    workspace = tmp_path / 'custom_workspace/config'
    path = write_config(workspace / 'apps/camera/1.1.0/1.1.0.yaml', defaults())
    assert camera.settings(path)['ros_domain_id'] == 0
    write_config(workspace / 'sys.yaml', {'_version': 7, 'sys': {'ros2_default_domain_id': 17}})
    assert camera.settings(path)['ros_domain_id'] == 17
    monkeypatch.setenv('ROS_DOMAIN_ID', '232')
    assert camera.settings(path)['ros_domain_id'] == 232
    monkeypatch.delenv('ROS_DOMAIN_ID')
    cfg = defaults()
    cfg['camera']['ros_domain_id'] = 0
    write_config(workspace / 'sys.yaml', {'sys': {'ros2_default_domain_id': True}})
    with pytest.raises(ValueError, match='sys.ros2_default_domain_id'):
        camera.settings(path)
    assert camera.settings(write_config(path, cfg))['ros_domain_id'] == 0
    # With a standalone app config there is no config/apps ancestor; use the workspace root.
    standalone = write_config(tmp_path / 'standalone.yaml', defaults())
    write_config(tmp_path / 'config/sys.yaml', {'sys': {'ros2_default_domain_id': 31}})
    assert camera.settings(standalone)['ros_domain_id'] == 31


@pytest.mark.parametrize('raw', ['', 'abc', '-1', '233', '2.5'])
def test_invalid_inherited_value(camera, tmp_path, monkeypatch, raw):
    monkeypatch.setenv('ROS_DOMAIN_ID', raw)
    config = write_config(tmp_path / 'config.yaml', defaults())
    with pytest.raises(ValueError, match='ROS_DOMAIN_ID') as exc:
        camera.prepare(config, tmp_path)
    assert raw in str(exc.value)
    assert not (tmp_path / 'app_config.json').exists()


@pytest.mark.parametrize('mode', ['off', 'isaac_ros_h264', 'native_nvenc'])
def test_explicit_settings_reach_the_driver(camera, tmp_path, monkeypatch, mode):
    monkeypatch.setenv('ROS_DOMAIN_ID', 'invalid')
    cfg = defaults()
    cfg['camera'].update(ros_domain_id=232, exposure_us=0)
    cfg['stream'].update(mode=mode, fps=30, bitrate_kbps=8000, iframe_interval=1)
    domain, command = camera.prepare(write_config(tmp_path / 'camera.yaml', cfg), tmp_path)
    assert domain == 232
    assert command == ['/app/scripts/in_container.sh', mode, '/data/app_config.json']
    result = json.loads((tmp_path / 'app_config.json').read_text())
    assert result['exposure_us'] == 0 and result['stream'] == cfg['stream']


@pytest.mark.parametrize('compat', [None, False, True])
def test_isaac_compressed_image_opt_in(camera, tmp_path, compat):
    import jsonschema
    cfg = defaults()
    cfg['stream']['mode'] = 'isaac_ros_h264'
    if compat is None:
        del cfg['stream']['publish_compressed_image']
    else:
        cfg['stream']['publish_compressed_image'] = compat
    jsonschema.validate(cfg, yaml.safe_load((APP / 'config/config_schema.yaml').read_text()))
    camera.prepare(write_config(tmp_path / 'camera.yaml', cfg), tmp_path)
    stream = json.loads((tmp_path / 'app_config.json').read_text())['stream']
    assert stream['publish_compressed_image'] is (compat is True)


@pytest.mark.parametrize(('key', 'value'), [('publish_compressed_image', 1),
                         ('publish_compressed_image', 'false'), ('mode', 'jpeg'), ('fps', 31), ('fps', True),
                         ('width', 641), ('bitrate_kbps', 299), ('iframe_interval', 0),
                         ('config', 'unknown'), ('qp', 52), ('profile', 3), ('hw_preset', -1)])
def test_invalid_stream(camera, tmp_path, key, value):
    cfg = defaults()
    cfg['stream'][key] = value
    path = write_config(tmp_path / 'config.yaml', cfg)
    with pytest.raises(ValueError, match='stream.' + key):
        camera.prepare(path, tmp_path)


def test_old_preview_disabled_stays_disabled(camera, tmp_path):
    cfg = defaults()
    del cfg['stream']
    cfg['preview'] = {'enabled': False}
    assert camera.settings(write_config(tmp_path / 'camera.yaml', cfg))['stream']['mode'] == 'off'


@pytest.mark.parametrize('script', ['start.sh', 'scripts/pre_install.sh'])
def test_missing_bundled_launcher_rejects_adjacent_runtime(tmp_path, script):
    app = tmp_path / 'package'
    entry = app / script
    entry.parent.mkdir(parents=True)
    shutil.copy(APP / script, entry)
    adjacent = tmp_path / 'isaac_runtime'
    adjacent.mkdir()
    (adjacent / 'run.py').write_text('raise SystemExit("legacy launcher invoked")\n')
    (adjacent / 'check.sh').write_text('exit 0\n')
    result = subprocess.run(['bash', str(entry)], capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert f'Missing bundled launcher: {app}/isaac_runtime/run.py' in result.stderr


@pytest.mark.parametrize(('states', 'rm_status', 'error'), [
    (['old exited', ''], 0, None),
    (['old exited', ''], 1, None),  # Auto-removal won before docker rm.
    (['old exited', 'old removing', ''], 1, None),
    (['old removing', ''], None, None),
    (['old exited', 'new created'], 1, 'already in use'),
    (['old paused'], None, 'already in use'),
    (['old exited', 'old exited'], 1, 'permission denied'),
    (['old removing', 'old removing'], None, 'removal is still in progress'),
])
def test_docker_cleanup(monkeypatch, states, rm_status, error):
    monkeypatch.syspath_prepend(str(APP / 'isaac_runtime'))
    import run
    rows = iter(states)
    rm_calls = []

    def docker(args, **kwargs):
        if args[1] == 'rm':
            rm_calls.append(args[2:])
            return subprocess.CompletedProcess(args, rm_status, '', 'permission denied' if rm_status else '')
        assert args[:3] == ['docker', 'container', 'ls']
        assert '--no-trunc' in args and 'name=^/camera$' in args
        assert '{{.ID}} {{.State}}' in args and kwargs['check']
        return subprocess.CompletedProcess(args, 0, next(rows, states[-1]), '')

    now = iter(range(0, 30, 2))
    monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=lambda: next(now), sleep=lambda _: None))
    monkeypatch.setattr(run.subprocess, 'run', docker)
    if error:
        with pytest.raises(SystemExit, match=error):
            run.cleanup_stale_container('camera')
    else:
        run.cleanup_stale_container('camera')
    assert rm_calls == ([['old']] if rm_status is not None else [])
    assert list(rows) == []


def test_competing_launch_keeps_owner_settings(tmp_path):
    # A foreground Docker stand-in keeps the real launcher's inherited flock through exec.
    docker = tmp_path / 'docker'
    docker.write_text('''#!/bin/sh
case "$1" in
    container) printf '%s' "$CONTAINER_STATE"; exit "${DOCKER_STATUS:-0}";;
    run) printf '%s\\n' "$*"; read -r done;;
esac
''')
    docker.chmod(0o755)
    entry = '''
import grp, os, sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(sys.argv[2]) / 'isaac_runtime'))
import run
run.CAMERA_OWNER_DIR = Path(os.environ['HOME']) / 'camera-lock'
grp.getgrnam = lambda _: SimpleNamespace(gr_gid=os.getgid())
run.main()
'''
    data = tmp_path / 'runtime'
    config = defaults()
    config['camera']['ros_domain_id'] = 17
    cfg = write_config(tmp_path / 'config.yaml', config)
    cmd = [sys.executable, '-c', entry, 'camera', str(APP), '--config', str(cfg)]
    env = dict(os.environ, HOME=str(tmp_path), PATH=f'{tmp_path}:{os.environ["PATH"]}',
               AIVUDA_APP_RUNTIME_DATA_PATH=str(data))
    first = subprocess.Popen(cmd, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True)
    try:
        assert select.select([first.stdout], [], [], 10)[0], 'First launch timed out'
        assert 'ROS_DOMAIN_ID=17' in first.stdout.readline()
        payload = (data / 'app_config.json').read_bytes()
        config['camera']['ros_domain_id'] = 42
        write_config(cfg, config)
        for target in (data, tmp_path / 'another-version'):
            rival = subprocess.run(cmd, env={**env, 'AIVUDA_APP_RUNTIME_DATA_PATH': str(target)},
                                   capture_output=True, text=True, timeout=10)
            assert rival.returncode != 0 and 'already has an owner' in rival.stderr
        assert (data / 'app_config.json').read_bytes() == payload
        assert not (tmp_path / 'another-version/app_config.json').exists()
    finally:
        first.communicate('stop\n', timeout=10)
    assert first.returncode == 0
    restarted = subprocess.run(cmd, env=env, input='stop\n', capture_output=True, text=True,
                               timeout=10, check=True)
    assert 'ROS_DOMAIN_ID=42' in restarted.stdout
    previous = (data / 'app_config.json').read_bytes()
    config['camera']['ros_domain_id'] = 53
    write_config(cfg, config)
    for condition in ({'CONTAINER_STATE': 'abc running'}, {'DOCKER_STATUS': '1'}):
        refused = subprocess.run(cmd, env={**env, **condition}, capture_output=True,
                                 text=True, timeout=10)
        assert refused.returncode != 0
        assert (data / 'app_config.json').read_bytes() == previous
