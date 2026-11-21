"""Check the DDS discovery settings passed from aivudaOS to the Isaac ROS container."""

import importlib.util
from pathlib import Path

APP_DIR = Path(__file__).resolve().parents[1] / 'app'


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_isaac_runtime_mirrors_aivuda_discovery_env(monkeypatch):
    run_mod = _load_module('isaac_runtime_run', APP_DIR / 'isaac_runtime' / 'run.py')

    for key in ('ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS'):
        monkeypatch.delenv(key, raising=False)
    assert run_mod.resolve_discovery_env() == ['--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST']

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    assert run_mod.resolve_discovery_env() == ['--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST']

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.delenv('ROS_AUTOMATIC_DISCOVERY_RANGE', raising=False)
    assert run_mod.resolve_discovery_env() == ['--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET']

    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setenv('ROS_STATIC_PEERS', '192.168.1.10')
    assert run_mod.resolve_discovery_env() == [
        '--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET',
        '--env', 'ROS_STATIC_PEERS=192.168.1.10',
    ]
