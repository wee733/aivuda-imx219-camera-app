"""Resolve camera settings on the host before starting the Isaac ROS container."""
import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

PREVIEW_NUMBERS = {
    'fps': (float, 15.0, 1, 30),
    'width': (int, 960, 0, 1280),
    'jpeg_quality': (int, 85, 40, 95),
    'h264_bitrate_kbps': (int, 3000, 500, 12000),
}


def preview_numbers(data, *, live_update=False):
    numbers = {}
    for name, (convert, default, low, high) in PREVIEW_NUMBERS.items():
        if live_update and name not in data:
            continue
        value = data.get(name, default)
        if live_update:
            value = convert(value)
        types = (int, float) if convert is float else (int,)
        if type(value) not in types or not low <= value <= high:
            kind = 'a number' if convert is float else 'an integer'
            raise ValueError(f'preview.{name} must be {kind} in [{low}, {high}], got {value!r}')
        numbers[name] = value if live_update else convert(value)
    return numbers


def integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}], got {value!r}')
    return value


def read_mapping(path):
    import yaml

    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    return data


def inherited_domain(config_path):
    raw = os.environ.get('ROS_DOMAIN_ID')
    if raw is not None:
        if not raw.isascii() or not raw.isdecimal():
            raise ValueError(f'Invalid ROS_DOMAIN_ID {raw!r}, expected 0 to 232')
        return integer(int(raw), 'ROS_DOMAIN_ID', 0, 232)

    # Older launchers expose the active config path without exporting the system domain.
    config_dir = next((p.parent for p in config_path.resolve().parents
                       if p.name == 'apps' and p.parent.name == 'config'), None)
    if config_dir is None:
        workspace = Path(os.environ.get('AIVUDAOS_WS_ROOT') or Path.home() / 'aivudaOS_ws')
        config_dir = workspace / 'config'
    try:
        system = read_mapping(config_dir / 'sys.yaml').get('sys', {})
    except FileNotFoundError:
        return 0
    if not isinstance(system, dict):
        raise ValueError(f'{config_dir}/sys.yaml must contain a mapping for sys')
    return integer(system.get('ros2_default_domain_id', 0), 'sys.ros2_default_domain_id', 0, 232)


def settings(config_path):
    config = read_mapping(config_path)
    camera = config['camera']
    if 'ros_domain_id' in camera:
        domain = integer(camera['ros_domain_id'], 'camera.ros_domain_id', -1, 232)
    elif 'ros' in config:
        domain = integer(config['ros']['domain_id'], 'ros.domain_id', 0, 232)
    else:
        domain = -1
    if domain == -1:
        domain = inherited_domain(config_path)
    exposure = integer(camera['exposure_us'], 'camera.exposure_us', 0, 33333)

    preview = config.get('preview', {})
    enabled = preview.get('enabled', True)
    if type(enabled) is not bool:
        raise ValueError(f'preview.enabled must be a boolean, got {enabled!r}')
    codec = preview.get('codec', 'auto')
    if codec not in ('auto', 'nvenc_h264', 'jpeg'):
        raise ValueError('preview.codec must be auto, nvenc_h264 or jpeg')
    return {
        'ros_domain_id': domain,
        'exposure_us': exposure,
        'preview': {
            'enabled': enabled,
            **preview_numbers(preview),
            'codec': codec,
            'ui_port': 19311,
        },
    }


def prepare(config_path, runtime_data):
    payload = settings(config_path)
    tmp = NamedTemporaryFile(mode='w', encoding='utf-8', dir=runtime_data, delete=False)
    try:
        with tmp:
            json.dump(payload, tmp, indent=2, ensure_ascii=False)
            tmp.write('\n')
        os.replace(tmp.name, runtime_data / 'app_config.json')
    finally:
        Path(tmp.name).unlink(missing_ok=True)
    return payload['ros_domain_id'], [
        '/app/scripts/in_container.sh', str(payload['exposure_us']),
        '1' if payload['preview']['enabled'] else '0', '/data/app_config.json',
    ]
