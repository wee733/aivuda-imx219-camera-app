"""Resolve camera settings on the host before starting the Isaac ROS container."""
import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

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
    domain = integer(camera.get('ros_domain_id', -1), 'camera.ros_domain_id', -1, 232)
    if domain == -1:
        domain = inherited_domain(config_path)
    exposure = integer(camera['exposure_us'], 'camera.exposure_us', 0, 33333)

    # Existing versioned configs may retain the old preview switch.
    defaults = read_mapping(Path(__file__).resolve().parents[1] / 'config/default_config.yaml')['stream']
    stream = dict(defaults)
    if 'stream' in config:
        if not isinstance(config['stream'], dict) or set(config['stream']) - set(stream):
            raise ValueError('Invalid stream configuration')
        stream.update(config['stream'])
    elif config.get('preview', {}).get('enabled') is False:
        stream['mode'] = 'off'
    for key, allowed in (('mode', ('native_nvenc', 'isaac_ros_h264', 'off')),
                         ('config', ('custom', 'pframe_cqp', 'iframe_cqp')),
                         ('width', (640, 960, 1280))):
        if type(stream[key]) is not type(defaults[key]) or stream[key] not in allowed:
            raise ValueError(f'stream.{key} must be one of {allowed!r}')
    if type(stream['publish_compressed_image']) is not bool:
        raise ValueError('stream.publish_compressed_image must be a boolean')
    for key, low, high in (('fps', 1, 30), ('bitrate_kbps', 300, 8000),
                           ('iframe_interval', 1, 120), ('qp', 0, 51),
                           ('hw_preset', 0, 3), ('profile', 0, 2)):
        integer(stream[key], 'stream.' + key, low, high)
    return {'ros_domain_id': domain, 'exposure_us': exposure, 'stream': stream}


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
        '/app/scripts/in_container.sh', payload['stream']['mode'], '/data/app_config.json',
    ]
