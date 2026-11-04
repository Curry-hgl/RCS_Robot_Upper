"""总配置只选择任务，内部参数由各任务配置提供。"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from rcs_vision.rcs_vision_config import load_vision_config

from .rcs_task import TASK_NAMES, select_tasks


def config_dir():
    return Path(get_package_share_directory('rcs_task')) / 'config'


def _read(path):
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f'Configuration must be a mapping: {path}')
    return config


def load_system_config(path=None, *, tasks=None):
    path = Path(path).expanduser().resolve() if path else config_dir() / 'rcs_task.yaml'
    config = _read(path)
    tasks = select_tasks(tasks if tasks is not None else config.get('tasks', ['vision']))
    overrides = config.get('task_configs', {})
    if not isinstance(overrides, dict) or any(name not in TASK_NAMES for name in overrides):
        raise ValueError('task_configs must map task names to configuration paths')
    profiles = {}
    for name in tasks:
        if name in overrides:
            profile = Path(overrides[name]).expanduser()
            if not profile.is_absolute():
                profile = path.parent / profile
        else:
            profile = config_dir() / f'rcs_task_{name}.yaml'
        profiles[name] = str(profile.resolve())
    return tasks, profiles


def load_task_config(name, path=None, *, vision_config=None):
    select_tasks([name])
    default = config_dir() / f'rcs_task_{name}.yaml'
    path = Path(path).expanduser().resolve() if path else default
    config = _read(default)
    if path != default:
        config.update(_read(path))
    if 'vision_config' in config:
        vision_path = config['vision_config'] if vision_config is None else vision_config
        if vision_path:
            vision_path = Path(vision_path).expanduser()
            if not vision_path.is_absolute():
                vision_path = path.parent / vision_path
        else:
            vision_path = Path(get_package_share_directory('rcs_vision')) / 'config/rcs_vision.yaml'
        merged = load_vision_config(vision_path)
        merged.update(config)
        merged['vision_config'] = str(vision_path.resolve())
        config = merged
    for key in ('serial_enabled', 'monitor', 'fullscreen'):
        if key in config and type(config[key]) is not bool:
            raise ValueError(f'{key} must be true or false')
    config['tasks'] = [name]
    return config
