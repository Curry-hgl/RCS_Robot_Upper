"""子任务启动与公共依赖复用。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .rcs_task import select_tasks
from .rcs_task_config import config_dir, load_system_config, load_task_config


def _overrides(config, context):
    config = config.copy()
    for name, default in config.items():
        if name in ('tasks', 'vision_config') or name not in context.launch_configurations:
            continue
        raw = context.launch_configurations[name]
        if type(default) is bool:
            if raw.lower() not in ('true', 'false', '1', '0'):
                raise ValueError(f'{name} must be true or false')
            config[name] = raw.lower() in ('true', '1')
        elif type(default) in (int, float, str):
            config[name] = type(default)(raw)
    return config


def _effective_config(name, path, context):
    return _overrides(load_task_config(
        name, path, vision_config=context.launch_configurations.get('vision_config')), context)


def _serial_settings(config):
    if not config['serial_enabled']:
        return None
    return {name: config[name] for name in ('port', 'baudrate', 'escape_enabled')}


def _serial_node(settings):
    return Node(package='rcs_serial', executable='serial_bridge', name='rcs_serial_bridge',
                output='screen', parameters=[settings])


def shared_resources(configs):
    settings = None
    for config in configs:
        request = _serial_settings(config)
        if request is None:
            continue
        if settings is not None and request != settings:
            raise ValueError('Selected tasks have conflicting shared serial settings')
        settings = request
    return [_serial_node(settings)] if settings is not None else []


def status_remappings(name):
    return [('/rcs_task/status', f'/rcs_task/{name}/status')]


def task_node(name, config, config_path, *, parameters=None):
    values = {'tasks': name, 'tick_period': config['tick_period']}
    values.update(parameters or {})
    return Node(package='rcs_task', executable='rcs_task_node', name=f'rcs_task_{name}',
                output='screen', arguments=[name, '--config', str(config_path)],
                parameters=[values], remappings=status_remappings(name))


def _task_actions(context, *, name, path, build_nodes):
    config = _effective_config(name, path, context)
    managed = LaunchConfiguration('shared_serial').perform(context).lower() == 'true'
    resources = [] if managed else shared_resources([config])
    return resources + build_nodes(config, path)


def _declare_task(context, *, name, build_nodes):
    path = LaunchConfiguration('config').perform(context)
    config = _effective_config(name, path, context)
    declarations = [DeclareLaunchArgument(key, default_value=str(value).lower()
                                         if type(value) is bool else str(value))
                    for key, value in config.items() if key != 'tasks']
    return declarations + [OpaqueFunction(function=_task_actions, kwargs={'name': name, 'path': path, 'build_nodes': build_nodes})]


def task_launch(name, build_nodes):
    select_tasks([name])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config_dir() / f'rcs_task_{name}.yaml')),
        DeclareLaunchArgument('shared_serial', default_value='false', description='组合启动内部使用'),
        OpaqueFunction(function=_declare_task, kwargs={'name': name, 'build_nodes': build_nodes}),
    ])


def _system_actions(context):
    explicit = LaunchConfiguration('tasks').perform(context)
    names, paths = load_system_config(LaunchConfiguration('config').perform(context),
                                     tasks=select_tasks(explicit) if explicit else None)
    configs = [_effective_config(name, paths[name], context) for name in names]
    resources = shared_resources(configs)  # Validate every request before starting anything.
    launch_dir = Path(get_package_share_directory('rcs_task')) / 'launch'
    children = [GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / f'rcs_task_{name}.launch.py')),
            launch_arguments={'config': paths[name], 'shared_serial': 'true'}.items())
    ]) for name in names]
    return resources + children


def system_launch(*, tasks=None, config=None):
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config or config_dir() / 'rcs_task.yaml')),
        DeclareLaunchArgument('tasks', default_value=','.join(select_tasks(tasks)) if tasks else ''),
        OpaqueFunction(function=_system_actions),
    ])
