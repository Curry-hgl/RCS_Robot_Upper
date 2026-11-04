"""短命令启动器；通过 LaunchService 管理整套任务进程。"""
import argparse

from launch import LaunchService
from .rcs_task import select_tasks
from .rcs_task_launch import system_launch


def parse_args(args=None):
    parser = argparse.ArgumentParser(description='启动完整任务流程：upper chassis vision')
    parser.add_argument('tasks', nargs='*', metavar='TASK', help='省略时读取总配置')
    parser.add_argument('--config', help='总配置路径')
    options = parser.parse_args(args)
    if options.tasks:
        try:
            options.tasks = select_tasks(options.tasks)
        except ValueError as exc:
            parser.error(str(exc))
    return options


def main(args=None):
    options = parse_args(args)
    service = LaunchService()
    service.include_launch_description(system_launch(tasks=options.tasks or None, config=options.config))
    return service.run()
