"""chassis 任务启动入口。"""
from rcs_task.rcs_task_launch import task_launch, task_node


def build_nodes(config, path):
    return [task_node('chassis', config, path)]


def generate_launch_description():
    return task_launch('chassis', build_nodes)
