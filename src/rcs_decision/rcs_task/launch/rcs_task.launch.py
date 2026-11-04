"""按总配置组合子任务 launch。"""
from rcs_task.rcs_task_launch import system_launch


def generate_launch_description():
    return system_launch()
