"""视觉任务：任务逻辑、识别、可选串口和状态窗口。"""
from launch_ros.actions import Node
from rcs_task.rcs_task_launch import status_remappings, task_launch, task_node


def build_nodes(config, path):
    task_parameters = {key: config[key] for key in (
        'qr_enabled', 'initial_mode', 'send_interval', 'qr_send_duration', 'qr_confirm_frames')}
    vision_parameters = {key: config[key] for key in (
        'block_camera', 'qr_camera', 'width', 'height', 'process_period', 'initial_mode',
        'qr_enabled', 'qr_backend', 'qr_model_dir', 'preview', 'qr_preview',
        'block_min_radius', 'block_max_radius', 'block_min_fill', 'block_min_circularity',
        'block_min_aspect', 'ring_min_radius', 'ring_max_radius', 'ring_min_area', 'ring_param2')}
    nodes = [
        task_node('vision', config, path, parameters=task_parameters),
        Node(package='rcs_vision', executable='rcs_vision', name='rcs_vision', output='screen',
             arguments=['--config', config['vision_config']], parameters=[vision_parameters],
             remappings=status_remappings('vision')),
    ]
    if config['monitor']:
        nodes.append(Node(package='rcs_debug', executable='rcs_debug_monitor',
                          name='rcs_debug_monitor', output='screen',
                          parameters=[{'fullscreen': config['fullscreen']}],
                          remappings=status_remappings('vision')))
    return nodes


def generate_launch_description():
    return task_launch('vision', build_nodes)
