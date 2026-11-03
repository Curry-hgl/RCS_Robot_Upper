"""工创一键启动：参数统一传递，显示节点默认关闭，便于无桌面部署。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'port': '/dev/ttyUSB0', 'baudrate': '115200',
        'escape_enabled': 'false', 'block_camera': '/dev/video20',
        'qr_camera': '0', 'qr_enabled': 'true', 'qr_backend': 'opencv',
        'qr_model_dir': '~/.opencv/wechat_qrcode', 'preview': 'false',
        'monitor': 'false', 'fullscreen': 'false', 'send_interval': '0.5',
        'qr_send_duration': '5.0', 'initial_mode': '0',
    }
    def value(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)
    args = [DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()]
    return LaunchDescription(args + [
        Node(package='rcs_serial', executable='serial_bridge', output='screen', parameters=[{
            'port': value('port', str), 'baudrate': value('baudrate', int),
            'escape_enabled': value('escape_enabled', bool)}]),
        Node(package='rcs_task', executable='gcs_task', output='screen', parameters=[{
            'qr_enabled': value('qr_enabled', bool), 'initial_mode': value('initial_mode', int),
            'send_interval': value('send_interval', float),
            'qr_send_duration': value('qr_send_duration', float)}]),
        Node(package='rcs_vision', executable='gcs_vision', output='screen', parameters=[{
            'block_camera': value('block_camera', str), 'qr_camera': value('qr_camera', str),
            'qr_enabled': value('qr_enabled', bool), 'qr_backend': value('qr_backend', str),
            'qr_model_dir': value('qr_model_dir', str), 'preview': value('preview', bool)}]),
        Node(package='rcs_debug', executable='gcs_monitor', output='screen',
             condition=IfCondition(LaunchConfiguration('monitor')),
             parameters=[{'fullscreen': value('fullscreen', bool)}]),
    ])
