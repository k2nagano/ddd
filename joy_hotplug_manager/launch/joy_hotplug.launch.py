from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    mapping = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', '').split(':')[0] or '/',
        'share', 'joy_hotplug_manager', 'config', 'mappings.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy_hotplug_manager',
            executable='joy_manager',
            name='joy_manager',
            output='screen',
            parameters=[{
                'mapping_yaml': mapping,
                # 必要に応じて変更（環境で exec 名が違う場合あり）
                'joy_linux_pkg': 'joy_linux',
                'joy_linux_exec': 'joy_linux_node',
                # 追加パラメータ例: deadzone や autorepeat を入れたいとき
                # 'extra_ros_args': ['--ros-args', '-p', 'deadzone:=0.05', '-p', 'autorepeat_rate:=20.0'],
            }]
        ),
        Node(
            package='joy_hotplug_manager',
            executable='joy_mapper',
            name='joy_mapper',
            output='screen',
            parameters=[{'mapping_yaml': mapping}]
        ),
    ])
