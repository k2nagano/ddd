# launch/nucleus_launch.py
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # 1) nucleus_node の起動
    nucleus_node = Node(
        package='nucleus_driver_ros2',
        executable='nucleus_node',
        name='nucleus_node',
        output='screen'
    )

    # 2) TCP 接続コマンド
    connect_tcp = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nucleus_driver_ros2',
            'connect_tcp', '192.168.2.201', 'nortek'
        ],
        output='screen'
    )

    # 3) start コマンド
    start_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nucleus_driver_ros2', 'start'],
        output='screen'
    )

    # nucleus_node 起動後に TCP 接続
    connect_on_nucleus = RegisterEventHandler(
        OnProcessStart(
            target_action=nucleus_node,
            on_start=[connect_tcp],
        )
    )

    # 接続成功（プロセス終了）後に start
    start_on_connect = RegisterEventHandler(
        OnProcessExit(
            target_action=connect_tcp,
            on_exit=[start_cmd],
        )
    )

    return LaunchDescription([
        nucleus_node,
        connect_on_nucleus,
        start_on_connect,
    ])
