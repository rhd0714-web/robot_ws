"""
robot.launch.py

사용법:
  ros2 launch ah_bot_dashboard robot.launch.py
  ros2 launch ah_bot_dashboard robot.launch.py use_plotjuggler:=true
  ros2 launch ah_bot_dashboard robot.launch.py use_mujoco:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = os.path.join(
    os.path.expanduser('~'), 'robot_ws', 'config', 'config.yaml'
)

PLOTJUGGLER_LAYOUT = os.path.join(
    os.path.expanduser('~'), 'robot_ws', 'test_layout.xml'
)


def generate_launch_description():

    arg_config = DeclareLaunchArgument(
        'config',
        default_value=DEFAULT_CONFIG,
        description='config.yaml 경로'
    )
    arg_plotjuggler = DeclareLaunchArgument(
        'use_plotjuggler',
        default_value='false',
        description='PlotJuggler 자동 실행 여부'
    )
    arg_mujoco = DeclareLaunchArgument(
        'use_mujoco',
        default_value='true',
        description='MuJoCo 시뮬레이션 실행 여부'
    )

    config_path     = LaunchConfiguration('config')
    use_plotjuggler = LaunchConfiguration('use_plotjuggler')
    use_mujoco      = LaunchConfiguration('use_mujoco')

    vision_node = Node(
        package    = 'ah_bot_dashboard',
        executable = 'vision_node',
        name       = 'vision_node',
        output     = 'screen',
        parameters = [{'config_path': config_path}],
    )

    driver_node = Node(
        package    = 'ah_bot_driver',
        executable = 'driver_node',
        name       = 'driver_node',
        output     = 'screen',
        parameters = [{'config_path': config_path}],
        respawn    = True,
        respawn_delay = 2.0,
    )

    mujoco_node = Node(
        package    = 'ah_bot_mujoco',
        executable = 'mujoco_node',
        name       = 'mujoco_node',
        output     = 'screen',
        condition  = IfCondition(use_mujoco),
    )

    plotjuggler = ExecuteProcess(
        cmd       = ['ros2', 'run', 'plotjuggler', 'plotjuggler',
                     '--layout', PLOTJUGGLER_LAYOUT],
        output    = 'screen',
        condition = IfCondition(use_plotjuggler),
    )

    return LaunchDescription([
        arg_config,
        arg_plotjuggler,
        arg_mujoco,
        vision_node,
        driver_node,
        mujoco_node,
        plotjuggler,
    ])
