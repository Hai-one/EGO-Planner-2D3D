"""
单独启动 RViz (不启动节点)
用法: ros2 launch ego_planner_real rviz.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('ego_planner_real'),
        'config', 'real_world.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([rviz_node])
