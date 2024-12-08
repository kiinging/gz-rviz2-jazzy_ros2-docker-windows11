import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description_path = os.path.join(get_package_share_directory('generic_robot_package'), 'urdf', 'generic_robot.urdf')
    diff_drive_controller_config = os.path.join(get_package_share_directory('controller_manager'), 'config', 'diff_drive_controller.yaml')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description_path': robot_description_path},
                {'robot_description': open(robot_description_path).read()},
                diff_drive_controller_config
            ]
        )
    ])