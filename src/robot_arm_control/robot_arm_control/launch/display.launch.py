from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_file]),
                    value_type=str),
                'frame_prefix': '',
                'publish_default_positions': False,
            }]
        ),

        # RVIZ2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),

        # Servo Driver Node
        Node(
            package='robot_arm_control',
            executable='servo_driver',
            name='servo_driver',
        )
    ])