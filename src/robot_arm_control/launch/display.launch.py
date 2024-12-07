from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf')
    controller_config = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_arm_control"), "urdf", "robot_arm.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Delay the joint state broadcaster spawner
    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    # Delay the trajectory controller spawner and make it dependent on joint state broadcaster
    delayed_robot_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    # Add delay for RViz after controllers are loaded
    delay_rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
            )
        ]
    )

    # Servo Driver Node - start after controller manager
    delayed_servo_driver = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='robot_arm_control',
                executable='servo_driver',
                name='servo_driver',
                output='screen',
            )
        ]
    )

    nodes = [
        robot_state_pub_node,
        controller_manager,
        delayed_servo_driver,
        delayed_joint_state_broadcaster,
        delayed_robot_controller,
        delay_rviz,
    ]

    return LaunchDescription(nodes)

