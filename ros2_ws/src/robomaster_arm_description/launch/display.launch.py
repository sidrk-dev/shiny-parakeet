from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    gui = LaunchConfiguration("gui")

    robot_description_content = ParameterValue(Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("robomaster_arm_description"),
            "urdf",
            "robot_arm.urdf.xacro",
        ]),
        " serial_port:=",
        serial_port,
        " baud_rate:=",
        baud_rate,
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud_rate", default_value="921600"),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(gui),
            output="screen",
        ),
    ])
