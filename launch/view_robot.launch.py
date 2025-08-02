from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("bitbot_gz"),
                    "urdf",
                    "hhfc",
                    "urdf",
                    "hhfc.urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_file}

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("bitbot_gz"),
            "rviz",
            "hhfc.rviz",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    jspg_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        jspg_node,
    ]
    return LaunchDescription(nodes)
