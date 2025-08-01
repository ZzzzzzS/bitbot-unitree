from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch CLI arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz",
        )
    )

    rviz = LaunchConfiguration("rviz")

    gazebo_world_file = PathJoinSubstitution(
        [
            FindPackageShare("bitbot_gz"),
            "world",
            "empty.sdf",
        ]
    )
    gazebo_gui_config = PathJoinSubstitution(
        [
            FindPackageShare("bitbot_gz"),
            "world",
            "gui.config",
        ]
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            (
                "gz_args",
                ["-r -v 3 ", gazebo_world_file, " --gui-config ", gazebo_gui_config],
            ),
        ],
    )

    gazebo_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    gz_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "hhfc",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.91",
            "-Y",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
        ],
    )

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

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("bitbot_gz"),
            "config",
            "hhfc_gz_controllers.yaml",
        ]
    )
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
        output="screen",
        parameters=[robot_description],
    )
    jsb_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    jtc_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            # "joint_trajectory_controller",
            "effort_controller",
            "--param-file",
            robot_controllers_config,
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    set_gz_env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[get_package_share_directory("bitbot_gz").rsplit("bitbot_gz", 1)[0]],
    )

    nodes = [
        set_gz_env_var,
        gazebo_gui,
        gazebo_bridge_node,
        robot_state_publisher_node,
        gz_spawn_entity_node,
        jsb_spawner_node,
        jtc_spawner_node,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)
