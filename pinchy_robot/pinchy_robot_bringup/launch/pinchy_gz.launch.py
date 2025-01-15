# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Important constants
    robot_xacro_name = "pinchy"

    # Initialize Arguments
    gui = LaunchConfiguration("gui", default=False)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    use_gazebo = LaunchConfiguration("use_gazebo", default=True)

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value=gui,
            description="Launch RViz automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=use_sim_time,
            description="If true, use simulated clock"),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="true",
            description="Whether to enable gazebo-specific configurations in the robot description.",
        )
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": " -r -v -v4 empty.sdf", "on_exit_shutdown": "true"}.items(),
    )

    bridge_params = PathJoinSubstitution([FindPackageShare("pinchy_robot_bringup"), "config", "pinchy_bridge.yaml"])
    node_gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params
        }]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pinchy_robot_description"),
                    "models",
                    "pinchy.urdf.xacro",
                ]
            ),
            " ",
            "use_gazebo:=", use_gazebo
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_xacro_name,
            "-allow_renaming", "true"
        ]
    )

    #ros2 control
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pinchy_robot_bringup"),
            "config",
            "pinchy_controllers.yaml",
        ]
    )

    ros2_control_ = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager", "--param-file", robot_controllers],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare("pinchy_robot_bringup"), "config", "pinchy.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        node_gazebo_ros_bridge,
        rviz_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_state_publisher,
                on_exit=[ros2_control_],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ros2_control_,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)