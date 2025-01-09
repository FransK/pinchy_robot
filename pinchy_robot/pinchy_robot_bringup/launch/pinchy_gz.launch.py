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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz automatically with this launch file.",
        )
    )

    # Initialize Arguments
    rviz = LaunchConfiguration("rviz")

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "pinchy_system_position",
            "-allow_remaining",
            "true",
        ]
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
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pinchy_robot_bringup"),
            "config",
            "pinchy_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pinchy_robot_bringup"), "config", "pinchy.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            robot_description,
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )

    nodes = [
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

    # Bridge ROS topics and Gazebo messages for establishing communication
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(pkg_project_bringup, 'config', 'pinchy_bridge.yaml'),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     }],
    #     output='screen'
    # )