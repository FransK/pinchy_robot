from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=use_sim_time,
            description="If true, use simulated clock"),
    )

    #ros2 control
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pinchy_robot_bringup"),
            "config",
            "pinchy_controllers.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--param-file", robot_controllers],
    )

    nodes = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)