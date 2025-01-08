# pinchy_robot

A pinchy robot project integrating ROS 2 and Gazebo simulator.

## Included packages

- `pinchy_robot_description` - holds the sdf description of the simulated system and any other assets.

- `pinchy_robot_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

- `pinchy_robot_application` - holds ros2 specific code and configurations.

- `pinchy_robot_bringup` - holds launch files and high level utilities.

## Install

For using the template with Gazebo Fortress switch to the `fortress` branch of this repository, otherwise use the default branch `main` for Gazebo Harmonic onwards.

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation

   Build [`ros_gz`](https://github.com/gazebosim/ros_gz) and [`sdformat_urdf`](https://github.com/ros/sdformat_urdf) from source if binaries are not available for your chosen combination.

1. Install necessary tools

   ```bash
   sudo apt install python3-vcstool python3-colcon-common-extensions git wget
   ```

## Usage

1. Install dependencies

   ```bash
   cd ~/pinchy_ws
   source /opt/ros/$ROS_DISTRO/setup.bash
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
   ```

1. Build the project

   ```bash
   colcon build --cmake-args -DBUILD_TESTING=ON
   ```

1. Source the workspace

   ```bash
   . ~/template_ws/install/setup.sh
   ```

1. Launch the simulation

   ```bash
   ros2 launch pinchy_robot_bringup pinchy.launch.py
   ```
