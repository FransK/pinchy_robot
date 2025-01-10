cmake_minimum_required(VERSION 3.5)

project(pinchy_robot_application)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. ")
endif()

# find dependecies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
    pinchy_robot_application
    SHARED
    hardware/pinchy.cpp
)
target_compile_features(pinchy_robot_application PUBLIC cxx_std_17)
target_include_directories(pinchy_robot_application PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/pinchy_robot_application>
)
ament_target_dependencies(
    pinchy_robot_application PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface pinchy_robot_application.xml)

# INSTALL
install(
    DIRECTORY hardware/include/
    DESTINATION include/pinchy_robot_application
)
install(TARGETS pinchy_robot_application
  EXPORT export_pinchy_robot_application
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_pinchy_robot_application HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
