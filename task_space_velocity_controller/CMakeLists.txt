cmake_minimum_required(VERSION 3.22)
project(task_space_velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(action_cartesian_trajectory REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_manager REQUIRED)



add_library(velocity_controller SHARED
  src/velocity_controller.cpp
)

ament_target_dependencies(velocity_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  moveit_core
  moveit_ros_planning
  action_cartesian_trajectory
  realtime_tools
  controller_manager
)


target_include_directories(velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


pluginlib_export_plugin_description_file(controller_interface velocity_controller.xml)

install(TARGETS velocity_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)



ament_export_targets(export_task_space_velocity_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
