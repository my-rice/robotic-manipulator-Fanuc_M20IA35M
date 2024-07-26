cmake_minimum_required(VERSION 3.22)
project(fanuc_m20ia_35m_planning_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(action_cartesian_trajectory REQUIRED)
find_package(control_msgs REQUIRED)


set(THIS_PACKAGE_DEPENDS
    rclcpp
    rclcpp_components
    control_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_msgs
    trajectory_interfaces
    rosbag2_cpp
    action_cartesian_trajectory
    control_msgs
)

add_executable(fanuc_m20ia_35m_planning_demo 
src/fanuc_moveit_planning_node.cpp
src/action_node.cpp
)
ament_target_dependencies(fanuc_m20ia_35m_planning_demo ${THIS_PACKAGE_DEPENDS})

target_include_directories(fanuc_m20ia_35m_planning_demo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(fanuc_m20ia_35m_planning_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS fanuc_m20ia_35m_planning_demo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_dependencies(
  ${THIS_PACKAGE_DEPENDS}
)

ament_package()

