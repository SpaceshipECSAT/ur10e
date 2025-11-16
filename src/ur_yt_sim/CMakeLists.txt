cmake_minimum_required(VERSION 3.8)
project(ur_yt_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(linkattacher_msgs REQUIRED)
find_package(ros2_linkattacher REQUIRED)

find_package(gazebo_msgs REQUIRED) # this is for auto dock node
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(test_ik src/test_ik.cpp)
# ament_target_dependencies(test_ik rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs)

add_executable(test_world3 src/test_world3.cpp)
ament_target_dependencies(test_world3 rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs linkattacher_msgs ros2_linkattacher)

add_executable(test1 src/test1.cpp)
ament_target_dependencies(test1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

add_executable(test1_redo src/test1_redo.cpp)
ament_target_dependencies(test1_redo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

add_executable(aruco_docking_node src/aruco_docking_node.cpp)
ament_target_dependencies(aruco_docking_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

# add_executable(auto_dock_node src/auto_dock_node.cpp)
# ament_target_dependencies(auto_dock_node rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs linkattacher_msgs ros2_linkattacher tf2_geometry_msgs gazebo_msgs)

# add_executable(pick src/pick.cpp)
# ament_target_dependencies(pick rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs linkattacher_msgs ros2_linkattacher)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)



install(DIRECTORY launch urdf config meshes worlds rviz models
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
#   auto_dock_node
#   pick
#   test_ik
  test1_redo
  test_world3
  test1
  aruco_docking_node
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
