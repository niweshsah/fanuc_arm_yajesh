cmake_minimum_required(VERSION 3.1.3)
project(moveit_resources_fanuc_moveit_config)

# Find required catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  rospy
  std_msgs
)

# Catkin package configuration
catkin_package(
  CATKIN_DEPENDS moveit_commander moveit_core moveit_ros_planning_interface geometry_msgs rospy std_msgs
)

# Specify locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add Python scripts
catkin_install_python(PROGRAMS
  src/test.py  # Ensure the path is correctly pointing to the Python script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(move_group_cpp_interface src/test.cpp)
target_link_libraries(move_group_cpp_interface ${catkin_LIBRARIES})

# Install additional files like launch and config
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
