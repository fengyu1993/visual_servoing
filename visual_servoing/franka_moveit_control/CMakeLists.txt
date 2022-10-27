cmake_minimum_required(VERSION 3.0.2)
project(franka_moveit_control)


find_package(catkin REQUIRED COMPONENTS
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  ./include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(franka_moveit_control src/franka_moveit_control.cpp)
add_executable(marker_target_pose src/marker_target_pose.cpp)

target_link_libraries(franka_moveit_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(marker_target_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS franka_moveit_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS marker_target_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
