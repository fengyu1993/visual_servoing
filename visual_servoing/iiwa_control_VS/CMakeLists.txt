cmake_minimum_required(VERSION 3.0.2)
project(iiwa_control_VS)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  geometry_msgs
  message_filters
  image_process
  cv_bridge
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(realsense2 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  ./include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_control_VS
#  CATKIN_DEPENDS roscpp std_msgs message_runtime 
#  DEPENDS system_lib
)

# add_library(iiwa_control_VS   src/ros_VS.cpp) 


include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR} 
                            ${EIGEN3_INCLUDE_DIRS})
        
# add_executable(iiwa_control_DVS       src/iiwa_control_DVS.cpp)

# target_link_libraries(iiwa_control_DVS    iiwa_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY})

# install(TARGETS iiwa_control_DVS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
