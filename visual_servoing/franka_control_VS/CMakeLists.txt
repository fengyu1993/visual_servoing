cmake_minimum_required(VERSION 3.0.2)
project(franka_control_VS)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
find_package(Franka REQUIRED)
find_package(realsense2 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  ./include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES franka_control_VS
#  CATKIN_DEPENDS roscpp std_msgs message_runtime 
#  DEPENDS system_lib
)

add_library(franka_control_VS src/ros_VS.cpp 
                              src/ros_DVS.cpp
                              src/ros_DCT_VS.cpp
                              src/ros_TM_VS.cpp
                              src/ros_KM_VS.cpp
                              src/ros_HM_VS.cpp) 


include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR} 
                            ${EIGEN3_INCLUDE_DIRS}
                            ${FRANKA_INCLUDE_DIRS})
        
add_executable(franka_control_DVS       src/franka_control_DVS.cpp)
add_executable(franka_control_DCT_VS    src/franka_control_DCT_VS.cpp)
add_executable(franka_control_TM_VS     src/franka_control_TM_VS.cpp)
add_executable(franka_control_KM_VS     src/franka_control_KM_VS.cpp)
add_executable(franka_control_HM_VS     src/franka_control_HM_VS.cpp)
add_executable(save_pose_image          src/save_pose_jointangle_image.cpp)
add_executable(publish_realsense        src/publish_realsense.cpp)

target_link_libraries(franka_control_DVS    franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY})
target_link_libraries(franka_control_DCT_VS franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY})
target_link_libraries(franka_control_TM_VS  franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY})
target_link_libraries(franka_control_KM_VS  franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY})
target_link_libraries(franka_control_HM_VS  franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY})
target_link_libraries(save_pose_image  franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY}) 
target_link_libraries(publish_realsense  franka_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY}) 


install(TARGETS franka_control_DVS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_DCT_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_TM_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_KM_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_HM_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS save_pose_image DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS publish_realsense DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})