cmake_minimum_required(VERSION 3.0.2)
project(robot_control_VS)

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
find_package(Franka REQUIRED)
find_package(realsense2 REQUIRED)

find_package(arena_sdk QUIET)
if (NOT ${arena_sdk_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Findarena_sdk.cmake")
endif()

set(ROSLINT_CPP_OPTS
   "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

add_definitions("-std=gnu++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DIRS
  ./include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_control_VS
#  CATKIN_DEPENDS roscpp std_msgs message_runtime 
#  DEPENDS system_lib
)

add_library(robot_control_VS src/ros_VS.cpp 
                              src/ros_DVS.cpp
                              src/ros_DCT_VS.cpp
                              src/ros_TM_VS.cpp
                              src/ros_KM_VS.cpp
                              src/ros_HM_VS.cpp
                              src/ros_PVS.cpp
                              src/ros_ur_DVS.cpp) 


include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR} 
                            ${EIGEN3_INCLUDE_DIRS}
                            ${FRANKA_INCLUDE_DIRS}
                            ${arena_sdk_INCLUDE_DIRS})

add_executable(franka_control_DVS               src/franka_control_DVS.cpp)
add_executable(franka_control_DCT_VS            src/franka_control_DCT_VS.cpp)
add_executable(franka_control_TM_VS             src/franka_control_TM_VS.cpp)
add_executable(franka_control_KM_VS             src/franka_control_KM_VS.cpp)
add_executable(franka_control_HM_VS             src/franka_control_HM_VS.cpp)
add_executable(franka_save_pose_jointangle_image  src/franka_save_pose_jointangle_image.cpp)
add_executable(franka_control_keyboard          src/franka_control_keyboard.cpp)
add_executable(publish_realsense                src/publish_realsense.cpp)
add_executable(publish_L515                     src/publish_L515.cpp)
add_executable(publish_LUCID                    src/publish_LUCID.cpp)
add_executable(publish_L515_LUCID               src/publish_L515_LUCID.cpp)
add_executable(publish_LUCID_new           src/publish_LUCID_new.cpp)
add_executable(rosbag_record                    src/rosbag_record.cpp)
add_executable(ur_control_PVS                   src/ur_control_PVS.cpp)
add_executable(ur_control_DVS                   src/ur_control_DVS.cpp)
add_executable(ur_save_pose_jointangle_image    src/ur_save_pose_jointangle_image.cpp)
add_executable(ur_control_keyboard              src/ur_control_keyboard.cpp)
add_executable(robot_twist_controller           src/robot_twist_controller.cpp)

target_link_libraries(franka_control_DVS        robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES})
target_link_libraries(franka_control_DCT_VS     robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES})
target_link_libraries(franka_control_TM_VS      robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES})
target_link_libraries(franka_control_KM_VS      robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES})
target_link_libraries(franka_control_HM_VS      robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES})
target_link_libraries(franka_save_pose_jointangle_image  robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(franka_control_keyboard   robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(publish_realsense         robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(publish_L515              robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(publish_LUCID             robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(publish_L515_LUCID        robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(publish_LUCID_new    robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(rosbag_record             robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(ur_control_PVS            robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(ur_control_DVS            robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(ur_save_pose_jointangle_image  robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(ur_control_keyboard  robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 
target_link_libraries(robot_twist_controller    robot_control_VS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${Franka_LIBRARIES} ${realsense2_LIBRARY} ${arena_sdk_LIBRARIES}) 

install(TARGETS franka_control_DVS              DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_DCT_VS           DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_TM_VS            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_KM_VS            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_HM_VS            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_save_pose_jointangle_image DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS franka_control_keyboard         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS publish_realsense               DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS publish_L515                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS publish_LUCID                   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS publish_L515_LUCID              DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS publish_LUCID_new          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS rosbag_record                   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ur_control_PVS                  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ur_control_DVS                  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ur_save_pose_jointangle_image   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ur_control_keyboard             DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS robot_twist_controller          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})