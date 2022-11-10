cmake_minimum_required(VERSION 3.0.2)
project(image_process)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  ./include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_process
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

add_library(DVS src/direct_visual_servoing.cpp src/visual_servoing.cpp)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR} 
                            ${EIGEN3_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS})


add_executable(read_image_realsense src/read_image_realsense.cpp)
add_executable(test_DVS src/test_DVS.cpp)

target_link_libraries(read_image_realsense  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
target_link_libraries(test_DVS  DVS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})


install(TARGETS read_image_realsense DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_DVS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
