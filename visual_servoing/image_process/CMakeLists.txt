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

add_library(DVS src/direct_visual_servoing.cpp 
                src/visual_servoing.cpp
                src/discrete_orthogonal_moment_vs.cpp
                src/discrete_cosine_transform_vs.cpp
                src/Tchebichef_moments_vs.cpp
                src/Krawtchouk_moments_vs.cpp
                src/Hahn_moments_vs.cpp) 

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR} 
                            ${EIGEN3_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS})


add_executable(test_DVS     src/test/test_DVS.cpp)
add_executable(test_DCT_VS  src/test/test_DCT_VS.cpp)
add_executable(test_TM_VS   src/test/test_TM_VS.cpp)
add_executable(test_KM_VS   src/test/test_KM_VS.cpp)
add_executable(test_HM_VS   src/test/test_HM_VS.cpp)


target_link_libraries(test_DVS  DVS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
target_link_libraries(test_DCT_VS  DVS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
target_link_libraries(test_TM_VS  DVS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
target_link_libraries(test_KM_VS  DVS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
target_link_libraries(test_HM_VS  DVS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_LIBRARIES})


install(TARGETS test_DVS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_DCT_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_TM_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_KM_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_HM_VS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
