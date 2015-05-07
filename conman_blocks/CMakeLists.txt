cmake_minimum_required(VERSION 2.8.6)

project(conman_blocks)

find_package(catkin REQUIRED cmake_modules)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

orocos_use_package(conman)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

orocos_component(${PROJECT_NAME}
  src/conman_blocks.cpp
  src/vector_sum.cpp
  src/feed_forward_feed_back.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})

orocos_generate_package()

