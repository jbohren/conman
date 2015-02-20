cmake_minimum_required(VERSION 2.8.3)
project(test_scheme)

find_package(catkin REQUIRED COMPONENTS
  conman
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )

add_definitions(-DRTT_COMPONENT)

add_executable(test_scheme src/test_scheme.cpp)

orocos_generate_package(
  INCLUDE_DIRS include
  )

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
