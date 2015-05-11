cmake_minimum_required(VERSION 2.8.3)
project(conman_examples)

find_package(catkin REQUIRED COMPONENTS
  conman
  roscpp
  rtt_ros
  actionlib
  actionlib_msgs
  genmsg
  rtt_roscomm
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_action_files(DIRECTORY action FILES SomeAction.action)
#generate_messages(DEPENDENCIES actionlib_msg)

ros_generate_rtt_typekit(${PROJECT_NAME})
catkin_package()


find_package(catkin REQUIRED)

find_package(OROCOS-RTT REQUIRED)

#find_package(conman_hook)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS}, ${USE_OROCOS_INCLUDE_DIRS})
orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )

#orocos_plugin(conman_hook
#    src/hook_service.cpp )
#target_link_libraries(conman_hook conman)

#orocos_component(conman_components
#    src/conman_components.cpp )
#target_link_libraries(conman_components conman)

add_definitions(-DRTT_COMPONENT)

add_executable(conman_ros_example src/conman_ros_example.cpp)

orocos_generate_package(
  INCLUDE_DIRS include
  )

target_link_libraries(conman_ros_example
    ${catkin_LIBRARIES}
    ${GMOCK_LIBRARY}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}
    ${USE_OROCOS_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )


