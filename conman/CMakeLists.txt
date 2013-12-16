cmake_minimum_required(VERSION 2.8.3)
project(conman)

find_package(catkin REQUIRED)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES conman
  #CATKIN_DEPENDS other_catkin_pkg
  DEPENDS rtt
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )

add_definitions(-DRTT_COMPONENT)
orocos_plugin(conman 
  src/conman.cpp 
  src/scheme.cpp 
  src/hook_service.cpp )

orocos_component(conman_test_components
  src/conman_test_plugins.cpp)

orocos_executable(scheme_test src/scheme_test.cpp)
target_link_libraries(scheme_test 
  conman 
  conman_test_components 
  ${USE_OROCOS_LIBRARIES})

orocos_generate_package()

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

  find_library(GMOCK_LIBRARY NAMES gmock)

  catkin_add_gtest(test_conman tests/test_conman.cpp)
  target_link_libraries(test_conman conman ${catkin_LIBRARIES} ${GMOCK_LIBRARY})
endif()

