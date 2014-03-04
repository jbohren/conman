cmake_minimum_required(VERSION 2.8.3)
project(conman)

find_package(catkin REQUIRED)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE)
  add_definitions(--coverage)
  link_libraries(gcov)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )

add_definitions(-DRTT_COMPONENT)
orocos_library(conman
  src/conman.cpp 
  src/scheme.cpp )

orocos_plugin(conman_hook
  src/hook_service.cpp )
target_link_libraries(conman_hook conman)

orocos_component(conman_components
  src/conman_components.cpp )
target_link_libraries(conman_components conman)

orocos_component(conman_test_components
  src/conman_test_plugins.cpp)
target_link_libraries(conman_test_components conman)

orocos_executable(scheme_test src/scheme_test.cpp)
target_link_libraries(scheme_test 
  conman
  conman_hook
  conman_components
  conman_test_components 
  ${USE_OROCOS_LIBRARIES})

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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  find_library(GMOCK_LIBRARY REQUIRED NAMES gmock)

  catkin_add_gtest(test_conman tests/test_conman.cpp)
  target_link_libraries(test_conman 
    conman 
    conman_hook 
    conman_components 
    ${catkin_LIBRARIES} 
    ${GMOCK_LIBRARY}
    ${USE_OROCOS_LIBRARIES})

  if (ENABLE_COVERAGE)
    list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
    include(CodeCoverage)
    setup_target_for_coverage(
      ${PROJECT_NAME}_coverage 
      test_conman 
      ${PROJECT_BINARY_DIR}/coverage)
  endif()  

endif()

