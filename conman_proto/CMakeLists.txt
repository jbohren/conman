cmake_minimum_required(VERSION 2.8.3)
project(conman_proto)

find_package(catkin REQUIRED)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories(include)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES conman_proto
  #CATKIN_DEPENDS other_catkin_pkg
  DEPENDS rtt
)

###########
## Build ##
###########

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )

#orocos_plugin(conman_plugin src/conman.cpp src/manager.cpp)
#orocos_plugin(conman_test_plugins src/conman_test_plugins.cpp)

#orocos_executable(port_dag src/port_dag.cpp)
add_definitions(-DRTT_COMPONENT)
orocos_plugin(conman 
  src/scheme.cpp 
  src/conman.cpp 
  src/hook_service.cpp )

orocos_component(conman_test_components
  src/conman_test_plugins.cpp)

orocos_executable(scheme_test src/scheme_test.cpp)
target_link_libraries(scheme_test conman conman_test_components)

orocos_generate_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(conman_proto
#   src/${PROJECT_NAME}/conman_proto.cpp
# )

## Declare a cpp executable
# add_executable(conman_proto_node src/conman_proto_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(conman_proto_node conman_proto_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(conman_proto_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS conman_proto conman_proto_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_conman_proto.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
