cmake_minimum_required(VERSION 2.8.3)
project(conman_ros)

find_package(catkin REQUIRED COMPONENTS controller_manager_msgs)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Generate ROS messages
#add_message_files(FILES
  #SemiAbsoluteCalibrationState.msg)
#add_service_files(FILES
  #Calibrate.srv)
#generate_messages(DEPENDENCIES std_msgs)

# Generate RTT typekit
#ros_generate_rtt_typekit(${PROJECT_NAME})
#ros_generate_rtt_service_proxies(${PROJECT_NAME})

orocos_use_package(conman)
orocos_use_package(rtt_roscomm)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

add_definitions(-DRTT_COMPONENT)
orocos_service(${PROJECT_NAME}
  src/ros_interface_service.cpp
  )

target_link_libraries(
  ${PROJECT_NAME}
  ${USE_OROCOS_LIBRARIES} 
  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

orocos_generate_package(
  DEPENDS ${PROJECT_NAME}
  DEPENDS_TARGETS rtt_roscomm)



