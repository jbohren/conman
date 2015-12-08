cmake_minimum_required(VERSION 2.8.3)
project(rtt_conman_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm conman_msgs)

ros_generate_rtt_typekit(conman_msgs)
ros_generate_rtt_service_proxies(conman_msgs)

orocos_generate_package(
  DEPENDS conman_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_actionlib rtt_actionlib_msgs
)
