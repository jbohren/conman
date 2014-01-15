/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_ROS_ROS_INTERFACE_SERVICE_H
#define __CONMAN_ROS_ROS_INTERFACE_SERVICE_H

#include <vector>
#include <set>

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <rtt_rosservice/rosservice.h>

#include <conman/conman.h>
#include <conman/scheme.h>

// ROS srv types
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>

namespace conman_ros {
  
  //! This service provides a ROS interface to control a Conman Scheme
  /**
   *  It gets loaded onto a conman Scheme at runtime, and provides ROS
   *  service calls compatible with the ros_control controller_manager.
   */
  class ROSInterfaceService : public RTT::Service 
  {
  public:
    ROSInterfaceService(RTT::TaskContext* owner);

    boost::shared_ptr<RTT::Service> roscontrol;
    boost::shared_ptr<rtt_rosservice::ROSService> rosservice;

    bool listControllerTypesCB(       controller_manager_msgs::ListControllerTypes::Request &req,       controller_manager_msgs::ListControllerTypes::Response& resp);
    bool listControllersCB(           controller_manager_msgs::ListControllers::Request &req,           controller_manager_msgs::ListControllers::Response& resp);
    bool loadControllerCB(            controller_manager_msgs::LoadController::Request &req,            controller_manager_msgs::LoadController::Response& resp);
    bool reloadControllerLibrariesCB( controller_manager_msgs::ReloadControllerLibraries::Request &req, controller_manager_msgs::ReloadControllerLibraries::Response& resp);
    bool switchControllerCB(          controller_manager_msgs::SwitchController::Request &req,          controller_manager_msgs::SwitchController::Response& resp);
    bool unloadControllerCB(          controller_manager_msgs::UnloadController::Request &req,          controller_manager_msgs::UnloadController::Response& resp);

  private:
    conman::Scheme *scheme;

    RTT::OperationCaller<std::vector<std::string>(void)> getBlocks;
    RTT::OperationCaller<std::vector<std::string>(void)> getGroups;
    RTT::OperationCaller<bool(std::vector<std::string>&, std::vector<std::string>&, bool, bool)> switchBlocks;
  };
}

#endif // ifndef __CONMAN_ROS_ROS_INTERFACE_SERVICE_H
