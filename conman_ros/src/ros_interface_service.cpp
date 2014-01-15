/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include <rtt/deployment/ComponentLoader.hpp>

#include "ros_interface_service.h"

using namespace conman_ros;

ROSInterfaceService::ROSInterfaceService(RTT::TaskContext* owner) :
  RTT::Service("conman_ros",owner),
  scheme(dynamic_cast<conman::Scheme*>(owner))
{ 
  // Make sure we're attached to a scheme
  if(!scheme) { 
    std::string err_text = "Attmpted to load the Conman ROS interface on a component which isn't a scheme!";
    RTT::log(RTT::Error) << err_text << RTT::endlog();
    throw std::runtime_error(err_text);
  }

  // Create ros-control operation bindings
  roscontrol = owner->provides("roscontrol");
  roscontrol->addOperation("listControllerTypes", &ROSInterfaceService::listControllerTypesCB, this);
  roscontrol->addOperation("listControllers", &ROSInterfaceService::listControllersCB, this);
  roscontrol->addOperation("loadController", &ROSInterfaceService::loadControllerCB, this);
  roscontrol->addOperation("reloadControllerLibraries", &ROSInterfaceService::reloadControllerLibrariesCB, this);
  roscontrol->addOperation("switchController", &ROSInterfaceService::switchControllerCB, this);
  roscontrol->addOperation("unloadController", &ROSInterfaceService::unloadControllerCB, this);

  // Load the rosservice service
  //RTT::plugin::PluginLoader::Instance()->loadService("rosservice",owner);

  rosservice = owner->getProvider<rtt_rosservice::ROSService>("rosservice");

  rosservice->connect("roscontrol.listControllerTypes",
                     "controller_manager/list_controller_types",
                     "controller_manager_msgs/ListControllerTypes");

  rosservice->connect("roscontrol.listControllers",
                     "controller_manager/list_controllers",
                     "controller_manager_msgs/ListControllers");
  
  rosservice->connect("roscontrol.loadController",
                     "controller_manager/load_controller",
                     "controller_manager_msgs/LoadController");

  rosservice->connect("roscontrol.reloadControllerLibraries",
                     "controller_manager/reload_controller_libraries",
                     "controller_manager_msgs/ReloadControllerLibraries");

  rosservice->connect("roscontrol.switchController",
                     "controller_manager/switch_controller",
                     "controller_manager_msgs/SwitchController");

  rosservice->connect("roscontrol.unloadController",
                     "controller_manager/unload_controller",
                     "controller_manager_msgs/UnloadController"); 

}

bool ROSInterfaceService::listControllerTypesCB(
    controller_manager_msgs::ListControllerTypes::Request &req,
    controller_manager_msgs::ListControllerTypes::Response& resp)
{

  return false;
}
bool ROSInterfaceService::listControllersCB(
    controller_manager_msgs::ListControllers::Request &req,
    controller_manager_msgs::ListControllers::Response& resp)
{
  const std::vector<std::string> block_names = scheme->getBlocks();

  resp.controller.reserve(block_names.size());

  for(std::vector<std::string>::const_iterator it = block_names.begin();
      it != block_names.end();
      ++it)
  {
    controller_manager_msgs::ControllerState cs;
    cs.name = *it;
    cs.type = "OROCOS COMPONENT";
    resp.controller.push_back(cs);
  }

  return true;
}
bool ROSInterfaceService::loadControllerCB(
    controller_manager_msgs::LoadController::Request &req,
    controller_manager_msgs::LoadController::Response& resp)
{
  return false;
}
bool ROSInterfaceService::reloadControllerLibrariesCB(
    controller_manager_msgs::ReloadControllerLibraries::Request &req,
    controller_manager_msgs::ReloadControllerLibraries::Response& resp)
{
  return false;
}
bool ROSInterfaceService::switchControllerCB(
    controller_manager_msgs::SwitchController::Request &req,
    controller_manager_msgs::SwitchController::Response& resp)
{
  return false;
}
bool ROSInterfaceService::unloadControllerCB(
    controller_manager_msgs::UnloadController::Request &req,
    controller_manager_msgs::UnloadController::Response& resp)
{
  return false;
}

ORO_SERVICE_NAMED_PLUGIN(conman_ros::ROSInterfaceService, "conman_ros");


