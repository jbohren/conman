/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include "ros_interface_service.h"

#include <rtt/deployment/ComponentLoader.hpp>

using namespace conman;

ORO_SERVICE_NAMED_PLUGIN(conman_ros::ROSInterfaceService, "conman_ros");

ROSInterfaceService::ROSInterfaceService(RTT::TaskContext* owner) :
  RTT::Service("conman_ros",owner),
  // Property Initialization
  execution_period_(0.0),
  output_ports_by_layer_(conman::Layer::ids.size())
{ 
  // Load other plugins
/*
 *  boost::shared_ptr<RTT::ComponentLoader> component_loader =
 *    RTT::ComponentLoader::Instance();
 *
 *  component_loader->import("rtt_ro","");
 *  component_loader->import("rtt_roscomm","");
 *  component_loader->import("rtt_actionlib","");
 */

  // Load the rosservice service
  RTT::plugin::PluginLoader::Instance()->loadService("rosservice",owner);

}


