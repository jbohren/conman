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

#include <conman/conman.h>

namespace conman_ros {
  
  //! This service provides a ROS interface to control a Conman Scheme
  class ROSInterfaceService : public RTT::Service 
  {
  public:
    ROSInterfaceService(RTT::TaskContext* owner);

    boost::shared_ptr<RTT::Service> rosservice_;
  };
}

#endif // ifndef __CONMAN_ROS_ROS_INTERFACE_SERVICE_H
