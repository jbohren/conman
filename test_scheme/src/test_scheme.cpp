/** Copyright (c) 2013, Jonathan Bohren, all rights reserved.
 * * This software is released under the BSD 3-clause license, for the details of
 * * this license, please see LICENSE.txt at the root of this repository.
 * */

#include <iostream>

#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/startstop.h>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <ocl/DeploymentComponent.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>

#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <rtt/RTT.hpp>
#include <rtt/scripting/Scripting.hpp>

#include <actionlib/action_definition.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

#include <conman/conman.h>
#include <conman/scheme.h>
#include <conman/hook.h>

#include <boost/assign/std/vector.hpp>

#include <ros/ros.h>

class IOBlock : public RTT::TaskContext {
public:
  RTT::InputPort<double> in;

  RTT::OutputPort<double> out;

  IOBlock(int argc, char** argv, const std::string &name) : RTT::TaskContext(name) {
    this->addPort("in",in);

    this->addPort("out",out);

    conman_hook_ = conman::Hook::GetHook(this);
  }
  boost::shared_ptr<conman::Hook> conman_hook_;
};

int main(int argc, char** argv) {

  // Initialize Orocos
  __os_init(argc, argv);

  OCL::DeploymentComponent deployer("deployer");
  RTT::Logger::Instance()->setLogLevel(RTT::Logger::Debug);
  deployer.import("rtt_ros");
  deployer.import("rtt_rosnode");
  deployer.getProvider<RTT::Scripting>("scripting")->eval("ros.import(\"conman\")");
  deployer.getProvider<RTT::Scripting>("scripting")->eval("ros.import(\"conman_ros\")"); 

  conman::Scheme scheme("scheme");
  scheme.connectPeers(&deployer);
  scheme.loadService("conman_ros");
  
  IOBlock iob1(argc, argv, "iob1");
  IOBlock iob2(argc, argv, "iob2");
  IOBlock iob3(argc, argv, "iob3");
  IOBlock iob4(argc, argv, "iob4");
  IOBlock iob5(argc, argv, "iob5");

  iob1.out.connectTo(&iob2.in);
  iob2.out.connectTo(&iob3.in);
  iob3.out.connectTo(&iob4.in);
  iob4.out.connectTo(&iob5.in);
  iob5.out.connectTo(&iob1.in);

  scheme.addBlock(&iob1);
  scheme.addBlock(&iob2);
  scheme.addBlock(&iob3);
  scheme.addBlock(&iob4);
  scheme.addBlock(&iob5);

  scheme.latchConnections("iob5","iob1",true);

  scheme.setGroupMembers("even", "iob2");
  scheme.addToGroup("iob4", "even");

  scheme.addGroup("odd");
  scheme.addToGroup("iob1", "odd");
  scheme.addToGroup("iob3", "odd");
  scheme.addToGroup("iob5", "odd");

  scheme.addGroup("other");
  scheme.addToGroup("iob5", "other");
  scheme.addToGroup("iob4", "other");

  std::vector<std::string> execution_order;
  scheme.getExecutionOrder(execution_order);
  std::vector<std::string> &ptr_blocks = execution_order;

  scheme.configure();

  scheme.start();

  scheme.enableBlocks(ptr_blocks, true, true);

  OCL::TaskBrowser browse(&deployer);
  browse.loop();

  return 0;
}
