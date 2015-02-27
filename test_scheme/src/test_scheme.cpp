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

class IOBlock : public RTT::TaskContext {
public:
  RTT::InputPort<double> in;
  RTT::InputPort<double> in_ex;

  RTT::OutputPort<double> out1;
  RTT::OutputPort<double> out2;

  IOBlock(const std::string &name) : RTT::TaskContext(name) {
    this->addPort("in",in);
    this->addPort("in_ex",in_ex);

    this->addPort("out1",out1);
    this->addPort("out2",out2);

    conman_hook_ = conman::Hook::GetHook(this);
    conman_hook_->setInputExclusivity("in_ex",conman::Exclusivity::EXCLUSIVE);
  }
  boost::shared_ptr<conman::Hook> conman_hook_;
};

int main(int argc, char** argv) {

  // Initialize Orocos
  __os_init(argc, argv);

  OCL::DeploymentComponent deployer("deployer");
  RTT::Logger::Instance()->setLogLevel(RTT::Logger::Debug);
  deployer.import("rtt_ros");
  deployer.getProvider<RTT::Scripting>("scripting")->eval("ros.import(\"conman\")");
  deployer.getProvider<RTT::Scripting>("scripting")->eval("ros.import(\"conman_ros\")");

  //deployer.getProvider<RTT::Scripting>("scripting")->eval("ros.import(\"conman_ros_node\")");
  //deployer.import("rtt_ros_node");  

  conman::Scheme scheme("scheme");

  IOBlock iob1("iob1");
  IOBlock iob2("iob2");
  IOBlock iob3("iob3");
  IOBlock iob4("iob4");
  IOBlock iob5("iob5");
  scheme.addBlock(&iob1);
  scheme.addBlock(&iob2);
  scheme.addBlock(&iob3);
  scheme.addBlock(&iob4);
  scheme.addBlock(&iob5);
  iob1.out1.connectTo(&iob2.in);
  iob2.out2.connectTo(&iob3.in);
  iob3.out1.connectTo(&iob4.in);
  iob4.out1.connectTo(&iob5.in);
  iob5.out1.connectTo(&iob1.in);
 
  scheme.latchConnections("iob5","iob1",true);

  std::vector<std::string> execution_order;
  scheme.getExecutionOrder(execution_order);
  std::vector<std::string> &ptr_blocks = execution_order;

  scheme.loadService("conman_ros");
  scheme.configure();

  scheme.start();

  scheme.enableBlocks(ptr_blocks, true, true);

  OCL::TaskBrowser browse(&deployer);
  browse.loop();

  return 0;
}
