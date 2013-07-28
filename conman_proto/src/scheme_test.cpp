
#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/main.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

#include <conman_proto/conman.h>
#include <conman_proto/scheme.h>
#include <conman_proto/conman_test_plugins.h>

int ORO_main(int argc, char** argv) {

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  RTT::Logger::In in("Prototype");

  TestEffortController 
    c0_left("c0_left"),//,"left_arm"),
    c1_left("c1_left"),//,"left_arm"),
    c0_right("c0_right");//,"right_arm");

  RTT::Property<std::string>(c0_left.getProperty("group")) = "left_arm";
  RTT::Property<std::string>(c1_left.getProperty("group")) = "left_arm";
  RTT::Property<std::string>(c0_right.getProperty("group")) = "right_arm";

  {
    conman::Scheme scheme("Scheme"); 

    scheme.add_peer(&c0_left);
    scheme.add_peer(&c1_left);
    scheme.add_peer(&c0_right);

    // Create some controllers
    //manager.connectPeers(&c0);
    //manager.connectPeers(&c1);
    //manager.connectPeers(&c2);

    // Get the control groups of a given controller
    RTT::Logger::log() << RTT::Logger::Info << "Control groups: " << RTT::endlog();

    //manager.connect("c0.control.out.left_arm.joint_effort","c1.control.in.left_arm.joint_effort",RTT::ConnPolicy());

    OCL::TaskBrowser task_browser(&scheme);

    task_browser.loop();
  }

  return 0;
}

/**

  connect("c0.left_arm__in.joint_effort","c1.left_arm__out.joint_effort",ConnPolicy())

  **/
