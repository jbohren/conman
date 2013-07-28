
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

int ORO_main(int argc, char** argv) {

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  RTT::Logger::In in("Prototype");

  std::vector<std::string> components;
  std::map<std::string,std::vector<std::string> > resources; 

  MyEffortController c0("c0","left_arm");
  MyEffortController c1("c1","left_arm");
  MyEffortController c2("c2","left_arm");

  {
    conman::BlockManager manager("BlockManager"); 

    // Create some controllers
    //manager.connectPeers(&c0);
    //manager.connectPeers(&c1);
    //manager.connectPeers(&c2);

    // Get the control groups of a given controller
    RTT::Logger::log() << RTT::Logger::Info << "Control groups: " << RTT::endlog();

    //manager.connect("c0.control.out.left_arm.joint_effort","c1.control.in.left_arm.joint_effort",RTT::ConnPolicy());

    OCL::TaskBrowser task_browser(&manager);

    task_browser.loop();
  }

  return 0;
}

/**

  connect("c0.left_arm__in.joint_effort","c1.left_arm__out.joint_effort",ConnPolicy())

  **/
