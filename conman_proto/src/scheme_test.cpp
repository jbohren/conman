
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
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  RTT::Logger::In in("Prototype");

  {
    OCL::DeploymentComponent deployer("Deployer");
    deployer.import("conman_proto");

  TestEffortController 
    left_1("left_1"),
    left_2("left_2"),
    right_1("right_1");

    conman::Scheme scheme("Scheme"); 

    // Connect some stuff
    // left_2 --> left_1 --> 
    left_2.getPort("effort_out")->connectTo( left_1.getPort("effort_in"));
    left_1.getPort("effort_out")->connectTo( right_1.getPort("effort_in"));
    right_1.getPort("effort_out")->connectTo( left_2.getPort("effort_in"));

    // Add the blocks
    scheme.add_block(&left_1);
    scheme.add_block(&right_1);
    scheme.add_block(&left_2);

    //RTT::Logger::log() << RTT::Logger::Info << "Control groups: " << RTT::endlog();

    //manager.connect("c0.control.out.left_arm.joint_effort","c1.control.in.left_arm.joint_effort",RTT::ConnPolicy());

    scheme.connectPeers(&deployer);

    OCL::TaskBrowser task_browser(&scheme);

    task_browser.loop();
  }

  return 0;
}

/**

  connect("c0.left_arm__in.joint_effort","c1.left_arm__out.joint_effort",ConnPolicy())

  **/
