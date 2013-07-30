
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
    left_1("left_1"),//,"left_arm"),
    left_2("left_2"),//,"left_arm"),
    right_1("right_1");//,"right_arm");

  //RTT::Property<std::string>(c0_left.getProperty("group")) = "left_arm";
  //RTT::Property<std::string>(c1_left.getProperty("group")) = "left_arm";
  //RTT::Property<std::string>(c0_right.getProperty("group")) = "right_arm";

  {
    conman::Scheme scheme("Scheme"); 

    ConnPolicy policy = RTT::ConnPolicy::buffer(10);
    left_1.getPort("effort_out").connectTo( &left_2.getPort("effort_in"), policy );
    left_2.getPort("effort_out").connectTo( &right_1.getPort("effort_in"), policy );

    scheme.add_peer(&left_1);
    scheme.add_peer(&left_2);
    scheme.add_peer(&right_1);

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
