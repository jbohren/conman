/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

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

#include <conman/conman.h>
#include <conman/scheme.h>
#include <conman/conman_test_plugins.h>

int ORO_main(int argc, char** argv) {

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  RTT::Logger::In in("Prototype");

  {
    OCL::DeploymentComponent deployer("Deployer");
    deployer.import("conman");

  TestEffortController 
    A("A"),
    B("B"),
    C("C"),
    D("D");

    conman::Scheme scheme("Scheme"); 
    scheme.connectPeers(&deployer);


    // Connect some stuff
    // left_2 --> left_1 --> right_1 --X--> left_2
    D.getPort("effort_out")->connectTo(A.getPort("effort_in"));
    A.getPort("effort_out")->connectTo(B.getPort("effort_in"));
    A.getPort("effort_out")->connectTo(C.getPort("effort_in"));
    B.getPort("effort_out")->connectTo(C.getPort("effort_in"));
    C.getPort("effort_out")->connectTo(D.getPort("effort_in"));

    // Add the blocks
    scheme.addBlock(&C);
    scheme.addBlock(&D);
    scheme.addBlock(&B);
    scheme.addBlock(&A);

    if(scheme.hasCycles()) {
      RTT::log(RTT::Info) << "Scheme has cycles!" << RTT::endlog();
    }

    // conman::graph::Cycles cycles = scheme.getCycles();
    scheme.latchConnections(&C,&D);

    if(!scheme.hasCycles()) {
      RTT::log(RTT::Info) << "Scheme no longer has cycles!" << RTT::endlog();
    }

    std::vector<std::string> group_bc;
    group_bc.push_back("B");
    group_bc.push_back("C");
    scheme.createGroup("BC",group_bc);

    //RTT::Logger::log() << RTT::Logger::Info << "Control groups: " << RTT::endlog();
    //manager.connect("c0.control.out.left_arm.joint_effort","c1.control.in.left_arm.joint_effort",RTT::ConnPolicy());

    OCL::TaskBrowser task_browser(&scheme);

    task_browser.loop();
  }

  return 0;
}

/**

  connect("c0.left_arm__in.joint_effort","c1.left_arm__out.joint_effort",ConnPolicy())

  **/
