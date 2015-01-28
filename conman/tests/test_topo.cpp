/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/startstop.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

#include <conman/conman.h>
#include <conman/scheme.h>
#include <conman/hook.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <gtest/gtest.h>
#include <gmock/gmock.h>
using ::testing::ElementsAre;


class InvalidBlock : public RTT::TaskContext {
public:
  InvalidBlock(const std::string &name) : RTT::TaskContext(name) { }
};

class ValidBlock : public RTT::TaskContext {
public:
  ValidBlock(const std::string &name) : RTT::TaskContext(name) { 
    conman_hook_ = conman::Hook::GetHook(this);
  }
  boost::shared_ptr<conman::Hook> conman_hook_;
};

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

class SchemeTest : public ::testing::Test {
protected:
  SchemeTest() : scheme("Scheme") { }

  conman::Scheme scheme;
};

class TopoTest : public SchemeTest { 
public:
  IOBlock iob1;
  IOBlock iob2;
  IOBlock iob3;
  IOBlock iob4;
  IOBlock iob5;

  // Expected cycles
  std::vector<std::string> c1;//,c2,c3,c4;

  TopoTest() : SchemeTest(),
    iob1("iob1"),
    iob2("iob2"),
    iob3("iob3"),
    iob4("iob4"),
    iob5("iob5")
  {
    // Expected cycles
    c1 += "iob1", "iob2", "iob3", "iob4", "iob5";
    //c2 += "iob1", "iob3", "iob4", "iob5";
    //c3 += "iob1", "iob5";
    //c4 += "iob2", "iob3", "iob4", "iob5";
  }

  void AddBlocks() {
    scheme.addBlock(&iob1);
    scheme.addBlock(&iob2);
    scheme.addBlock(&iob3);
    scheme.addBlock(&iob4);
    scheme.addBlock(&iob5);
  }

  void ConnectBlocksAcyclic() {
    iob1.out1.connectTo(&iob2.in);
    //iob1.out2.connectTo(&iob3.in_ex);
    //iob2.out1.connectTo(&iob3.in_ex);
    iob2.out2.connectTo(&iob3.in);
    iob3.out1.connectTo(&iob4.in);
    //iob1.out1.connectTo(&iob5.in);
    iob4.out1.connectTo(&iob5.in);
  }

  void ConnectBlocksCyclic() {
    iob5.out1.connectTo(&iob1.in);
    //iob5.out2.connectTo(&iob2.in);
  }

  void PrintCycles(std::vector<std::vector<std::string> > &cycles) {
    std::cerr<<"cycles: "<<std::endl;
    for(size_t i=0; i<cycles.size(); i++) {
      std::cerr<<"  [";
      for(size_t v=0; v < cycles[i].size(); v++) {
        std::cerr<<" "<<cycles[i][v];
      }
      std::cerr<<" ]"<<std::endl;
    }
  }
};

TEST_F(TopoTest, StartTopo) {
  std::vector<std::vector<std::string> > flow_cycles, exec_cycles;

  // Connect blocks with cycles
  ConnectBlocksAcyclic();
  ConnectBlocksCyclic();
  AddBlocks();

  EXPECT_TRUE(scheme.latchConnections("iob5","iob1",true));
  EXPECT_EQ(1,scheme.getFlowCycles(flow_cycles));
  EXPECT_EQ(0,scheme.getExecutionCycles(exec_cycles));
  EXPECT_TRUE(scheme.executable());

  std::vector<std::string> execution_order;

  EXPECT_TRUE(scheme.getExecutionOrder(execution_order));

  EXPECT_THAT(execution_order, ElementsAre("iob1", "iob2", "iob3", "iob4", "iob5"));

  //OK GAME TIME
  scheme.start();
  scheme.stop();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  //RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  // Import conman plugin
  RTT::ComponentLoader::Instance()->import("conman", "" );

  return RUN_ALL_TESTS();
}
