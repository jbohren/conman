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

TEST_F(SchemeTest, Init) {
  std::vector<std::vector<std::string> > cycles;
  EXPECT_EQ(scheme.getFlowCycles(cycles),0);
  EXPECT_EQ(cycles.size(),0);

  EXPECT_EQ(scheme.getExecutionCycles(cycles),0);
  EXPECT_EQ(cycles.size(),0);

  std::vector<std::string> path;
  EXPECT_EQ(scheme.latchCount(path),0);

  EXPECT_EQ(scheme.maxLatchCount(),0);
  EXPECT_EQ(scheme.minLatchCount(),0);

  EXPECT_EQ(scheme.executable(),true);
}

class BlocksTest : public SchemeTest { };

TEST_F(BlocksTest, GetBlocks) {
  std::vector<std::string> blocks;
  
  blocks = scheme.getBlocks();
  EXPECT_EQ(blocks.size(),0);

  scheme.getBlocks(blocks);
  EXPECT_EQ(blocks.size(),0);
}

TEST_F(BlocksTest, AddBlocks) {
  EXPECT_FALSE(scheme.addBlock(""));
  EXPECT_FALSE(scheme.addBlock("fail"));
  EXPECT_FALSE(scheme.addBlock(NULL));

  InvalidBlock ib1("ib1");
  EXPECT_TRUE(scheme.addBlock(&ib1));
  EXPECT_EQ(scheme.getBlocks().size(),1);

  ValidBlock vb1("vb1");
  EXPECT_FALSE(scheme.addBlock("vb1"));
  EXPECT_TRUE(scheme.addPeer(&vb1));
  EXPECT_TRUE(scheme.addBlock("vb1"));
  EXPECT_EQ(scheme.getBlocks().size(),2);

  ValidBlock vb2("vb2");
  EXPECT_TRUE(scheme.addBlock(&vb2));

  EXPECT_EQ(scheme.getBlocks().size(),3);
}

TEST_F(BlocksTest, RemoveBlocks) {
  EXPECT_FALSE(scheme.removeBlock(""));
  EXPECT_FALSE(scheme.removeBlock("fail"));
  EXPECT_FALSE(scheme.removeBlock(NULL));

  ValidBlock vb1("vb1");
  EXPECT_TRUE(scheme.addPeer(&vb1));
  EXPECT_TRUE(scheme.removeBlock("vb1"));
  EXPECT_EQ(scheme.getBlocks().size(),0);
}

class GroupsTest : public SchemeTest { 
public:
  GroupsTest() : SchemeTest(),
    vb1("vb1"),
    vb2("vb2"),
    vb3("vb3")
  {
    scheme.addBlock(&vb1);
    scheme.addBlock(&vb2);
    scheme.addBlock(&vb3);
  }

  ValidBlock vb1;
  ValidBlock vb2;
  ValidBlock vb3;
};

TEST_F(GroupsTest, GetGroups) {
  EXPECT_FALSE(scheme.hasGroup("fail"));

  std::vector<std::string> members;
  EXPECT_FALSE(scheme.getGroupMembers("fail",members));
}

TEST_F(GroupsTest, AddGroups) {
  EXPECT_TRUE(scheme.addGroup(""));
  EXPECT_TRUE(scheme.addGroup("win"));
  EXPECT_TRUE(scheme.addGroup("win"));
}

TEST_F(GroupsTest, SetGroups) {
  std::vector<std::string> members;

  EXPECT_TRUE(scheme.setGroup("",members));
  EXPECT_TRUE(scheme.setGroup("win",members));

  members.push_back("not_a_peer");
  EXPECT_FALSE(scheme.setGroup("fail",members));

}

TEST_F(GroupsTest, AddToGroups) {
  std::vector<std::string> members, members_get;

  EXPECT_FALSE(scheme.addToGroup("fail",""));

  EXPECT_FALSE(scheme.addToGroup("vb1","win"));
  EXPECT_TRUE(scheme.addGroup("win"));
  EXPECT_TRUE(scheme.addToGroup("vb1","win"));
  EXPECT_TRUE(scheme.addToGroup("vb2","win"));
  // Add it again
  EXPECT_TRUE(scheme.addToGroup("vb2","win"));

  EXPECT_TRUE(scheme.getGroupMembers("win",members_get));
  EXPECT_EQ(members_get.size(),2);

  EXPECT_THAT(members_get, ElementsAre("vb1","vb2"));
}

TEST_F(GroupsTest, NestedGroups) {
  std::vector<std::string> members, members_get;

  EXPECT_TRUE(scheme.setGroup("win1","vb1"));
  EXPECT_TRUE(scheme.setGroup("win2","vb2"));
  EXPECT_TRUE(scheme.setGroup("win3","vb3"));
  EXPECT_TRUE(scheme.addGroup("win4"));

  std::vector<std::string> win123_members;
  win123_members += "win1", "win2", "win3", "win4", "win123";

  EXPECT_TRUE(scheme.setGroup("win123",win123_members));

  EXPECT_TRUE(scheme.getGroupMembers("win123",members_get));
  EXPECT_EQ(members_get.size(),3);
}

TEST_F(GroupsTest, RemoveFromGroups) {
  std::vector<std::string> members, members_get;

  // Add some members
  EXPECT_TRUE(scheme.setGroup("win1","vb1"));
  EXPECT_TRUE(scheme.addToGroup("vb2","win1"));
  EXPECT_TRUE(scheme.getGroupMembers("win1",members_get));
  EXPECT_EQ(members_get.size(),2);

  // Remove a member
  EXPECT_TRUE(scheme.removeFromGroup("win1","vb2"));
  EXPECT_TRUE(scheme.getGroupMembers("win1",members_get));
  EXPECT_EQ(members_get.size(),1);
  
  // Empty the group
  EXPECT_TRUE(scheme.emptyGroup("win1"));
  EXPECT_TRUE(scheme.getGroupMembers("win1",members_get));
  EXPECT_EQ(members_get.size(),0);

  // Empty it again (already empty)
  EXPECT_TRUE(scheme.emptyGroup("win1"));
  EXPECT_TRUE(scheme.getGroupMembers("win1",members_get));
  EXPECT_EQ(members_get.size(),0);
}

class DataFlowTest : public SchemeTest { 
public:
  IOBlock iob1;
  IOBlock iob2;
  IOBlock iob3;
  IOBlock iob4;
  IOBlock iob5;

  // Expected cycles
  std::vector<std::string> c1,c2,c3,c4;

  DataFlowTest() : SchemeTest(),
    iob1("iob1"),
    iob2("iob2"),
    iob3("iob3"),
    iob4("iob4"),
    iob5("iob5")
  {
    // Expected cycles
    c1 += "iob1", "iob2", "iob3", "iob4", "iob5";
    c2 += "iob1", "iob3", "iob4", "iob5";
    c3 += "iob1", "iob5";
    c4 += "iob2", "iob3", "iob4", "iob5";
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
    iob1.out2.connectTo(&iob3.in_ex);
    iob2.out1.connectTo(&iob3.in_ex);
    iob2.out2.connectTo(&iob3.in);
    iob3.out1.connectTo(&iob4.in);
    iob1.out1.connectTo(&iob5.in);
    iob4.out1.connectTo(&iob5.in);
  }

  void ConnectBlocksCyclic() {
    iob5.out1.connectTo(&iob1.in);
    iob5.out2.connectTo(&iob2.in);
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

TEST_F(DataFlowTest, Acyclic) {
  // Connect blocks without cycles
  ConnectBlocksAcyclic();
  AddBlocks();
  EXPECT_TRUE(scheme.executable());
}

TEST_F(DataFlowTest, Cyclic) {
  // Connect blocks without cycles
  ConnectBlocksAcyclic();
  AddBlocks();
  EXPECT_TRUE(scheme.executable());
  // Add some cycles
  ConnectBlocksCyclic();
  // At this point the model is out-of-sync with the actual DFG
  EXPECT_TRUE(scheme.executable());
  scheme.regenerateModel();
  EXPECT_FALSE(scheme.executable());
}

TEST_F(DataFlowTest, GetCycles) {
  // Connect blocks with cycles
  ConnectBlocksAcyclic();
  ConnectBlocksCyclic();
  AddBlocks();
  EXPECT_FALSE(scheme.executable());

  // Get the cycles
  std::vector<std::vector<std::string> > flow_cycles, exec_cycles;

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_THAT(flow_cycles, ElementsAre(c1,c2,c3,c4));

  //PrintCycles(flow_cycles);

  EXPECT_EQ(4,scheme.getExecutionCycles(exec_cycles));
  EXPECT_THAT(exec_cycles, ElementsAre(c1,c2,c3,c4));
}

TEST_F(DataFlowTest, LatchConnections) {
  std::vector<std::vector<std::string> > flow_cycles, exec_cycles;

  // Connect blocks with cycles
  ConnectBlocksAcyclic();
  ConnectBlocksCyclic();
  AddBlocks();

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_THAT(flow_cycles, ElementsAre(c1,c2,c3,c4));
  EXPECT_EQ(4,scheme.getExecutionCycles(exec_cycles));
  EXPECT_THAT(exec_cycles, ElementsAre(c1,c2,c3,c4));

  EXPECT_TRUE(scheme.latchConnections("iob5","iob1",true));

  EXPECT_FALSE(scheme.executable());

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_THAT(flow_cycles, ElementsAre(c1,c2,c3,c4));
  EXPECT_EQ(1,scheme.getExecutionCycles(exec_cycles));
  EXPECT_THAT(exec_cycles, ElementsAre(c4));

  EXPECT_TRUE(scheme.latchConnections("iob5","iob2",true));

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_THAT(flow_cycles, ElementsAre(c1,c2,c3,c4));
  EXPECT_EQ(0,scheme.getExecutionCycles(exec_cycles));
  EXPECT_TRUE(scheme.executable());

  std::vector<std::string> execution_order;

  EXPECT_TRUE(scheme.getExecutionOrder(execution_order));

  EXPECT_THAT(execution_order, ElementsAre("iob1", "iob2", "iob3", "iob4", "iob5"));
}

TEST_F(DataFlowTest, Latchanalysis) {
  std::vector<std::vector<std::string> > flow_cycles, exec_cycles;

  // Connect blocks with cycles
  ConnectBlocksAcyclic();
  ConnectBlocksCyclic();
  AddBlocks();

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_EQ(4,scheme.getExecutionCycles(exec_cycles));

  scheme.latchConnections("iob5","iob1",true);
  scheme.latchConnections("iob5","iob2",true);

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_EQ(0,scheme.getExecutionCycles(exec_cycles));

  std::vector<std::string> path_query1, path_query2;
  path_query1 += "iob1", "iob2", "iob3", "iob4", "iob5";
  EXPECT_EQ(0,scheme.latchCount(path_query1));
  path_query2 += "iob5","iob1";
  EXPECT_EQ(1,scheme.latchCount(path_query2));

  EXPECT_EQ(4,scheme.getFlowCycles(flow_cycles));
  EXPECT_EQ(0,scheme.getExecutionCycles(exec_cycles));

  EXPECT_EQ(1,scheme.maxLatchCount());
  EXPECT_EQ(1,scheme.minLatchCount());
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
