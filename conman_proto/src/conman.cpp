
#include <rtt/Component.hpp>
#include <boost/assign/list_of.hpp>
#include <conman/scheme.h>

//const std::string conman::interfaces::JointState::name = "joint_state";
//const std::string conman::interfaces::JointEffortCommand::name = "joint_effort_command";
//const std::string conman::interfaces::JointEffortFeedforward::name = "joint_effort_feedforward";
//const std::string conman::interfaces::JointEffortFeedback::name = "joint_effort_feedback";

using namespace conman;

const std::vector<Layer::ID> Layer::ids = 
  boost::assign::list_of
    (Layer::ESTIMATION)
    (Layer::CONTROL)
    .convert_to_container<std::vector<Layer::ID> >();

const std::map<Layer::ID,std::string> Layer::names = 
  boost::assign::map_list_of
    (Layer::ESTIMATION,"ESTIMATION")
    (Layer::CONTROL,"CONTROL")
    .convert_to_container<std::map<Layer::ID,std::string> >();

ORO_CREATE_COMPONENT_LIBRARY()
