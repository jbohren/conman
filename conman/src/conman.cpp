/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/Component.hpp>
#include <boost/assign/list_of.hpp>
#include <conman/scheme.h>

//const std::string conman::interfaces::JointState::name = "joint_state";
//const std::string conman::interfaces::JointEffortCommand::name = "joint_effort_command";
//const std::string conman::interfaces::JointEffortFeedforward::name = "joint_effort_feedforward";
//const std::string conman::interfaces::JointEffortFeedback::name = "joint_effort_feedback";

using namespace conman;

const std::vector<Role::ID> Role::ids = 
  boost::assign::list_of
    (Role::ESTIMATION)
    (Role::CONTROL)
    .convert_to_container<std::vector<Role::ID> >();

const std::map<Role::ID,std::string> Role::names = 
  boost::assign::map_list_of
    (Role::ESTIMATION,"ESTIMATION")
    (Role::CONTROL,"CONTROL")
    .convert_to_container<std::map<Role::ID,std::string> >();

ORO_CREATE_COMPONENT_LIBRARY()
