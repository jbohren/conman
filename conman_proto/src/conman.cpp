
#include <rtt/Component.hpp>

#include <conman_proto/scheme.h>

const std::string conman::interfaces::JointState::name = "joint_state";
const std::string conman::interfaces::JointEffortCommand::name = "joint_effort_command";
const std::string conman::interfaces::JointEffortFeedforward::name = "joint_effort_feedforward";
const std::string conman::interfaces::JointEffortFeedback::name = "joint_effort_feedback";

ORO_CREATE_COMPONENT_LIBRARY()
