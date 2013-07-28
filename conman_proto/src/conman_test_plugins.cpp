
#include <conman_proto/conman_test_plugins.h>

ORO_LIST_COMPONENT_TYPE(TestEffortController)

TestEffortController::TestEffortController(std::string const& name) :
  conman::Block(name),
  group_("")
{
  // Create RTT ports
  using namespace conman;
  using namespace conman::interfaces;

  this->addProperty("group",group_);

}

bool TestEffortController::configureHook() {
  // Construct conman ports
  this->add_conman_port("control",group_,"in", conman::interfaces::SingleJointEffort::name, EXCLUSIVE, effort_in_)
    .doc("Effort input.");
  this->add_conman_port("control",group_,"out", conman::interfaces::SingleJointEffort::name, EXCLUSIVE, effort_out_)
    .doc("Effort output = input + 1.");

  return true;
}

bool TestEffortController::startHook() {

  // Ready if the input is connected
  bool ready = effort_in_.connected();

  return ready;
}


void TestEffortController::compute_control(
    RTT::os::TimeService::Seconds secs, 
    RTT::os::TimeService::Seconds period) 
{
  conman::interfaces::SingleJointEffort::datatype effort;
  effort_in_.read(effort);
  effort_out_.write(effort + 1);
}
