
#include <conman_proto/conman_test_plugins.h>

ORO_LIST_COMPONENT_TYPE(TestEffortController)

TestEffortController::TestEffortController(std::string const& name) :
  conman::Block(name)
{
  // Create RTT ports
  //using namespace conman;
  //using namespace conman::interfaces;
}

bool TestEffortController::configureHook() {
  // Construct conman ports
  this->registerConmanPort("control",EXCLUSIVE, this->addPort("effort_in",effort_in_))
    .doc("Effort input.");
  this->registerConmanPort("control",EXCLUSIVE, this->addPort("effort_out",effort_out_))
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
  double effort;
  effort_in_.read(effort);
  effort_out_.write(effort + 1);
}
