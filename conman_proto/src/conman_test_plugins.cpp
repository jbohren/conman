#include <conman_proto/conman_test_plugins.h>
#include <conman_proto/hook.h>

ORO_LIST_COMPONENT_TYPE(TestEffortController)

TestEffortController::TestEffortController(std::string const& name) :
  RTT::TaskContext(name)
{
  using namespace conman;

  // Create block interface
  boost::shared_ptr<conman::Hook> conman_hook = conman::GetHook(this);
  
  // Create RTT ports
  this->addPort("effort_in", effort_in_).doc("Effort input.");
  this->addPort("effort_out", effort_out_).doc("Effort output := input + 1.");

  // Add the ports to conman
  conman_hook->setInputExclusivity(EXCLUSIVE, &effort_in_);
  conman_hook->setOutputLayer("control", &effort_out_);

  // Register the conman execution hooks
  conman_hook->setComputeControlHook(boost::bind(&TestEffortController::computeControlHook,this,_1));
}

bool TestEffortController::configureHook() {

  return true;
}

bool TestEffortController::startHook() {

  // Ready if the input is connected
  bool ready = effort_in_.connected();

  return ready;
}


void TestEffortController::computeControlHook(
    RTT::os::TimeService::Seconds secs, 
    RTT::os::TimeService::Seconds period) 
{
  double effort;
  effort_in_.read(effort);
  effort_out_.write(effort + 1);
}
