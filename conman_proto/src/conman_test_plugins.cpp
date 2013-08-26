
#include <rtt/Component.hpp>

#include <conman_proto/conman_test_plugins.h>
#include <conman_proto/hook.h>

TestEffortController::TestEffortController(std::string const& name) :
  RTT::TaskContext(name)
{
  using namespace conman;
  // Create RTT ports
  this->addPort("effort_in", effort_in_).doc("Effort input.");
  this->addPort("effort_out", effort_out_).doc("Effort output := input + 1.");

  // Register the conman execution hooks (use default name)
  this->addOperation("computeControlHook",&TestEffortController::computeControlHook, this);

  // Load block interface
  boost::shared_ptr<conman::Hook> conman_hook = conman::Hook::GetHook(this);

  if(conman_hook.get()) {

    // Add the ports to conman
    conman_hook->setInputExclusivity("effort_in", Exclusivity::EXCLUSIVE);
    conman_hook->setOutputLayer("effort_out", Layer::CONTROL);

  } else {
    RTT::log(RTT::Fatal) << "Could not load conman hook." << RTT::endlog();
  }
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

ORO_LIST_COMPONENT_TYPE(TestEffortController)
ORO_CREATE_COMPONENT_LIBRARY()
