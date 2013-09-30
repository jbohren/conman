/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/Component.hpp>

#include <conman/conman_test_plugins.h>
#include <conman/hook.h>

TestEffortController::TestEffortController(std::string const& name) :
  RTT::TaskContext(name)
{
  // Create RTT ports
  this->addPort("effort_in", effort_in_).doc("Effort input.");
  this->addPort("effort_out", effort_out_).doc("Effort output := input + 1.");

  // Load block interface
  using namespace conman;
  conman_hook_ = Hook::GetHook(this);

  if(conman_hook_) {
    // Make the effort input port exclusive
    conman_hook_->setInputExclusivity("effort_in", Exclusivity::EXCLUSIVE);
  } else {
    RTT::log(RTT::Fatal) << "Could not load conman hook." << RTT::endlog();
  }
}

bool TestEffortController::configureHook() 
{
  // Nothing to do 
  return true;
}

bool TestEffortController::startHook() 
{
  // Ready if the input is connected
  bool ready = effort_in_.connected();

  return ready;
}


void TestEffortController::updateHook()
{
  // Get the current and the time since the last update
  const RTT::Seconds 
    time = conman_hook_->getTime(), 
    period = conman_hook_->getPeriod();

  // Some stupid computation
  double effort;
  effort_in_.read(effort);
  effort_out_.write(effort + 1);
}

ORO_LIST_COMPONENT_TYPE(TestEffortController)
ORO_CREATE_COMPONENT_LIBRARY()
