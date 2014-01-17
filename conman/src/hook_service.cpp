/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include <conman/hook_service.h>

#include <boost/algorithm/string.hpp>

using namespace conman;

ORO_SERVICE_NAMED_PLUGIN(conman::HookService, "conman_hook");

HookService::HookService(RTT::TaskContext* owner) :
  RTT::Service("conman_hook",owner),
  // Property Initialization
  desired_min_exec_period_(0.0),
  exec_duration_smoothing_factor_(0.5)
{ 
  // Constants 
  this->provides("exclusivity")->addConstant("UNRESTRICTED",Exclusivity::UNRESTRICTED);
  this->provides("exclusivity")->addConstant("EXCLUSIVE",Exclusivity::EXCLUSIVE);

  // Conman Properties
  this->addProperty("desired_min_exec_period",desired_min_exec_period_)
    .doc("The desired (minimum) execution period for this block, in seconds. By default, "
        "this is 0 and it will run as fast as the scheme period.");
  this->addProperty("exec_duration_smoothing_factor",exec_duration_smoothing_factor_)
    .doc("The exponential smoothing factor (between 0.0 and 1.0) used for measuring execution duration.");

  // Introspection Properties
  this->addProperty("last_exec_time",last_exec_time_)
    .doc("Last time this hook was executed.");

  this->addProperty("last_exec_period",last_exec_period_)
    .doc("The last period between two consecutive executions.");
  this->addProperty("min_exec_period",min_exec_period_)
    .doc("The minimum observed execution period between two consecutive executions.");
  this->addProperty("max_exec_period",max_exec_period_)
    .doc("The maximum observed execution period between two consecutive executions.");

  this->addProperty("last_exec_duration",last_exec_duration_)
    .doc("The last duration needed to execute the owner's update hook.");
  this->addProperty("min_exec_duration",min_exec_duration_)
    .doc("The minimum observed duration needed to execute the owner's update hook.");
  this->addProperty("max_exec_duration",max_exec_duration_)
    .doc("The maximum observed duration needed to execute the owner's update hook.");
  this->addProperty("smooth_exec_duration",smooth_exec_duration_)
    .doc("The maximum observed duration needed to execute the owner's update hook.");

  // Conman Configuration Interface
  this->addOperation("setDesiredMinPeriod",&HookService::setDesiredMinPeriod,this,RTT::ClientThread);
  this->addOperation("getDesiredMinPeriod",&HookService::getDesiredMinPeriod,this,RTT::ClientThread);
  this->addOperation("setInputExclusivity",&HookService::setInputExclusivity,this,RTT::ClientThread);
  this->addOperation("getInputExclusivity",&HookService::getInputExclusivity,this,RTT::ClientThread);
  this->addOperation("getRegisteredInputPorts",&HookService::getRegisteredInputPorts,this,RTT::ClientThread);

  // Conman Introspection interface
  // Note: These must be client-thread-based because they are called from the master activity
  this->addOperation("getTime",&HookService::getTime,this,RTT::ClientThread);
  this->addOperation("getPeriod",&HookService::getPeriod,this,RTT::ClientThread);

  // Conman Execution Interface
  // Note: These must be client-thread-based because they are called from the master activity
  this->addOperation("init",&HookService::init,this,RTT::ClientThread)
    .doc("Initialize period computation and execution statistics.");
  this->addOperation("update",&HookService::update,this,RTT::ClientThread)
    .doc("Execute the owner's updateHook and compute execution statistics");
}

bool HookService::setDesiredMinPeriod(const RTT::Seconds period) 
{
  // Make sure the period is nonnegative
  if(period < 0.0) {
    return false;
  }

  // Store the period
  desired_min_exec_period_ = period;
  // Reset init flag
  this->init(0.0);
    
  return true;
}

RTT::Seconds HookService::getDesiredMinPeriod() 
{
  return desired_min_exec_period_;
}

bool HookService::setInputExclusivity(
    const std::string &port_name,
    const unsigned int mode)
{
  // Get the port
  RTT::base::PortInterface *port = this->getOwnerPort(port_name);

  // Make sure that the port is an input port
  if(dynamic_cast<RTT::base::InputPortInterface*>(port)) {
    // Add to the input port map
    input_ports_[port_name].exclusivity = Exclusivity::Mode(mode); 
  } else if(port) {
    // Complain
    RTT::log(RTT::Error) << "Tried to set input exclusivity for an output"
      "port. Output ports do not have exclusivity, and do not make good companions." << RTT::endlog();

    return false;
  } else {
    RTT::log(RTT::Error) << "Tried to set input exclusivity for an unknown port." << RTT::endlog();

    return false;
  }

  return true;
}

unsigned int HookService::getInputExclusivity(
    const std::string &port_name)
{
  // Get the port
  std::map<std::string,InputProperties>::const_iterator props = 
    input_ports_.find(port_name);

  if(props != input_ports_.end()) {
    return props->second.exclusivity; 
  }

  // Return undefined if the port isn't registered
  RTT::log(RTT::Debug) << "Exclusivity for input port \"" << port_name << "\" has not been declared explicitly." <<RTT::endlog();
  return (unsigned int)Exclusivity::UNRESTRICTED;
}

std::vector<std::string> HookService::getRegisteredInputPorts() const {
  std::vector<std::string> port_names;
  port_names.reserve(input_ports_.size());

  for(std::map<std::string, InputProperties>::const_iterator it = input_ports_.begin();
      it != input_ports_.end();
      ++it)
  {
    port_names.push_back(it->first);
  }
  
  return port_names;
}

RTT::Seconds HookService::getTime() 
{
  return last_exec_time_;
}

RTT::Seconds HookService::getPeriod() 
{
  return last_exec_period_;
}


bool HookService::init(const RTT::Seconds time) 
{
  init_ = true;
  return true;
}

bool HookService::update(const RTT::Seconds time) 
{
  // Handle initialization explicitly or if time resets (like in simulation)
  if(init_ || time <= last_exec_time_) {
    last_exec_time_ = time - desired_min_exec_period_;

    min_exec_period_ = std::numeric_limits<double>::max();
    max_exec_period_ = 0.0;

    min_exec_duration_ = std::numeric_limits<double>::max();
    max_exec_duration_ = 0.0;

    init_ = false;
  }

  RTT::Seconds time_since_last_exec = time - last_exec_time_;

  // Return true if we haven't met the desired minimum execution period
  // TODO: Subtract half the scheme period here for better timing?
  if(time_since_last_exec < desired_min_exec_period_) {
    return true;
  }
  
  // Compute statistics describing how often update is being called
  last_exec_period_ = time_since_last_exec;
  last_exec_time_ = time;

  min_exec_period_ = std::min(min_exec_period_,last_exec_period_);
  max_exec_period_ = std::max(max_exec_period_,last_exec_period_);

  // Track how long it takes to execute the component's update hook
  RTT::nsecs exec_start = RTT::os::TimeService::Instance()->getNSecs();

  // Execute the component's update hook
  bool success = this->getOwner()->update();

  // Compute statistics describing how long it actually took to update
  last_exec_duration_ = 
    RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs(exec_start));
  
  min_exec_duration_ = std::min(min_exec_duration_,last_exec_duration_);
  max_exec_duration_ = std::max(max_exec_duration_,last_exec_duration_);

  const double &a = exec_duration_smoothing_factor_;
  smooth_exec_duration_ = a*smooth_exec_duration_ + (1.0-a)*last_exec_duration_;

  return success;
}

RTT::base::PortInterface* HookService::getOwnerPort(const std::string &name) {
  std::vector<std::string> tokens;
  boost::split(tokens, name, boost::is_any_of("."));

  RTT::base::PortInterface* port = NULL;
  boost::shared_ptr<RTT::Service> service = this->getOwner()->provides();
  for(std::vector<std::string>::const_iterator it=tokens.begin();
      it!=tokens.end();
      ++it)
  {
    // Try to get the next token as a port
    if(port = service->getPort(*it)) {
      break;
    }
    // Otherwise continue to treat them as services
    if(!(service = service->provides(*it))) {
      break;
    }
  }

  return port;
}

RTT::OperationInterfacePart* HookService::getOwnerOperation(const std::string &name) {
  return this->getOwner()->getOperation(name);
}
