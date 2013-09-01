/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include <conman/hook_service.h>

using namespace conman;

ORO_SERVICE_NAMED_PLUGIN(conman::HookService, "conman_hook");

HookService::HookService(RTT::TaskContext* owner) :
  RTT::Service("conman_hook",owner),
  // Property Initialization
  execution_period_(0.0),
  output_ports_by_layer_(conman::Layer::ids.size())
{ 
  // Conman Properties
  this->addProperty("executionPeriod",execution_period_)
    .doc("The desired execution period for this block, in seconds. By default, "
        "this is 0 and it will run as fast as the scheme period.");

  // Enums TODO: expose these to orocos api
/*
 *  this->addAttribute("UNRESTRICTED",static_cast<int>(Exclusivity::UNRESTRICTED));
 *  this->addAttribute("EXCLUSIVE",static_cast<int>(Exclusivity::EXCLUSIVE));
 *
 *  for(std::vector<Layer::ID>::iterator it = Layer::ids.begin();
 *      it != Layer::ids.end();
 *      ++it)
 *  {
 *    this->addAttribute(Layer::names[*it],static_cast<int<(*id));
 *  }
 */

  // Conman Introspection interface
  this->addOperation("getPeriod",&HookService::getPeriod, this, RTT::ClientThread);

  this->addOperation("setInputExclusivity",&HookService::setInputExclusivity,this,RTT::ClientThread);
  this->addOperation("getInputExclusivity",&HookService::getInputExclusivity,this,RTT::ClientThread);

}


RTT::os::TimeService::Seconds HookService::getPeriod() {
  return execution_period_;
}


bool HookService::setInputExclusivity(
    const std::string &port_name,
    const Exclusivity::Mode mode)
{
  // Get the port
  RTT::base::PortInterface *port = this->getOwnerPort(port_name);

  // Make sure that the port is an input port
  if(dynamic_cast<RTT::base::InputPortInterface*>(port)) {
    // Add to the input port map
    input_ports_[port_name].exclusivity = mode; 
  } else {
    // Complain
    RTT::log(RTT::Error) << "Tried to set input exclusivity for an output"
      "port. Output ports do not have exclusivity" << RTT::endlog();

    return false;
  }

  return true;
}

conman::Exclusivity::Mode HookService::getInputExclusivity(
    const std::string &port_name)
{
  // Get the port
  std::map<std::string,InputProperties>::const_iterator props = 
    input_ports_.find(port_name);

  if(props != input_ports_.end()) {
    return props->second.exclusivity; 
  }

  // Return undefined if the port isn't registered
  return Exclusivity::UNRESTRICTED;
}

RTT::base::PortInterface* HookService::getOwnerPort(const std::string &name) {
  return this->getOwner()->getPort(name);
}

RTT::OperationInterfacePart* HookService::getOwnerOperation(const std::string &name) {
  return this->getOwner()->getOperation(name);
}
