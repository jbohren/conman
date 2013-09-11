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

  // Constants 
  this->provides("exclusivity")->addConstant("UNRESTRICTED",static_cast<int>(Exclusivity::UNRESTRICTED));
  this->provides("exclusivity")->addConstant("EXCLUSIVE",static_cast<int>(Exclusivity::EXCLUSIVE));

  for(std::vector<Layer::ID>::const_iterator it = Layer::ids.begin();
      it != Layer::ids.end();
      ++it)
  {
    this->provides("layer")->addConstant(Layer::names.find(*it)->second,static_cast<int>(*it));
  }

  // Conman Introspection interface
  this->addOperation("getPeriod",&HookService::getPeriod, this, RTT::ClientThread);

  // Conman Execution interface
  this->addOperation("readHardware",&HookService::readHardware,this, RTT::ClientThread);
  this->addOperation("computeEstimation",&HookService::computeEstimation,this, RTT::ClientThread);
  this->addOperation("computeControl",&HookService::computeControl,this, RTT::ClientThread);
  this->addOperation("writeHardware",&HookService::writeHardware,this, RTT::ClientThread);

  this->addOperation("setOutputLayer",&HookService::setOutputLayer,this,RTT::ClientThread);
  this->addOperation("setInputExclusivity",&HookService::setInputExclusivity,this,RTT::ClientThread);
  this->addOperation("getInputExclusivity",&HookService::getInputExclusivity,this,RTT::ClientThread);
  this->addOperation("getOutputLayer",&HookService::getOutputLayer,this,RTT::ClientThread);
  this->addOperation("getOutputPortsOnLayer",&HookService::getOutputPortsOnLayer,this,RTT::ClientThread);
  this->addOperation("setReadHardwareHook",&HookService::setReadHardwareHook,this,RTT::ClientThread);
  this->addOperation("setComputeEstimationHook",&HookService::setComputeEstimationHook,this,RTT::ClientThread);
  this->addOperation("setComputeControlHook",&HookService::setComputeControlHook,this,RTT::ClientThread);
  this->addOperation("setWriteHardwareHook",&HookService::setWriteHardwareHook,this,RTT::ClientThread);

  // Try to connect with default client hooks
  if(owner != NULL) {
    if(this->setReadHardwareHook("readHardwareHook")) {
      RTT::log(RTT::Info) << "Binding to default readHardwareHook for block \"" << owner->getName() << "\"" << RTT::endlog();
    }
    if(this->setReadHardwareHook("computeEstimationHook")) {
      RTT::log(RTT::Info) << "Binding to default computeEstimationHook for block \"" << owner->getName() << "\"" << RTT::endlog();
    }
    if(this->setComputeControlHook("computeControlHook")) {
      RTT::log(RTT::Info) << "Binding to default computeControlHook for block \"" << owner->getName() << "\"" << RTT::endlog();
    }
    if(this->setReadHardwareHook("writeHardwareHook")) { 
      RTT::log(RTT::Info) << "Binding to default writeHardwareHook for block \"" << owner->getName() << "\"" << RTT::endlog();
    }
  }
}


RTT::os::TimeService::Seconds HookService::getPeriod() {
  return execution_period_;
}


bool HookService::setOutputLayer(
    const std::string &port_name,
    const conman::Layer::ID layer) 
{
  // Get the port
  RTT::base::PortInterface *port = this->getOwnerPort(port_name);

  // Make sure that the port is an output port
  if(dynamic_cast<RTT::base::OutputPortInterface*>(port)) {
    // Add to the output port map
    output_ports_[port_name].layer = layer; 
    // Add to the layer map
    output_ports_by_layer_[layer].insert(port);

    RTT::log(RTT::Debug) << "Added port \""<<port_name<<"\" to the"
      "\""<<conman::Layer::Name(layer)<<"\" layer." << RTT::endlog();
  } else {
    // Complain
    RTT::log(RTT::Error) << "Tried to set output layer for an input port."
      "Input ports inherit the layer from the output port to which they are"
      "connected." << RTT::endlog();

    return false;
  }

  return true;
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

conman::Layer::ID HookService::getOutputLayer(const std::string &port_name)
{
  // Get the port properties
  std::map<std::string,OutputProperties>::const_iterator props =
    output_ports_.find(port_name);
  
  if(props != output_ports_.end()) {
    return props->second.layer; 
  }

  // Return invalid layer
  return conman::Layer::INVALID;
}

void HookService::getOutputPortsOnLayer(
    const conman::Layer::ID layer,
    std::vector<RTT::base::PortInterface*> &ports)  
{
  // Copy the port pointers
  if(layer < conman::Layer::ids.size()) {
    ports.assign(
        output_ports_by_layer_[layer].begin(), 
        output_ports_by_layer_[layer].end());
  }
}


bool HookService::setReadHardwareHook(const std::string &operation_name) {
  RTT::OperationInterfacePart *caller = this->getOwnerOperation(operation_name);
  if(caller != NULL) {
    read_hardware_hook_ = caller;
    return true;
  }
  return false;
}
bool HookService::setComputeEstimationHook(const std::string &operation_name) {
  RTT::OperationInterfacePart *caller = this->getOwnerOperation(operation_name);
  if(caller != NULL) {
    compute_estimation_hook_ = caller;
    return true;
  }
  return false;
}
bool HookService::setComputeControlHook(const std::string &operation_name) {
  RTT::OperationInterfacePart *caller = this->getOwnerOperation(operation_name);
  if(caller != NULL) {
    compute_control_hook_ = caller;
    return true;
  }
  return false;
}
bool HookService::setWriteHardwareHook(const std::string &operation_name) {
  RTT::OperationInterfacePart *caller = this->getOwnerOperation(operation_name);
  if(caller != NULL) {
    write_hardware_hook_ = caller;
    return true;
  }
  return false;
}


void HookService::readHardware( RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period) { 
  this->read_hardware_hook_(time, period); 
  // TODO: Check no conman ports were written to
}

void HookService::computeEstimation( RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period) {
  this->compute_estimation_hook_(time, period); 
  // TODO: Check no conman control ports were written to
}

void HookService::computeControl( RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period)  {
  this->compute_control_hook_(time, period); 
  // TODO: Check no conman estimation ports were written to
}

void HookService::writeHardware(RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period)  { 
  this->write_hardware_hook_(time, period); 
  // TODO: Check no conman ports were written to
}

RTT::base::PortInterface* HookService::getOwnerPort(const std::string &name) {
  return this->getOwner()->getPort(name);
}

RTT::OperationInterfacePart* HookService::getOwnerOperation(const std::string &name) {
  return this->getOwner()->getOperation(name);
}
