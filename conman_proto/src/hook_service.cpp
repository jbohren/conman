
#include <conman_proto/hook_service.h>

using namespace conman;

HookService::HookService(RTT::TaskContext* owner) :
  RTT::Service("conman",owner),
  // Property Initialization
  execution_period_(0.0)
{ 
  // ConMan Properties
  this->addProperty("executionPeriod",execution_period_)
    .doc("The desired execution period for this block, in seconds. By default, "
        "this is 0 and it will run as fast as the scheme period.");

  // ConMan Introspection interface
  this->addOperation("getPeriod",&HookService::getPeriod, this, RTT::ClientThread);

  // ConMan Execution interface
  this->addOperation("readHardware",&HookService::readHardware,this, RTT::ClientThread);
  this->addOperation("computeEstimation",&HookService::computeEstimation,this, RTT::ClientThread);
  this->addOperation("computeControl",&HookService::computeControl,this, RTT::ClientThread);
  this->addOperation("writeHardware",&HookService::writeHardware,this, RTT::ClientThread);
}


RTT::os::TimeService::Seconds HookService::getPeriod() {
  return execution_period_;
}


bool HookService::setOutputLayer(
    const std::string &port_name,
    const std::string &layer_name) 
{
  // Get the port
  RTT::base::PortInterface *port = this->getOwnerPort(port_name);

  // Make sure that the port is an output port
  if(dynamic_cast<RTT::base::OutputPortInterface*>(port)) {
    // Add to the output port map
    output_ports_[port_name].layer = layer_name; 
    // Add to the layer map
    output_ports_by_layer_[layer_name].insert(port);
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
    const ExclusivityMode mode)
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

conman::ExclusivityMode HookService::getInputExclusivity(const std::string &port_name)
{
  // Get the port
  std::map<std::string,InputProperties>::const_iterator props = input_ports_.find(port_name);

  if(props != input_ports_.end()) {
    return props->second.exclusivity; 
  }

  // Return undefined if the port isn't registered
  return UNDEFINED;
}

std::string HookService::getOutputLayer(const std::string &port_name)
{
  // Get the port properties
  std::map<std::string,OutputProperties>::const_iterator props = output_ports_.find(port_name);
  
  if(props != output_ports_.end()) {
    return props->second.layer; 
  }

  // Return empty string if the port isn't registered
  return "";
}

void HookService::getOutputPortsOnLayer(
    const std::string &layer_name,
    std::vector<RTT::base::PortInterface*> &ports)  
{
  // Get the layer
  std::map<std::string, std::set<RTT::base::PortInterface*> >::iterator layer = 
    output_ports_by_layer_.find(layer_name);

  // Copy the port pointers
  if(layer != output_ports_by_layer_.end()) {
    ports.assign(layer->second.begin(), layer->second.end());
  }
}


bool HookService::setReadHardwareHook(const std::string &operation_name) {
  read_hardware_hook_ = this->getOwnerOperation(operation_name);
  return true;
}
bool HookService::setComputeEstimationHook(const std::string &operation_name) {
  compute_estimation_hook_ = this->getOwnerOperation(operation_name);
  return true;
}
bool HookService::setComputeControlHook(const std::string &operation_name) {
  compute_control_hook_ = this->getOwnerOperation(operation_name);
  return true;
}
bool HookService::setWriteHardwareHook(const std::string &operation_name) {
  write_hardware_hook_ = this->getOwnerOperation(operation_name);
  return true;
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
