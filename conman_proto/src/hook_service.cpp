
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


RTT::base::PortInterface* HookService::setOutputLayer(
    const std::string &layer_name,
    RTT::base::PortInterface *port) 
{
  // Make sure that the port is an inputport
  if(dynamic_cast<RTT::base::OutputPortInterface*>(port)) {
    // Add to the output port map
    output_ports_[port].layer = layer_name; 
    // Add to the layer map
    output_ports_by_layer_[layer_name].insert(port);
  } else {
    // Complain
    RTT::log(RTT::Error) << "Tried to set output layer for an input port."
      "Input ports inherit the layer from the output port to which they are"
      "connected." << RTT::endlog();
  }
  // Return the port for more manipulation similarly to TaskContext::addPort
  return port;
}

RTT::base::PortInterface* HookService::setInputExclusivity(
    const ExclusivityMode mode,
    RTT::base::PortInterface *port)
{
  // Make sure that the port is an inputport
  if(dynamic_cast<RTT::base::InputPortInterface*>(port)) {
    // Add to the input port map
    input_ports_[port].exclusivity = mode; 
  } else {
    // Complain
    RTT::log(RTT::Error) << "Tried to set input exclusivity for an output"
      "port. Output ports do not have exclusivity" << RTT::endlog();
  }
  // Return the port for more manipulation similarly to TaskContext::addPort
  return port;
}

conman::ExclusivityMode HookService::getInputExclusivity(
    RTT::base::PortInterface *port)
{
  std::map<RTT::base::PortInterface*,InputProperties>::const_iterator props = input_ports_.find(port);

  if(props != input_ports_.end()) {
    return props->second.exclusivity; 
  }

  // Return undefined if the port isn't registered
  return UNDEFINED;
}

std::string HookService::getOutputLayer(
    RTT::base::PortInterface *port)
{
  std::map<RTT::base::PortInterface*,OutputProperties>::const_iterator props = output_ports_.find(port);
  
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
  std::map<std::string, std::set<RTT::base::PortInterface*> >::iterator layer = output_ports_by_layer_.find(layer_name);

  if(layer != output_ports_by_layer_.end()) {
    ports.assign(layer->second.begin(), layer->second.end());
  }
}


bool HookService::setReadHardwareHook(ExecutionHook func) {
  read_hardware_hook_ = func;
  return true;
}
bool HookService::setComputeEstimationHook(ExecutionHook func) {
  compute_estimation_hook_ = func;
  return true;
}
bool HookService::setComputeControlHook(ExecutionHook func) {
  compute_control_hook_ = func;
  return true;
}
bool HookService::setWriteHardwareHook(ExecutionHook func) {
  write_hardware_hook_ = func;
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
