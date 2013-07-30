
#include <conman_proto/block.h>

using namespace conman;

bool Block::HasConmanInterface(RTT::TaskContext *block)
{
  return (block != NULL)
    && block->provides()->hasService("conman")
    && block->provides()->getService("conman")->hasOperation("getConmanPorts");
}


Block::Block(std::string const& name) :
  RTT::TaskContext(name, RTT::base::TaskCore::PreOperational)
{ 
  // All conman services are implemnented under the "conman" service
  conman_service_ = this->provides("conman");

  // Conman Properties
  conman_service_->addProperty("executionRate",execution_rate_)
    .doc("The desired execution rate for this block [Hz]");

  // Conman Introspection interface
  conman_service_->addOperation("getConmanPorts",&Block::getConmanPorts,this);

  // Conman Execution interface
  conman_service_->addOperation("readHardware",&Block::readHardware,this);
  conman_service_->addOperation("computeEstimation",&Block::computeEstimation,this);
  conman_service_->addOperation("computeControl",&Block::computeControl,this);
  conman_service_->addOperation("writeHardware",&Block::writeHardware,this);
}

RTT::base::PortInterface& Block::registerConmanPort(
    const std::string &layer,
    const ExclusivityMode exclusivity_mode,
    RTT::base::PortInterface &port)
{
  // Store the exclusivity mode
  this->setExclusivity(port.getName(),exclusivity_mode);
  // Designate this port as a control port
  conman_ports_[layer].insert(port.getName());
  // Add the port & pass-through normal interface
  return port;
}

void Block::setExclusivity(
    const std::string &port_name,
    const ExclusivityMode mode)
{
  exclusivity_[port_name] = mode; 
}

const Block::ExclusivityMode Block::getExclusivity(const std::string &port_name)
{
  // Check if this is a registered port
  if(exclusivity_.find(port_name) != exclusivity_.end()) {
    return exclusivity_[port_name]; 
  }
  // Return undefined if the port isn't registered
  return UNDEFINED;
}

void Block::getConmanPorts(
    const std::string &layer,
    std::vector<std::string> &port_names)  
{
  if(conman_ports_.find(layer) != conman_ports_.end()) {
    port_names.assign(conman_ports_[layer].begin(), conman_ports_[layer].end());
  }
}

