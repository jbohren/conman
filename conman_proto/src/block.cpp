
#include <conman_proto/block.h>

using namespace conman;

static bool has_conman_operation(RTT::TaskContext *task, const std::string &name) 
{
  if( task != NULL &&
      task->provides()->hasService("conman") &&
      task->provides()->getService("conman")->hasOperation(name) )
  {
    return true;
  }

  RTT::Logger::log() << RTT::Logger::Error << "TaskContext does not have the \"conman."<<name<<"\" service." << RTT::endlog();
  return false;
}

bool Block::HasConmanInterface(RTT::TaskContext *task)
{
  bool valid = true;

  if(task == NULL) {
    RTT::Logger::log() << RTT::Logger::Error << "TaskContext is NULL." << RTT::endlog();
    return false;
  } else if(task->provides()->hasService("conman") == false) {
    RTT::Logger::log() << RTT::Logger::Error << "TaskContext does not have the \"conman\" service." << RTT::endlog();
    return false;
  } else {
    valid &= has_conman_operation(task, "getConmanPorts");
    valid &= has_conman_operation(task, "getExclusivity");
    valid &= has_conman_operation(task, "getPeriod");
    valid &= has_conman_operation(task, "readHardware");
    valid &= has_conman_operation(task, "computeEstimation");
    valid &= has_conman_operation(task, "computeControl");
    valid &= has_conman_operation(task, "writeHardware");
  }

  return valid;
}


Block::Block(std::string const& name) :
  RTT::TaskContext(name, RTT::base::TaskCore::PreOperational),
  execution_period_(0.0)
{ 
  // All conman services are implemnented under the "conman" service
  conman_service_ = this->provides("conman");

  // Conman Properties
  conman_service_->addProperty("executionPeriod",execution_period_)
    .doc("The desired execution period for this block, in seconds. By default, this is 0 and it will run as fast as the scheme period.");

  // Conman Introspection interface
  conman_service_->addOperation("getConmanPorts",&Block::getConmanPorts, this, RTT::ClientThread);
  conman_service_->addOperation("getExclusivity",&Block::getExclusivity, this, RTT::ClientThread);
  conman_service_->addOperation("getPeriod",&Block::getPeriod, this, RTT::ClientThread);

  // Conman Execution interface
  conman_service_->addOperation("readHardware",&Block::readHardware,this, RTT::ClientThread);
  conman_service_->addOperation("computeEstimation",&Block::computeEstimation,this, RTT::ClientThread);
  conman_service_->addOperation("computeControl",&Block::computeControl,this, RTT::ClientThread);
  conman_service_->addOperation("writeHardware",&Block::writeHardware,this, RTT::ClientThread);
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

RTT::os::TimeService::Seconds Block::getPeriod() {
  return execution_period_;
}
