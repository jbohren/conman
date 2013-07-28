
#include <conman_proto/block.h>

using namespace conman;

void Block::set_exclusion(
    const std::string &layer,
    const std::string &group_name,
    const std::string &direction,
    const std::string &port_name,
    const ExclusionMode mode)
{
  exclusion_[layer][group_name][direction][port_name] = mode; 
}

Block::ExclusionMode Block::get_exclusion(
    const std::string &layer,
    const std::string &group_name,
    const std::string &direction,
    const std::string &port_name)
{
  if(exclusion_.find(layer) != exclusion_.end()) {
    if(exclusion_[layer].find(group_name) != exclusion_[layer].end()) {
      if(exclusion_[layer][group_name].find(direction) != exclusion_[layer][group_name].end()) {
        if(exclusion_[layer][group_name][direction].find(port_name) != exclusion_[layer][group_name][direction].end()) {
          return exclusion_[layer][group_name][direction][port_name]; 
        }
      }
    }
  }

  return UNDEFINED;
}

Block::Block(std::string const& name) :
  RTT::TaskContext(name, RTT::base::TaskCore::PreOperational)
{ 
  // Create default services
  this->provides("control")->doc("Control interface layer. This service provides all control inputs & outputs for this block.");
  this->provides("estimation")->doc("Estimation interface layer. This service provides all estimation/state estimation inputs & outputs for this block.");
}

RTT::base::PortInterface& Block::add_conman_port(
    const std::string &layer,
    const std::string &group_name,
    const std::string &direction,
    const std::string &port_name,
    const ExclusionMode exclusion_mode,
    RTT::base::PortInterface &port)
{
  // Store the exclusion mode
  this->set_exclusion(layer,group_name,direction,port_name,exclusion_mode);
  // Add the port & pass-through normal interface
  return this->provides(layer)->provides(group_name)->provides(direction)->addPort(port_name, port);
}
