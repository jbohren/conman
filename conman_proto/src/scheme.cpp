
#include <conman_proto/scheme.h>

ORO_LIST_COMPONENT_TYPE(conman::Scheme);

using namespace conman;

Scheme::Scheme(std::string name) 
 : OCL::DeploymentComponent(name)
{
  // Add operations
  this->addOperation("load_block", &Scheme::load_block, this, RTT::ClientThread)
    .doc("Load a Conman block into this scheme.");
  this->addOperation("add_block", &Scheme::add_block, this, RTT::ClientThread)
    .doc("Add an already loaded conman block into this scheme.");
}

bool Scheme::load_block(
    const std::string &block_name,
    const std::string &component_type)
{
  // Load block
  bool component_loaded = this->loadComponent(block_name, component_type);
  
  // Add the block to the graphs
  return this->add_block(block_name);
}

bool Scheme::add_peer(RTT::TaskContext *new_block)
{
  RTT::Logger::In in("Scheme::add_peer");

  if(!this->connectPeers(new_block)) {
    RTT::Logger::log() << RTT::Logger::Error << "Could not connect peer: "<< new_block->getName() << RTT::endlog();
  }

  // Add the block to the graphs
  return this->add_block(new_block->getName());
}

bool Scheme::add_block(const std::string &block_name)
{
  RTT::Logger::In in("Scheme::add_block");

  // Make sure the block exists
  if(!this->hasPeer(block_name)) {
    RTT::TaskContext::PeerList peers = this->getPeerList();

    RTT::Logger::log() << RTT::Logger::Error 
      << "No block named: "<< block_name << std::endl
      << "Available blocks include:" << std::endl;

    for(RTT::TaskContext::PeerList::iterator it = peers.begin();
        it != peers.end();
        ++it) 
    {
      RTT::Logger::log() << RTT::Logger::Error << "  " << *it << std::endl;
    }

    RTT::Logger::log() << RTT::Logger::Error 
      << RTT::endlog();

    return false;
  }

  // Get the newly loaded block
  RTT::TaskContext *new_block = this->getPeer(block_name);

  // Nulls are bad
  if(new_block == NULL) {
    RTT::Logger::log() << RTT::Logger::Error 
      << "Requested block named: "<<block_name
      << " was found, but could not be acquired (getPeer returned NULL)"
      << RTT::endlog();
    return false;
  }

  // Connect the block in the appropriate ports in the control and estimation graphs
  add_block_to_graph(new_block, control_graph_, "control");
  add_block_to_graph(new_block, estimation_graph_, "estimation");

  // Recompute topological sort
  control_serialization_.clear();
  estimation_serialization_.clear();
  boost::topological_sort(control_graph_,
                          std::back_inserter(control_serialization_));
  boost::topological_sort(estimation_graph_, 
                          std::back_inserter(estimation_serialization_));

  // Print out the ordering
  RTT::Logger::log() << RTT::Logger::Info 
    << "New ordering: [ ";
  for(conman::graph::CausalOrdering::iterator it = control_serialization_.begin();
      it != control_serialization_.end();
      ++it) 
  {
    RTT::Logger::log() << RTT::Logger::Info << control_graph_[*it].block->getName() << ", ";
  }
  RTT::Logger::log() << RTT::Logger::Info << " ] " << RTT::endlog();


  return true;
}

bool Scheme::enable_block(const std::string &block_name, const bool force)
{

  return true;
}

void Scheme::updateHook() 
{
  // Read from hardware

  // Compute state estimate
  
  // Compute control commands

  // Write to hardware

}

bool Scheme::add_block_to_graph(
    RTT::TaskContext *new_block,
    conman::graph::CausalGraph &graph,
    const std::string &layer)
{
  // TODO: Validate this this taskcontext has a valid conman interface

  // Add this block to the graph
  boost::graph_traits<conman::graph::CausalGraph>::vertex_descriptor new_vertex = boost::add_vertex(graph);
  graph[new_vertex].block = new_block;

  // Get groups for this block
  RTT::Service::ProviderNames groups = get_groups(new_block,layer);
  std::map<std::string, RTT::Service::ProviderNames> inputs, outputs;

  // Get ports for input and output of each group
  for(RTT::Service::ProviderNames::iterator group_it = groups.begin();
      group_it != groups.end();
      ++group_it) 
  {
    inputs[*group_it] = conman::get_ports(new_block,layer,*group_it,"in");
    outputs[*group_it] = conman::get_ports(new_block,layer,*group_it,"out");
  }

  // Connect this new block to the appropriate network
  typedef boost::graph_traits<conman::graph::CausalGraph>::vertex_iterator vertex_iter;

  // Iterate over all vertices in this graph
  for(std::pair<vertex_iter, vertex_iter> vp = boost::vertices(graph);
      vp.first != vp.second;
      ++vp.first) 
  {
    // Get a shared pointer to the existing block, for convenience
    RTT::TaskContext *existing_block = graph[*vp.first].block;

    // Make sure we're not connecting the block to itself
    if(existing_block == new_block) {
      continue;
    }

    // Connect all ports in this layer
    for(RTT::Service::ProviderNames::iterator group_it = groups.begin();
        group_it != groups.end();
        ++group_it)
    {
      // Check if the existing block uses this control group
      if(conman::has_group(existing_block,layer,*group_it)) 
      {
        // Temporary pointers to port interfaces
        RTT::base::PortInterface *out_port, *in_port;

        // Connect existing outputs to new inputs:
        // existing[control][control_group][out][*] --> new[control][control_group][in][*]
        for(RTT::Service::ProviderNames::iterator port_it = inputs[*group_it].begin();
            port_it != inputs[*group_it].end();
            ++port_it)
        {
          out_port = conman::get_port(existing_block,layer,*group_it,"out",*port_it);
          in_port = conman::get_port(new_block,layer,*group_it,"in",*port_it);

          // Check if the port exists on the new block
          if(out_port) {
            // Connect the port
            //out_port->connectTo(in_port.get());
            // Add the edge to the graph
            conman::graph::EdgeProperties edge_props = {false, out_port, in_port};
            boost::add_edge(*vp.first, new_vertex, edge_props, graph);
          }
        }

        // Connect existing inputs to new outputs
        // existing[control][control_group][in][*] <-- new[control][control_group][out][*]
        for(RTT::Service::ProviderNames::iterator port_it = outputs[*group_it].begin();
            port_it != outputs[*group_it].end();
            ++port_it)
        {
          out_port = conman::get_port(new_block,layer,*group_it,"out",*port_it);
          in_port = conman::get_port(existing_block,layer,*group_it,"in",*port_it);

          // Check if the port exists on the new block
          if(in_port) {
            // Connect the port
            //out_port->connectTo(in_port.get());
            // Add the edge to the graph
            conman::graph::EdgeProperties edge_props = {false, out_port, in_port};
            boost::add_edge(new_vertex, *vp.first, edge_props, graph);
          }
        }
      }
    }
  }

  return true;
}
