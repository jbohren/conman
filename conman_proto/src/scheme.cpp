
#include <conman_proto/scheme.h>

ORO_LIST_COMPONENT_TYPE(conman::Scheme);

using namespace conman;

Scheme::Scheme(std::string name) 
 : Block(name)
{
  // Add operations
  this->addOperation("add_block", &Scheme::add_block, this, RTT::ClientThread)
    .doc("Add a conman block into this scheme.");
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
      << "No peer block named: "<< block_name << std::endl
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
  add_block_to_graph(new_block, control_graph_, control_serialization_, "control");
  add_block_to_graph(new_block, estimation_graph_, estimation_serialization_, "estimation");

  // Print out the ordering
  RTT::Logger::log() << RTT::Logger::Info << "New ordering: [ ";
  for(conman::graph::CausalOrdering::iterator it = control_serialization_.begin();
      it != control_serialization_.end();
      ++it) 
  {
    RTT::Logger::log() << RTT::Logger::Info << control_graph_.graph()[*it].block->getName() << ", ";
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
    conman::graph::CausalOrdering &ordering,
    const std::string &layer)
{
  // Validate this this taskcontext has a valid conman interface
  if(!Block::HasConmanInterface(new_block)) {
    RTT::Logger::log() << RTT::Logger::Error << "RTT TaskContext is not a valid Conman::Block" << RTT::endlog();
    return false;
  }

  // Add this block to the graph
  std::string new_block_name = new_block->getName();
  graph.add_vertex(new_block_name);
  graph[new_block_name].block = new_block;

  // Get the registered ports for a given layer
  RTT::OperationCaller<void(const std::string &, std::vector<std::string>&)>
    get_conman_ports = new_block->provides()->getService("conman")->getOperation("getConmanPorts");

  std::vector<std::string> conman_port_names;
  get_conman_ports(layer, conman_port_names);

  // Create graph arcs for each port
  for(std::vector<std::string>::const_iterator name_it = conman_port_names.begin();
      name_it != conman_port_names.end();
      ++name_it)
  {
    // Get the port
    RTT::base::PortInterface* port = new_block->getPort(*name_it);

    // Get the port connections (to get endpoints)
    std::list<RTT::internal::ConnectionManager::ChannelDescriptor> channels = port->getManager()->getChannels();
    std::list<RTT::internal::ConnectionManager::ChannelDescriptor>::iterator channel_it;

    // Create graph arcs for each connection
    for(channel_it = channels.begin(); channel_it != channels.end(); ++channel_it) {
      // Get the connection descriptor
      RTT::base::ChannelElementBase::shared_ptr connection = channel_it->get<1>();

      // Pointers to the endpoints of this connection
      RTT::base::PortInterface  
        *source_port = connection->getOutputEndPoint()->getPort(), 
        *sink_port = connection->getInputEndPoint()->getPort();

      // Make sure the ports and components are not null
      if( source_port != NULL && source_port->getInterface() != NULL
          && sink_port != NULL && sink_port->getInterface() != NULL) 
      {
        // Get the source and sink names
        std::string 
          source_name = source_port->getInterface()->getOwner()->getName(),
          sink_name = sink_port->getInterface()->getOwner()->getName();

        // Make sure both blocks are in the graph
        if(graph.vertex(source_name) != graph.null_vertex() && graph.vertex(sink_name) != graph.null_vertex()) {
          // Create a new edge representing this connection
          conman::graph::EdgeProperties edge_props = {true, source_port, sink_port};
          // Add the edge to the graph
          boost::add_edge_by_label(source_name, sink_name, edge_props, graph);
        }
      }
    }
  }
  
  // Recompute topological sort (and require that this layer is still a DAG)
  try {
    // Clear the topologically-sorted ordering and recompute the sort
    ordering.clear();
    boost::topological_sort( graph.graph(), std::back_inserter(ordering));
  } catch(std::exception &ex) {
    // Report error
    RTT::Logger::log() << RTT::Logger::Error
      << "Cannot connect block \""<<new_block_name<<"\" in conman scheme \""<<layer<<"\" layer: " << ex.what() 
      << RTT::endlog();
    // Remove the vertex
    graph.remove_vertex(new_block_name);
    // Clear the topologically-sorted ordering and recompute the sort
    ordering.clear();
    boost::topological_sort( graph.graph(), std::back_inserter(ordering));
    return false;
  }

  return true;
}
