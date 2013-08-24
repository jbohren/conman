
#include <conman_proto/scheme.h>
#include <conman_proto/hook.h>

ORO_LIST_COMPONENT_TYPE(conman::Scheme);

using namespace conman;

Scheme::Scheme(std::string name) 
 : RTT::TaskContext(name),
   graphs_(conman::graph::Layer::N_LAYERS),
   vertex_maps_(conman::graph::Layer::N_LAYERS),
   causal_ordering_(conman::graph::Layer::N_LAYERS)
{
  // Add operations
  this->addOperation("addBlock", 
      (bool (Scheme::*)(const std::string&))&Scheme::add_block, this, 
      RTT::OwnThread)
    .doc("Add a conman block into this scheme.");
  
  this->addOperation("getBlocks", 
      &Scheme::get_blocks, this, 
      RTT::OwnThread)
    .doc("Get the list of all blocks.");

  // Block runtime management
  this->addOperation("enableBlock", 
      (bool (Scheme::*)(const std::string&, bool))&Scheme::enable_block, this, 
      RTT::OwnThread)
    .doc("Enable a block in this scheme.");

  this->addOperation("disableBlock", 
      (bool (Scheme::*)(const std::string&))&Scheme::disable_block, this, 
      RTT::OwnThread)
    .doc("Disable a block in this scheme.");

  this->addOperation("switchBlocks", 
      &Scheme::switch_blocks, this, 
      RTT::OwnThread)
    .doc("Simultaneousy enable and disable a list of blocks, any block not in either list will remain in its current state.");

  this->addOperation("setEnabledBlocks", 
      &Scheme::set_enabled_blocks, this, 
      RTT::OwnThread)
    .doc("Set the list running blocks, any block not on the list will be disabled.");
}


bool Scheme::add_block(RTT::TaskContext *new_block)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::add_block(task)");

  // Nulls are bad
  if(new_block == NULL) {
    RTT::log(RTT::Error) 
      << "Requested block to add is NULL." << RTT::endlog();
    return false;
  }

  // Make sure the block has the conman hook service
  if(!conman::Hook::HasHook(new_block)) {
    RTT::log(RTT::Error) 
      << "Requested block to add does not have the conman hook service." << RTT::endlog();
    return false;
  }

  // Try to add this block as a peer
  if(!this->connectPeers(new_block)) {
    RTT::log() << RTT::Logger::Error << "Could not connect peer: "<<
      new_block->getName() << RTT::endlog();
  }

  // Get the block name
  const std::string block_name = new_block->getName();

  // Connect the block in the appropriate ports in the estimation and control layers
  if(!add_block_to_graph(new_block, Layer::ESTIMATION)) {
    RTT::log() << RTT::Logger::Error << "Could not add TaskContext \""<< block_name <<"\" to scheme estimation layer." << RTT::endlog();
    return false;
  }

  if(!add_block_to_graph(new_block, Layer::CONTROL)) {
    RTT::log() << RTT::Logger::Error << "Could not add TaskContext \""<< block_name <<"\" to scheme control layer." << RTT::endlog();
    return false;
  }

  // Add this block to the list of block names
  block_names_.push_back(block_name);

  // Print out the ordering
  {
  RTT::log(RTT::Info) << "New ordering: [ ";
  for(CausalOrdering::iterator it = causal_ordering_[Layer::CONTROL].begin();
      it != causal_ordering_[Layer::CONTROL].end();
      ++it) 
  {
    RTT::log(RTT::Info) << graphs_[Layer::CONTROL][*it].block->getName() << ", ";
  }
  RTT::log(RTT::Info) << " ] " << RTT::endlog();
  }

  return true;
}

bool Scheme::add_block(const std::string &block_name)
{
  RTT::Logger::In in("Scheme::add_block(string)");

  // Make sure the block exists as a peer of the scheme
  if(!this->hasPeer(block_name)) {
    RTT::TaskContext::PeerList peers = this->getPeerList();

    RTT::log(RTT::Error)
      << "Requested block to add named \"" << block_name << "\" was not a peer"
      "of this Scheme." << std::endl
      << "  Available blocks include:" << std::endl;

    for(RTT::TaskContext::PeerList::iterator it = peers.begin();
        it != peers.end();
        ++it) 
    {
      RTT::log(RTT::Error) << "    " << *it << std::endl;
    }

    RTT::log(RTT::Error) << RTT::endlog();

    return false;
  }

  // Get the newly loaded block
  RTT::TaskContext *new_block = this->getPeer(block_name);

  // Add the block to the graphs
  return this->add_block(new_block);
}

bool Scheme::add_block_to_graph(
    RTT::TaskContext *new_block,
    const conman::graph::Layer::ID &layer)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::add_block_to_graph()");

  // Make sure the block isn't null
  if(new_block == NULL) {
    RTT::log(RTT::Error) << "TaskContext is NULL." << RTT::endlog();
    return false;
  }

  // Make sure the block has the conman hook service
  if(!conman::Hook::HasHook(new_block)) {
    RTT::log(RTT::Error) 
      << "Requested block to add does not have the conman hook service." << RTT::endlog();
    return false;
  }

  // Make sure the layer is valid
  if(layer >= conman::graph::Layer::N_LAYERS) {
    RTT::log(RTT::Error) 
      << "Tried to add block to invalid layer: "<< layer << RTT::endlog();
    return false;
  }

  // Get references to the graph structures
  CausalGraph &graph = graphs_[layer];
  CausalOrdering &ordering = causal_ordering_[layer];
  VertexMap &vertex_map = vertex_maps_[layer];

  // Add this block to the graph
  std::string new_block_name = new_block->getName();

  CausalGraph::vertex_descriptor new_block_descriptor = boost::add_vertex(graph);

  // Store the vertex descriptor in the map
  vertex_map[new_block] = new_block_descriptor;

  // Populate the vertex properties
  VertexProperties &new_vertex = graph[new_block_descriptor];
  new_vertex.index = boost::num_vertices(graph)-1;
  new_vertex.block = new_block;
  new_vertex.hook = conman::Hook::GetHook(new_block);

  RTT::log(RTT::Debug) << "Created vertex: "<< new_vertex.index <<" ("<< new_block_descriptor<<")" << RTT::endlog();

  // Regenerate the topological ordering
  if(!regenerate_graph(layer)) {
    // Report error (if this block's connections add cycles)
    RTT::log(RTT::Error) << "Cannot connect block "
      "\""<<new_block_name<<"\" in conman scheme \""<<Layer::Name(layer)<<"\""
      "layer." << RTT::endlog();

    // Remove the edges connected to this vertex
    boost::clear_vertex(new_block_descriptor, graph);
    RTT::log(RTT::Debug) << "Cleared vertex edges." << RTT::endlog();

    // Remove the vertex 
    boost::remove_vertex(new_block_descriptor, graph);
    RTT::log(RTT::Debug) << "Removed vertex: "<< new_block_descriptor << RTT::endlog();

    // Remove the vertex from the map
    vertex_map.erase(new_block);

    // Regenerate the graph without the vertex
    // This should never fail!
    regenerate_graph(layer);

    return false;
  }

  return true;
}

bool Scheme::regenerate_graph(
    const conman::graph::Layer::ID &layer)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::regenerate_graph()");

  // Make sure the layer is valid
  if(layer >= conman::graph::Layer::N_LAYERS) {
    RTT::log(RTT::Error) 
      << "Tried to add block to invalid layer: "<< Layer::Name(layer) << RTT::endlog();
    return false;
  }

  // Get references to the graph structures
  CausalGraph &graph = graphs_[layer];
  CausalOrdering &ordering = causal_ordering_[layer];
  VertexMap &vertex_map = vertex_maps_[layer];

  // Iterate over all vertices in this graph layer
  for(std::pair<VertexIterator, VertexIterator> vert_it = boost::vertices(graph);
      vert_it.first != vert_it.second;
      ++vert_it.first) 
  {

    RTT::log(RTT::Debug) << "Connecting block with vertex descriptor: "<<*(vert_it.first)<<RTT::endlog();

    // Temporary variable for readability
    conman::graph::VertexProperties &block_vertex = graph[*(vert_it.first)];

    // Get the registered output ports for a given layer
    std::vector<RTT::base::PortInterface*> layer_ports;
    block_vertex.hook->getOutputPortsOnLayer(layer, layer_ports);

    RTT::log(RTT::Debug) << "Block \""<<block_vertex.block->getName()<<"\" has "<<layer_ports.size()<<" ports in the \""<<Layer::Name(layer)<<"\" layer." << RTT::endlog();

    // Create graph arcs for each port between blocks
    for(std::vector<RTT::base::PortInterface*>::iterator port_it = layer_ports.begin();
        port_it != layer_ports.end();
        ++port_it)
    {
      // Get the port, for readability
      RTT::base::PortInterface * port = *port_it;

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
          // Get the source and sink components
          RTT::TaskContext
            *source_block = source_port->getInterface()->getOwner(),
            *sink_block = sink_port->getInterface()->getOwner();

          // Get the source and sink names
          std::string 
            source_name = source_port->getInterface()->getOwner()->getName(),
            sink_name = sink_port->getInterface()->getOwner()->getName();

          // Make sure both blocks are in the graph
          if( vertex_map.find(source_block) != vertex_map.end() && 
              vertex_map.find(sink_block) != vertex_map.end()) 
          {
            // Create a new edge representing this connection
            conman::graph::EdgeProperties edge_props = {true, source_port, sink_port};
            // Add the edge to the graph
            boost::add_edge(vertex_map[source_block], vertex_map[sink_block], edge_props, graph);
          }
        }
      }
    }
  }
  
  // Recompute topological sort (and require that this layer is still a DAG)
  try {
    // Clear the topologically-sorted ordering 
    ordering.clear();
    // Recompute the topological sort
    // NOTE: We need to use an external vertex index property for this
    // algorithm to work since our adjacency_list uses a list as the underlying
    // vertex data structure.
    boost::topological_sort( 
        graph, 
        std::back_inserter(ordering),
        boost::vertex_index_map(boost::get(&VertexProperties::index,graph)));
  } catch(std::exception &ex) {
    // Complain
    RTT::log(RTT::Error)
      << "Cannot regenerate topological ordering in conman scheme "
      "\""<<Layer::Name(layer)<<"\" layer because: " << ex.what() << RTT::endlog();

    return false;
  }

  RTT::log(RTT::Debug) << "Regenerated topological ordering." << RTT::endlog();

  return true;
}

bool Scheme::enable_block(const std::string &block_name, const bool force)
{
  // Get the block by name
  return this->enable_block(this->getPeer(block_name), force);
}

bool Scheme::enable_block(RTT::TaskContext *block, const bool force)
{
  RTT::Logger::In in("Scheme::enable_block");

  if(block == NULL) { 
    return false; 
  }

  const std::string &block_name = block->getName();

  // Make sure the block is configured
  if(!block->isConfigured()) {
    RTT::log(RTT::Error) << "Could not enable block \""<< block_name << "\""
      "because it has not been confiugre()ed." << RTT::endlog();
    return false;
  }

  // Check if conflicting blocks are running
  std::vector<RTT::TaskContext*> &conflicts = block_conflicts_[block];

  for(std::vector<RTT::TaskContext*>::iterator it = conflicts.begin();
      it != conflicts.end();
      ++it)
  {
    // Check if the conflicting block is running
    if((*it)->getTaskState() == RTT::TaskContext::Running) {
      // If force is selected, disable the conflicting block
      if(force) {
        RTT::log(RTT::Info) << "Force-enabling block \""<< block_name << "\""
          "involves disabling block \"" << (*it)->getName() << "\"" <<
          RTT::endlog();

        // Make sure we can actually disable it
        if(this->disable_block(*it) == false) {
          RTT::log(RTT::Error) << "Could not disable block \"" <<
            (*it)->getName() << "\"" << RTT::endlog();
          return false;
        }
      } else {
        RTT::log(RTT::Error) << "Could not enable block \""<< block_name << "\""
          "because it conflicts with block \"" << (*it)->getName() << "\"" <<
          RTT::endlog();
        return false;
      }
    }
  }

  // Try to start the block
  if(!block->start()) {
    RTT::log(RTT::Error) << "Could not enable block \""<< block_name << "\""
      "because it could not be start()ed." << RTT::endlog();
    return false;
  }

  return true;
}

bool Scheme::disable_block(const std::string &block_name)
{
  // Get the block by name
  return this->disable_block(this->getPeer(block_name));
}

bool Scheme::disable_block(RTT::TaskContext* block) 
{
  if(block == NULL) { return false; }

  // Stop a block
  if(block->isRunning()) {
    if(!block->stop()) {
      RTT::log(RTT::Error) 
        << "Could not disable block \""<< block->getName() << "\" because it"
        "could not be stop()ed." << RTT::endlog();
      return false;
    }
  }

  return true;
}

bool Scheme::enable_blocks(const std::vector<std::string> &block_names, const bool strict, const bool force)
{
  bool success = true;

  for(std::vector<std::string>::const_iterator it = block_names.begin();
      it != block_names.end();
      ++it)
  {
    // Try to start the block
    success &= this->enable_block(*it,force);

    // Break on failure if strict
    if(!success && strict) { return false; }
  }

  return success;
}

bool Scheme::disable_blocks(const std::vector<std::string> &block_names, const bool strict)
{
  bool success = true;

  for(std::vector<std::string>::const_iterator it = block_names.begin();
      it != block_names.end();
      ++it)
  {
    // Try to disable the block
    success &= this->disable_block(*it);

    // Break on failure if strict
    if(!success && strict) { return false; }
  }

  return success;
}

bool Scheme::switch_blocks(
    const std::vector<std::string> &disable_block_names,
    const std::vector<std::string> &enable_block_names,
    const bool strict,
    const bool force)
{
  // First disable blocks, so that "force" can be used appropriately when
  // enabling blocks. Also note that we used & instead of && in order to prevent
  // short-circuiting.
  return disable_blocks(disable_block_names, strict) & enable_blocks(enable_block_names, strict, force);
}

bool Scheme::set_enabled_blocks(
    const std::vector<std::string> &enabled_block_names,
    const bool strict)
{
  return this->switch_blocks(this->block_names_, enabled_block_names, strict, false);
}

bool Scheme::configureHook()
{
  return true;
}

bool Scheme::startHook()
{
  return true;
}

void Scheme::updateHook() 
{
  using namespace conman::graph;

  // What time is it
  RTT::os::TimeService::nsecs now = RTT::os::TimeService::Instance()->getNSecs();
  RTT::os::TimeService::Seconds 
    time = (1E-9)*static_cast<double>(now),
    period = (1E-9)*static_cast<double>(RTT::os::TimeService::Instance()->getNSecs(last_update_time_));
  
  // Store update time
  // NOTE: We maintain a single update time for all blocks so that any blocks
  // running at the same rate are executed in the same update() cycle
  last_update_time_ = now;

  // Iterate through estimation graph
  for(CausalOrdering::iterator it = causal_ordering_[Layer::ESTIMATION].begin();
      it != causal_ordering_[Layer::ESTIMATION].end();
      ++it)
  {
    // Temporary variable for readability
    VertexProperties &block_vertex = graphs_[Layer::ESTIMATION][*it];

    // Get the state of the task
    const RTT::base::TaskCore::TaskState block_state = block_vertex.block->getTaskState();
    const RTT::os::TimeService::Seconds block_period = time - block_vertex.last_estimation_time;

    // Check if the task is running and needs to be executed
    if( block_state == RTT::base::TaskCore::Running 
        && block_period >= block_vertex.hook->getPeriod())
    { 
      block_vertex.hook->readHardware(time, block_period);
      block_vertex.hook->computeEstimation(time, block_period);
      block_vertex.last_estimation_time = now;
    }
  }
  
  // Iterate through control graph
  for(CausalOrdering::iterator it = causal_ordering_[Layer::CONTROL].begin();
      it != causal_ordering_[Layer::CONTROL].end();
      ++it)
  {
    // Temporary variable for readability
    VertexProperties &block_vertex = graphs_[Layer::CONTROL][*it];
    
    // Get the state of the task
    const RTT::base::TaskCore::TaskState block_state = block_vertex.block->getTaskState();
    const RTT::os::TimeService::Seconds block_period = time - block_vertex.last_control_time;

    // Check if the task is running and needs to be executed
    if( block_state == RTT::base::TaskCore::Running
        && block_period >= block_vertex.hook->getPeriod())
    {
      block_vertex.hook->computeControl(time, block_period);
      block_vertex.hook->writeHardware(time, block_period);
      block_vertex.last_control_time = now;
    }
  }
}
