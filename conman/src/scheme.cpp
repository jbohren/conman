
#include <boost/bind.hpp>

#include <conman/scheme.h>
#include <conman/hook.h>

// function_property_map isn't available until version 1.51
#include <boost/version.hpp>
#if BOOST_VERSION / 100000 >= 1 && BOOST_VERSION / 100 % 1000 >= 51
#include <boost/property/function_property_map.hpp>
#else
#include "function_property_map.hpp"
#endif


ORO_LIST_COMPONENT_TYPE(conman::Scheme);

using namespace conman;

Scheme::Scheme(std::string name) 
 : RTT::TaskContext(name),
   flow_graphs_(conman::Layer::ids.size()),
   flow_vertex_maps_(conman::Layer::ids.size()),
   causal_ordering_(conman::Layer::ids.size())
{
  // Add operations
  this->addOperation("addBlock", 
      (bool (Scheme::*)(const std::string&))&Scheme::addBlock, this, 
      RTT::OwnThread)
    .doc("Add a conman block into this scheme.");
  
  this->addOperation("removeBlock", 
      (bool (Scheme::*)(const std::string&))&Scheme::removeBlock, this, 
      RTT::OwnThread)
    .doc("Remove a conman block from this scheme.");

  this->addOperation("getBlocks", 
      &Scheme::getBlocks, this, 
      RTT::OwnThread)
    .doc("Get the list of all blocks.");

  // Block runtime management
  this->addOperation("enableBlock", 
      (bool (Scheme::*)(const std::string&, bool))&Scheme::enableBlock, this, 
      RTT::OwnThread)
    .doc("Enable a block in this scheme.");

  this->addOperation("disableBlock", 
      (bool (Scheme::*)(const std::string&))&Scheme::disableBlock, this, 
      RTT::OwnThread)
    .doc("Disable a block in this scheme.");

  this->addOperation("switchBlocks", 
      &Scheme::switchBlocks, this, 
      RTT::OwnThread)
    .doc("Simultaneousy enable and disable a list of blocks, any block not in either list will remain in its current state.");

  this->addOperation("setEnabledBlocks", 
      &Scheme::setEnabledBlocks, this, 
      RTT::OwnThread)
    .doc("Set the list running blocks, any block not on the list will be disabled.");
}


///////////////////////////////////////////////////////////////////////////////

std::vector<std::string> Scheme::getBlocks() 
{
  using namespace conman::graph;

  std::vector<std::string> block_names(blocks_.size());

  std::vector<std::string>::iterator str_it = block_names.begin();
  std::map<std::string,VertexProperties::Ptr>::iterator block_it = 
    blocks_.begin();

  for(; str_it != block_names.end() && block_it != blocks_.end();
      ++str_it, ++block_it)
  {
    *str_it = block_it->first;
  }

  return block_names;
}

bool Scheme::addBlock(const std::string &block_name)
{
  RTT::Logger::In in("Scheme::addBlock(string)");

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
  return this->addBlock(new_block);
}

bool Scheme::addBlock(RTT::TaskContext *new_block)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::addBlock");

  // Nulls are bad
  if(new_block == NULL) {
    RTT::log(RTT::Error) << "Requested block to add is NULL." << RTT::endlog();
    return false;
  }

  // Make sure the block has the conman hook service
  if(!conman::Hook::HasHook(new_block)) {
    RTT::log(RTT::Error) << "Requested block to add does not have the conman"
      " hook service." << RTT::endlog();
    return false;
  }

  // Try to add this block as a peer
  if(!this->connectPeers(new_block)) {
    RTT::log() << RTT::Logger::Error << "Could not connect peer: " <<
      new_block->getName() << RTT::endlog();
  }
  
  // Get the block name
  const std::string block_name = new_block->getName();

  // Create the vertex properties
  VertexProperties::Ptr new_vertex = boost::make_shared<VertexProperties>();
  new_vertex->index = blocks_.size();
  new_vertex->block = new_block;
  new_vertex->hook = conman::Hook::GetHook(new_block);

  // Add this block to the set of blocks
  blocks_[block_name] = new_vertex;
  // Add this block to the block index (used for re-indexing)
  block_indices_.push_back(new_vertex);

  // Add this block to the conflict graph / map
  conflict_vertex_map_[new_block] = boost::add_vertex(new_vertex,conflict_graph_);

  // Connect the block in the appropriate ports in the estimation and control layers
  bool success = true;
  for(conman::Layer::const_iterator layer_it = Layer::ids.begin();
      layer_it != Layer::ids.end();
      ++layer_it) 
  {
    const Layer::ID &layer = *layer_it;

    if(!addBlockToGraph(new_vertex, layer)) {
      success = false;
      break;
    }
  }

  // Recompute conflicts for this block
  this->computeConflicts(new_block);

  // Cleanup
  if(!success) {
    RTT::log() << RTT::Logger::Error << "Could not add TaskContext \""<< block_name <<"\" to the scheme." << RTT::endlog();
    // Remove the block
    if(!this->removeBlock(new_block)) {
      // This is 
      RTT::log(RTT::Fatal) << "Could not clean up block \"" << block_name <<
        "\" when trying to remove it. Something is terribly wrong." <<
        RTT::endlog();
      return false;
    }
  }

  // Print out the ordering
  RTT::log(RTT::Info) << "Scheme ordering: [ ";
  for(conman::Layer::const_iterator layer_it = Layer::ids.begin();
      layer_it != Layer::ids.end();
      ++layer_it) 
  {
    const Layer::ID &layer = *layer_it;

    // Output the layer name
    RTT::log(RTT::Info) << conman::Layer::Name(layer) <<": ";
    // Output the blocks in the layer
    for(BlockOrdering::iterator it = causal_ordering_[layer].begin();
        it != causal_ordering_[layer].end();
        ++it) 
    {
      RTT::log(RTT::Info) << flow_graphs_[layer][*it]->block->getName() << ", ";
    }
  }
  RTT::log(RTT::Info) << " ] " << RTT::endlog();

  return success;
}

bool Scheme::addBlockToGraph(
    conman::graph::VertexProperties::Ptr new_vertex,
    const conman::Layer::ID &layer)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::addBlockToGraph");

  // Make sure the vertex isn't null
  if(new_vertex.get() == NULL) {
    RTT::log(RTT::Error) << "VertexProperties::Ptr is NULL." << RTT::endlog();
    return false;
  }

  // Make sure the layer is valid
  if(layer >= conman::Layer::ids.size()) {
    RTT::log(RTT::Error) << "Tried to add block to invalid layer: "<< layer <<
      RTT::endlog();
    return false;
  }

  // Get a reference to the block pointer
  TaskContext *&new_block = new_vertex->block;

  // Make sure the block isn't null
  if(new_block == NULL) {
    RTT::log(RTT::Error) << "TaskContext is NULL." << RTT::endlog();
    return false;
  }

  // Make sure the block has the conman hook service
  if(!conman::Hook::HasHook(new_block)) {
    RTT::log(RTT::Error) << "Requested block to add does not have the conman"
      "hook service." << RTT::endlog();
    return false;
  }

  // Get references to the graph structures
  BlockGraph &flow_graph = flow_graphs_[layer];
  BlockVertexMap &flow_vertex_map = flow_vertex_maps_[layer];

  // Add this block to the flow graph
  flow_vertex_map[new_block] = boost::add_vertex(new_vertex, flow_graph);

  RTT::log(RTT::Debug) << "Created vertex: "<< new_vertex->index << " (" <<
    flow_vertex_map[new_block]<<")" << RTT::endlog();

  // Regenerate the topological ordering
  if(!regenerateGraph(layer)) {
    // Report error (if this block's connections add cycles)
    RTT::log(RTT::Error) << "Cannot connect block \"" << new_block->getName()
      << "\" in conman scheme \"" << Layer::Name(layer) << "\"" "layer." <<
      RTT::endlog();

    // Clean up this graph (but not the others, yet)
    this->removeBlockFromGraph(new_vertex, layer);

    return false;
  }

  return true;
}

bool Scheme::removeBlock(const std::string &block_name)
{
  RTT::Logger::In in("Scheme::removeBlock(string)");

  // Make sure the block exists as a peer of the scheme
  if(!this->hasPeer(block_name)) {
    RTT::TaskContext::PeerList peers = this->getPeerList();

    RTT::log(RTT::Error)
      << "Requested block to remove named \"" << block_name << "\" was not a peer"
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
  RTT::TaskContext *block = this->getPeer(block_name);

  // Add the block to the graphs
  return this->removeBlock(block);
}

bool Scheme::removeBlock(
    RTT::TaskContext *block)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::removeBlock");

  // Succeed if the block isn't already in the scheme
  if(blocks_.find(block->getName()) == blocks_.end()) {
    return true;
  }

  bool success = true;

  // Remove the block from all of the flow layers
  for(conman::Layer::const_iterator layer_it = Layer::ids.begin();
      layer_it != Layer::ids.end();
      ++layer_it) 
  {
    const Layer::ID &layer = *layer_it;

    // Get references to the graph structures
    BlockGraph &flow_graph = flow_graphs_[layer];
    BlockVertexMap &flow_vertex_map = flow_vertex_maps_[layer];

    // Check if the block is in this layer
    if(flow_vertex_map.find(block) != flow_vertex_map.end()) {
      // Get the vertex properties pointer
      VertexProperties::Ptr vertex = flow_graph[flow_vertex_map[block]];
      // Remove the vertex from the graph
      if(!this->removeBlockFromGraph(vertex,layer)) {
        // Complain
        RTT::log(RTT::Fatal) << "Failed to remove block \"" << block->getName()
          << "\" from scheme " << Layer::Name(layer) << " layer." <<
          RTT::endlog();
        // Set failure
        success = false;
      }
    }
  }

  // Remove block from conflict graph / map
  boost::clear_vertex(conflict_vertex_map_[block], conflict_graph_);
  boost::remove_vertex(conflict_vertex_map_[block], conflict_graph_);
  conflict_vertex_map_.erase(block);

  // Remove the block from the block map
  blocks_.erase(block->getName());

  // Re-index the vertices 
  unsigned int i=0;
  std::list<VertexProperties::Ptr>::iterator it = block_indices_.begin();
  for(; it != block_indices_.end(); )
  {
    // Remove the block when we get to it
    if((*it)->block == block) {
      it = block_indices_.erase(it);
    } else {
      // Update index of the blocks we don't remove
      (*it)->index = i;
      // Increment the iterator
      ++it;
      // Increment the index
      ++i;
    }
  }

  return success;
}

bool Scheme::removeBlockFromGraph(
    conman::graph::VertexProperties::Ptr vertex,
    const conman::Layer::ID &layer)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::removeBlockFromGraph");

  // Get references to the graph structures
  BlockGraph &flow_graph = flow_graphs_[layer];
  BlockVertexMap &flow_vertex_map = flow_vertex_maps_[layer];

  // Succeed if the vertex already doesn't exist
  if(flow_vertex_map.find(vertex->block) == flow_vertex_map.end()) {
    return true;
  }

  // Remove the edges connected to this vertex 
  boost::clear_vertex(flow_vertex_map[vertex->block], flow_graph);
  // Remove the vertex 
  boost::remove_vertex(flow_vertex_map[vertex->block], flow_graph);
  // Remove the vertex from the map
  flow_vertex_map.erase(vertex->block);

  // Regenerate the graph without the vertex
  if(!regenerateGraph(layer)) {
    return false;
  }

  return true;
}

bool Scheme::regenerateGraph(
    const conman::Layer::ID &layer)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::regenerateGraph");

  // Make sure the layer is valid
  if(layer >= conman::Layer::ids.size()) {
    RTT::log(RTT::Error) 
      << "Tried to add block to invalid layer: "<< Layer::Name(layer) << RTT::endlog();
    return false;
  }

  // Get references to the graph structures
  BlockGraph &flow_graph = flow_graphs_[layer];
  BlockOrdering &ordering = causal_ordering_[layer];
  BlockVertexMap &flow_vertex_map = flow_vertex_maps_[layer];

  bool topology_modified = false;

  // Iterate over all vertices in this graph layer
  for(std::pair<BlockVertexIterator, BlockVertexIterator> vert_it = boost::vertices(flow_graph);
      vert_it.first != vert_it.second;
      ++vert_it.first) 
  {

    /*
     *RTT::log(RTT::Debug) << "Connecting block with vertex descriptor: "<<*(vert_it.first)<<RTT::endlog();
     */

    // Temporary variable for readability
    VertexProperties::Ptr block_vertex = flow_graph[*(vert_it.first)];

    // Get the registered output ports for a given layer
    std::vector<RTT::base::PortInterface*> layer_ports;
    block_vertex->hook->getOutputPortsOnLayer(layer, layer_ports);

    /*
     *RTT::log(RTT::Debug) << "Block \""<<block_vertex->block->getName()<<"\" has"
     *  <<layer_ports.size()<<" ports in the \""<<Layer::Name(layer)<<"\""
     *  "layer." << RTT::endlog();
     */

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
          *source_port = connection->getInputEndPoint()->getPort(), 
          *sink_port = connection->getOutputEndPoint()->getPort();

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
          if( flow_vertex_map.find(source_block) != flow_vertex_map.end() && 
              flow_vertex_map.find(sink_block) != flow_vertex_map.end()) 
          {
            // Get the existing edges between these two blocks
            // NOTE: Using out_edges instead of edge_range because edge_range is buggy
            BlockOutEdgeIterator existing_edge_it, existing_edge_end;
            boost::tie(existing_edge_it, existing_edge_end) = 
              boost::out_edges(flow_vertex_map[source_block], flow_graph);

            // Check if this edge already exists (so we don't create duplicate edges)
            bool edge_exists = false;
            for(; existing_edge_it != existing_edge_end; ++existing_edge_it) {
              if( flow_graph[*existing_edge_it]->source_port == source_port &&
                  flow_graph[*existing_edge_it]->sink_port == sink_port) {
                // The edge exists
                edge_exists = true;
                break;
              }
            }

            // Only create edge if it isn't already there
            if(!edge_exists) {
              // Create a new edge representing this connection
              EdgeProperties::Ptr edge_props = boost::make_shared<EdgeProperties>();
              edge_props->connected = true;
              edge_props->source_port = source_port;
              edge_props->sink_port = sink_port;

              // Add the edge to the graph
              boost::add_edge(flow_vertex_map[source_block], flow_vertex_map[sink_block], edge_props, flow_graph);

              // Set the flag to know we've modified edges
              topology_modified = true;

              RTT::log(RTT::Debug) << "Created "<<Layer::Name(layer)<<" edge "
                <<source_name<<"."<<source_port->getName()<<" --> "
                <<sink_name<<"."<<sink_port->getName()<< RTT::endlog();
            } else {
              RTT::log(RTT::Debug) << "Existis "<<Layer::Name(layer)<<" edge "
                <<source_name<<"."<<source_port->getName()<<" --> "
                <<sink_name<<"."<<sink_port->getName()<< RTT::endlog();
            }
          }
        }
      }
    }
  }
  
  if(topology_modified) {
    // Recompute topological sort (and require that this layer is still a DAG)
    try {
      // Clear the topologically-sorted ordering 
      ordering.clear();
      // Recompute the topological sort
      // NOTE: We need to use an external vertex index property for this
      // algorithm to work since our adjacency_list uses a list as the underlying
      // vertex data structure. See the documentation for BlockVertexIndex for
      // more info.
      boost::topological_sort( 
          flow_graph, 
          std::front_inserter(ordering),
          boost::vertex_index_map(
              boost::make_function_property_map<BlockVertexDescriptor>(
                boost::bind(&BlockVertexIndex,_1,flow_graph))));
    } catch(std::exception &ex) {
      // Complain
      RTT::log(RTT::Error)
        << "Cannot regenerate topological ordering in conman scheme "
        "\""<<Layer::Name(layer)<<"\" layer because: " << ex.what() << RTT::endlog();

      return false;
    }

    RTT::log(RTT::Debug) << "Regenerated topological ordering." << RTT::endlog();
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Scheme::createGroup(
    const std::string &group_name,
    const std::vector<std::string> &grouped_blocks) 
{ 
  RTT::Logger::In in("Scheme::createGroup");

  // Check if the group name collides with a real block
  if(blocks_.find(group_name) != blocks_.end())
  {
    RTT::log(RTT::Error) << "Block group named \"" << group_name << "\""
      "cannot be created because a block with the same name already exists."
      << RTT::endlog();
    return false;
  }

  // Make sure all the blocks are in the scheme
  for(std::vector<std::string>::const_iterator it=grouped_blocks.begin();
      it != grouped_blocks.end();
      ++it)
  {
    // Check if the block is in the scheme
    if( blocks_.find(*it) == blocks_.end() &&
        block_groups_.find(*it) == block_groups_.end())
    {
      RTT::log(RTT::Error) << "Block named \"" << *it << "\""
        "is not in the scheme." << RTT::endlog();
      return false;
    }
  }

  // Check if the group already exists
  if(block_groups_.find(group_name) != block_groups_.end()) {
    RTT::log(RTT::Warning) << "Block group named \"" << group_name << "\""
      " already exists. Over-writing." << RTT::endlog();
  }

  // Flatten and store the group 
  block_groups_[group_name] = 
    std::set<std::string>(grouped_blocks.begin(), grouped_blocks.end());

  return true; 
}

bool Scheme::addToGroup(
    const std::string &group_name,
    const std::string &new_block) 
{
  RTT::Logger::In in("Scheme::addToGroup");

  // Check if the group exists
  std::map<std::string, std::set<std::string> >::iterator group =
    block_groups_.find(group_name);
  if(group == block_groups_.end()) {
    return false;
  }

  // Check if the block is in the scheme
  if( blocks_.find(new_block) == blocks_.end()) {
    RTT::log(RTT::Error) << "Block named \"" << new_block << "\" is not in the"
      " scheme." << RTT::endlog();
    return false;
  }

  // Return the group constituents
  group->second.insert(new_block);

  return true; 
}

bool Scheme::removeFromGroup(
    const std::string &group_name,
    const std::string &block) 
{
  // Check if the group exists
  std::map<std::string, std::set<std::string> >::iterator group = 
    block_groups_.find(group_name);

  if(group == block_groups_.end()) {
    return false;
  }

  // Check if the block is in the group
  if( group->second.find(block) == group->second.end()) {
    // It's already gone
    return true;
  }

  // Remove the block from the group
  group->second.erase(block);

  return true; 
}

bool Scheme::disbandGroup( const std::string &group_name) 
{
  // Check if the group exists
  if(block_groups_.find(group_name) != block_groups_.end()) {
    block_groups_.erase(group_name);
  }
  
  return true; 
}

bool Scheme::getGroup(
    const std::string &group_name,
    std::vector<std::string> &grouped_blocks) 
{
  // Check if the group exists
  std::map<std::string, std::set<std::string> >::iterator group = 
    block_groups_.find(group_name);

  if(group == block_groups_.end()) {
    return false;
  }

  // Return the group constituents
  grouped_blocks.assign(group->second.begin(), group->second.end());

  return true; 
}

///////////////////////////////////////////////////////////////////////////////

void Scheme::computeConflicts() 
{
  for(std::map<std::string,graph::VertexProperties::Ptr>::iterator it=blocks_.begin();
      it != blocks_.end();
      ++it)
  {
    this->computeConflicts(it->second->block);
  }
}

void Scheme::computeConflicts(const std::string &block_name) 
{
  this->computeConflicts(this->getPeer(block_name));
}

void Scheme::computeConflicts(const std::vector<std::string> &block_names)
{
  for(std::vector<std::string>::const_iterator it = block_names.begin();
      it != block_names.end();
      ++it)
  {
    this->computeConflicts(*it);
  }
}

void Scheme::computeConflicts(RTT::TaskContext *block)
{
  using namespace conman::graph;

  // Iterator for out edges
  boost::graph_traits<BlockGraph>::out_edge_iterator out_edge_it, out_edge_end;
  boost::graph_traits<BlockGraph>::in_edge_iterator in_edge_it, in_edge_end;
  
  // For each layer
  for(conman::Layer::const_iterator layer_it = Layer::ids.begin();
      layer_it != Layer::ids.end();
      ++layer_it) 
  {
    const Layer::ID &layer = *layer_it;

    // Get references to the graph structures
    BlockGraph &flow_graph = flow_graphs_[layer];
    BlockVertexMap &flow_vertex_map = flow_vertex_maps_[layer];

    // Get all output ports on this layer
    boost::tie(out_edge_it, out_edge_end) = boost::out_edges(flow_vertex_map[block], flow_graph);

    // Handle conflicts resulting from each output port
    for(;out_edge_it != out_edge_end; ++out_edge_it) {
      // Get a reference to the edge properties for convenience
      EdgeProperties::Ptr edge = flow_graph[*out_edge_it];

      // Get a reference to the vertex properties of the sink for convenience
      BlockVertexDescriptor sink_vertex_descriptor = boost::target(*out_edge_it, flow_graph);
      VertexProperties::Ptr sink_vertex = flow_graph[sink_vertex_descriptor];

      // Get the exclusivity of this port
      const conman::Exclusivity::Mode mode = sink_vertex->hook->getInputExclusivity(edge->sink_port->getName());

      // Only exclusive ports can induce conflicts
      if(mode == conman::Exclusivity::EXCLUSIVE) {
        // Get input edges for the sink vertex
        boost::tie(in_edge_it, in_edge_end) = boost::in_edges(sink_vertex_descriptor, flow_graph);

        // Add conflicts with each other block that also has a connection to this input port
        for(;in_edge_it != in_edge_end; ++in_edge_it) {
          // Pointer comparison to check if this edge corresponds to the sink port 
          if(flow_graph[*in_edge_it]->sink_port == edge->sink_port) {
            // Add conflict between the seed block and the source block for this edge
            VertexProperties::Ptr conflicting_vertex = flow_graph[boost::source(*in_edge_it,flow_graph)];

            // Make sure the block is in the conflict map, and isn't itself
            if( conflict_vertex_map_.find(block) != conflict_vertex_map_.end() &&
                conflict_vertex_map_.find(conflicting_vertex->block) != conflict_vertex_map_.end() &&
                block != conflicting_vertex->block) 
            {
              // Add an edge in the conflict graph
              add_edge(
                  conflict_vertex_map_[block],
                  conflict_vertex_map_[conflicting_vertex->block],
                  conflict_graph_);

              // Debug output
              RTT::log(RTT::Debug) << "Added conflict between blocks "<<
                block->getName() << " and " <<
                conflicting_vertex->block->getName() << RTT::endlog();
            }
          }
        }
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////

bool Scheme::enableBlock(const std::string &block_name, const bool force)
{
  // First check if this block is a group
  std::map<std::string, std::set<std::string> >::iterator group = 
    block_groups_.find(block_name);

  if(group != block_groups_.end()) {
    // Enable the blocks in this group
    return this->enableBlocks(
        std::vector<std::string>(group->second.begin(),group->second.end()),
        true,
        force);
  }

  // Enable the block by name
  return this->enableBlock(this->getPeer(block_name), force);
}

bool Scheme::enableBlock(RTT::TaskContext *block, const bool force)
{
  using namespace conman::graph;

  RTT::Logger::In in("Scheme::enableBlock");

  if(block == NULL) { 
    return false; 
  }

  const std::string &block_name = block->getName();

  if(blocks_.find(block_name) == blocks_.end()) {
    RTT::log(RTT::Error) << "Could not enable block \""<< block_name << "\""
      " because it has not been added to the scheme." << RTT::endlog();
    return false;
  }

  // Make sure the block is configured
  if(!block->isConfigured()) {
    RTT::log(RTT::Error) << "Could not enable block \""<< block_name << "\""
      " because it has not been confiugre()ed." << RTT::endlog();
    return false;
  }

  // Check if the block is already enabled
  if(block->getTaskState() == RTT::TaskContext::Running) {
    // If it's already running, then we're going to assume for now that the
    // user isn't doing anything dirty.
    // TODO: Keep track of whether or not a block has been properly enabled.
    return true;
  }

  // Get the blocks that conflict with this block
  BlockConflictAdjacencyIterator conflict_it, conflict_end;

  boost::tie(conflict_it, conflict_end) =
    boost::adjacent_vertices(conflict_vertex_map_[block], conflict_graph_);

  // Check if conflicting blocks are running
  for(; conflict_it != conflict_end; ++conflict_it)
  {
    RTT::TaskContext *&conflict_block = conflict_graph_[*conflict_it]->block;

    // Check if the conflicting block is running
    if(conflict_block->getTaskState() == RTT::TaskContext::Running) {
      // If force is selected, disable the conflicting block
      if(force) {
        RTT::log(RTT::Info) << "Force-enabling block \""<< block_name << "\""
          " involves disabling block \"" << conflict_block->getName() << "\""
          << RTT::endlog();

        // Make sure we can actually disable it
        if(this->disableBlock(conflict_block) == false) {
          RTT::log(RTT::Error) << "Could not disable block \"" <<
            conflict_block->getName() << "\"" << RTT::endlog();
          return false;
        }
      } else {
        RTT::log(RTT::Error) << "Could not enable block \""<< block_name <<
          "\" because it conflicts with block \"" << conflict_block->getName()
          << "\"" << RTT::endlog();
        return false;
      }
    }
  }

  // Try to start the block
  if(!block->start()) {
    RTT::log(RTT::Error) << "Could not enable block \""<< block_name << "\""
      " because it could not be start()ed." << RTT::endlog();
    return false;
  }

  return true;
}

bool Scheme::disableBlock(const std::string &block_name)
{
  // First check if this block is a group
  std::map<std::string, std::set<std::string> >::iterator group = 
    block_groups_.find(block_name);

  if(group != block_groups_.end()) {
    // Enable the blocks in this group
    return this->disableBlocks(
        std::vector<std::string>(group->second.begin(),group->second.end()),
        true);
  }

  // Disable the block by name
  return this->disableBlock(this->getPeer(block_name));
}

bool Scheme::disableBlock(RTT::TaskContext* block) 
{
  if(block == NULL) { return false; }

  // Stop a block
  if(block->isRunning()) {
    if(!block->stop()) {
      RTT::log(RTT::Error) 
        << "Could not disable block \""<< block->getName() << "\" because it"
        " could not be stop()ed." << RTT::endlog();
      return false;
    }
  }

  return true;
}

bool Scheme::enableBlocks(
    const std::vector<std::string> &block_names,
    const bool strict,
    const bool force)
{
  using namespace conman::graph;

  // First make sure all the blocks can be enabled
  if(!force) {
    for(std::vector<std::string>::const_iterator it = block_names.begin();
        it != block_names.end();
        ++it)
    {
      // Get the blocks that conflict with this block
      BlockConflictAdjacencyIterator conflict_it, conflict_end;

      boost::tie(conflict_it, conflict_end) =
        boost::adjacent_vertices(conflict_vertex_map_[blocks_[*it]->block], conflict_graph_);

      // Check if conflicting blocks are running
      for(; conflict_it != conflict_end; ++conflict_it)
      {
        RTT::TaskContext *&conflict_block = conflict_graph_[*conflict_it]->block;

        // Check if the conflicting block is running
        if(conflict_block->getTaskState() == RTT::TaskContext::Running) {
          return false;
        }
      }
    }
  }

  // Enable the blocks
  bool success = true;

  for(std::vector<std::string>::const_iterator it = block_names.begin();
      it != block_names.end();
      ++it)
  {
    // Try to start the block
    success &= this->enableBlock(*it,force);

    // Break on failure if strict
    if(!success && strict) { return false; }
  }

  return success;
}

bool Scheme::disableBlocks(const bool strict)
{
  bool success = true;

  for(std::map<std::string,graph::VertexProperties::Ptr>::const_iterator it = blocks_.begin();
      it != blocks_.end();
      ++it)
  {
    // Try to disable the block
    success &= this->disableBlock(it->second->block);

    // Break on failure if strict
    if(!success && strict) { return false; }
  }

  return success;
}

bool Scheme::disableBlocks(
    const std::vector<std::string> &block_names,
    const bool strict)
{
  bool success = true;

  for(std::vector<std::string>::const_iterator it = block_names.begin();
      it != block_names.end();
      ++it)
  {
    // Try to disable the block
    success &= this->disableBlock(*it);

    // Break on failure if strict
    if(!success && strict) { return false; }
  }

  return success;
}

bool Scheme::switchBlocks(
    const std::vector<std::string> &disable_block_names,
    const std::vector<std::string> &enable_block_names,
    const bool strict,
    const bool force)
{
  // First disable blocks, so that "force" can be used appropriately when
  // enabling blocks. Also note that we used & instead of && in order to prevent
  // short-circuiting.
  return disableBlocks(disable_block_names, strict) & 
    enableBlocks(enable_block_names, strict, force);
}

bool Scheme::setEnabledBlocks(
    const std::vector<std::string> &enabled_block_names,
    const bool strict)
{
  return this->disableBlocks(strict) & 
    this->enableBlocks(enabled_block_names, strict, false);
}

///////////////////////////////////////////////////////////////////////////////

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
  for(BlockOrdering::iterator it = causal_ordering_[Layer::ESTIMATION].begin();
      it != causal_ordering_[Layer::ESTIMATION].end();
      ++it)
  {
    // Temporary variable for readability
    VertexProperties::Ptr block_vertex = flow_graphs_[Layer::ESTIMATION][*it];

    // Get the state of the task
    const RTT::base::TaskCore::TaskState block_state = block_vertex->block->getTaskState();
    const RTT::os::TimeService::Seconds block_period = time - block_vertex->last_estimation_time;

    // Check if the task is running and needs to be executed
    if( block_state == RTT::base::TaskCore::Running 
        && block_period >= block_vertex->hook->getPeriod())
    { 
      block_vertex->hook->readHardware(time, block_period);
      block_vertex->hook->computeEstimation(time, block_period);
      block_vertex->last_estimation_time = now;
    }
  }
  
  // Iterate through control graph
  for(BlockOrdering::iterator it = causal_ordering_[Layer::CONTROL].begin();
      it != causal_ordering_[Layer::CONTROL].end();
      ++it)
  {
    // Temporary variable for readability
    VertexProperties::Ptr block_vertex = flow_graphs_[Layer::CONTROL][*it];
    
    // Get the state of the task
    const RTT::base::TaskCore::TaskState block_state = block_vertex->block->getTaskState();
    const RTT::os::TimeService::Seconds block_period = time - block_vertex->last_control_time;

    // Check if the task is running and needs to be executed
    if( block_state == RTT::base::TaskCore::Running
        && block_period >= block_vertex->hook->getPeriod())
    {
      block_vertex->hook->computeControl(time, block_period);
      block_vertex->hook->writeHardware(time, block_period);
      block_vertex->last_control_time = now;
    }
  }
}
