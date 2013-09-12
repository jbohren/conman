/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_CONMAN_H
#define __CONMAN_CONMAN_H

#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/main.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/labeled_graph.hpp>

//! Conman Controller Manager
namespace conman {

  //! Forward declarations
  class Hook;

  namespace graph {

    //! Boost Graph Vertex Metadata
    struct VertexProperties {
      typedef boost::shared_ptr<VertexProperties> Ptr;

      //! An index for use in topological sort
      unsigned int index;
      //! The control and/or estimation block 
      RTT::TaskContext *block;
      //! The conman Hook service for this block (cached pointer)
      boost::shared_ptr<conman::Hook> hook;
    };

    //! Boost Graph Edge Metadata
    struct EdgeProperties {
      typedef boost::shared_ptr<EdgeProperties> Ptr;

      // TODO: Make these Input/OuputPortInterfaces instead of just PortInterfaces
      //! The source (output) port
      RTT::base::PortInterface *source_port;
      //! The sink (input) port:
      RTT::base::PortInterface *sink_port;
    };

    /** \brief Boost graph for representing the data flow graph between components
     *
     * The block flow graph is used to topologically sort control and estimation
     * networks so that they can be executed causally. This graph contains
     * vertices which correspond to blocks, and edges which correspond to port
     * connections between blocks. It models the abstract data flow graph of the
     * RTT components which have been added to a \ref conman::Scheme.
     *
     * Vertex Type: listS
     *  - Allows for low-time-complexity adding/removing edges
     *
     * Edge Type: listS
     *  - Low time complexity
     *  - Permits parallel edges to describe multiple links between blocks
     *
     * Directed Type: bidirectionalS
     *  - Edges have direction
     *  - Allows querying of both in- and out- edges (directedS does not provide
     *  this capability.
     */
    typedef 
      boost::adjacency_list< 
        boost::listS, 
        boost::listS, 
        boost::bidirectionalS, 
        VertexProperties::Ptr, 
        EdgeProperties::Ptr>
          BlockGraph;

    //! Boost Vertex Descriptor Type for BlockGraph
    typedef boost::graph_traits<BlockGraph>::vertex_descriptor BlockVertexDescriptor;
    //! Boost Edge Descriptor Type for BlockGraph
    typedef boost::graph_traits<BlockGraph>::edge_descriptor BlockEdgeDescriptor;
    //! Topological Ordering container for BlockGraph vertices
    typedef std::list<BlockVertexDescriptor> BlockOrdering;
    //! Iterator for iterating over vertices in the BlockGraph in no particular order
    typedef boost::graph_traits<conman::graph::BlockGraph>::vertex_iterator BlockVertexIterator;
    //! Iterator for iterating over edges in the BlockGraph in no particular order
    typedef boost::graph_traits<conman::graph::BlockGraph>::out_edge_iterator BlockOutEdgeIterator;
    //! Vertex descriptor map for retrieving BlockGraph vertices
    typedef std::map<RTT::TaskContext*,BlockVertexDescriptor> BlockVertexMap;

    /** \brief Function for extracting the vertex index from a block vertex
     *
     * This function is used in a boost::function_property_map by topological
     * sort because the topological sort algorithm needs an vertex index
     * property, and we're storing boost::shared_ptrs as vertex properties.
     *
     * Normally, one could use boost::get() to get the index property (as shown
     * below), but in this case, the use of shared_ptr properties precludes
     * this:
     * <pre>
     *    boost::vertex_index_map(boost::get(&VertexProperties::index,flow_graph)));
     * </pre>
     *
     * In this case, we need to do something more complicated to dereference
     * the shared_ptr:
     * <pre>
     *    boost::vertex_index_map(
     *      boost::make_function_property_map<BlockVertexDescriptor>(
     *        boost::bind(&BlockVertexIndex,_1,flow_graph))));
     * </pre>
     * 
     * Note above, we use boost::Bind so that the BlockVertexIndex function
     * gets the vertex descriptor from the correct graph.
     */
    static unsigned int BlockVertexIndex(BlockVertexDescriptor vertex, BlockGraph graph) {
      return graph[vertex]->index;
    }


    /** \brief Boost graph for representing the conflicts between components
     *
     * Vertices in this graph correspond to blocks, and edges correspond to
     * conflict relationships between those blocks. I.e. if two blocks are
     * adjacent in this graph, then they cannot be run at the same time without
     * causing a resource exclusivity violation.
     * 
     * Vertex Type: listS
     *  - Allows for low-time-complexity adding/removing edges
     *
     * Edge Type: listS
     *  - Low time complexity
     *  - Permits parallel edges to describe multiple links between blocks
     *
     * Directed Type: undirectedS
     *  - The "conflict" relationship is symmetric
     */
    typedef 
      boost::adjacency_list< 
        boost::listS, 
        boost::listS, 
        boost::undirectedS, 
        VertexProperties::Ptr>
          BlockConflictGraph;

    //! Vertex descriptor for the block conflict graph
    typedef boost::graph_traits<BlockConflictGraph>::vertex_descriptor BlockConflictVertexDescriptor;
    //! Vertex descriptor map for retrieving BlockGraph vertices
    typedef std::map<RTT::TaskContext*,BlockConflictVertexDescriptor> BlockConflictVertexMap;
    //! Iterator for iterating over vertices in the ConflictGraph in no particular order
    typedef boost::graph_traits<conman::graph::BlockConflictGraph>::vertex_iterator BlockConflictVertexIterator;
    //! Iterator for iterating over adjacent vertices in the ConflictGraph in no particular order
    typedef boost::graph_traits<conman::graph::BlockConflictGraph>::adjacency_iterator BlockConflictAdjacencyIterator;
  }
  
  //! Scheme role identification
  struct Role {
    enum ID {
      ESTIMATION,
      CONTROL,
      UNDEFINED
    };

    typedef std::vector<ID>::const_iterator const_iterator;

    //! The vector of role IDs and the order in which they should be computed
    static const std::vector<ID> ids;
    static const std::map<ID,std::string> names;

    //! Make sure the role ID is real
    static bool Valid(const ID id) {
      return static_cast<int>(id) < ids.size();
    }
    
    //! Get the name from a role ID
    static const std::string& Name(const ID id) {
      return names.find(id)->second;
    }
  };


  //! Exclusivity modes describe how a given port can be accessed.
  struct Exclusivity {
    enum Mode {
      //! Any number of connections.
      UNRESTRICTED,
      //! Limit to one connection.
      EXCLUSIVE
    };
  };

  
  //! Interface types for ports used to connect Blocks
  /***TODO: 
  namespace interfaces {

    template<class InterfaceT>
    struct Input {
      Input(RTT::TaskContext *parent) : parent_(parent) {}  

      //! Add a conman port in the layer and group
      RTT::base::PortInterface& addPort(std::string layer, std::string group) {
        return (parent_->
                provides(layer)->
                provides(group)->
                addPort(InterfaceT::name, this->port));
      }

      RTT::InputPort<InterfaceT::datatype> port;

      private:
      RTT::TaskContext *parent_;
    };

    //! Effort interface to a single joint
    struct JointState { typedef double datatype; };
    struct JointPosition { typedef double datatype; };
    struct JointVelocity { typedef double datatype; };
    struct JointAcceleration { typedef double datatype; };

    struct JointEffort { typedef double datatype; };
    
  }**/
}

#endif // ifndef __CONMAN_CONMAN_H
