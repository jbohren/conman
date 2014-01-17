/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_CONMAN_H
#define __CONMAN_CONMAN_H

#include <string>
#include <vector>
#include <set>
#include <iterator>

#include <rtt/os/main.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <boost/graph/directed_graph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/labeled_graph.hpp>

//! Conman Controller Manager
namespace conman {

  //! Forward declarations
  class Hook;

  namespace graph {

    //! Boost Graph Vertex Metadata for Data Flow Graph
    struct DataFlowVertex 
    {
      typedef boost::shared_ptr<DataFlowVertex> Ptr;

      //! An index for use in topological sort
      unsigned int index;
      //! If true, all inputs are latched
      bool latched_input;
      //! If true, all outputs are latched
      bool latched_output;
      //! The control and/or estimation block 
      RTT::TaskContext *block;
      //! The conman Hook service for this block (cached pointer)
      boost::shared_ptr<conman::Hook> hook;
    };

    // Get the string name for the path to a service
    static const std::string service_path(const RTT::Service *service)
    {
      RTT::Service
        *parent_service = NULL, 
        *parent_parent_service = NULL; 

      if(service) {
        parent_service = service->getParent().get();
      } else {
        // service doesn't exist
        return "";
      }

      if(parent_service) {
        parent_parent_service = parent_service->getParent().get();
      } else {
        // service is the root service
        return "";
      }

      if(parent_parent_service) {
        return service_path(parent_service) + service->getName() + ".";
      }

      return service->getName();
    }

    //! Boost Graph Edge Metadata for Data Flow Graph
    struct DataFlowEdge 
    {
      typedef boost::shared_ptr<DataFlowEdge> Ptr;

      //! If true, execution scheduling does not consider this edge as a constraint
      bool latched;

      //! Model representing a single RTT data port connection
      struct Connection {
        Connection(
            RTT::Service *source_service_,
            RTT::base::PortInterface *source_port_,
            RTT::Service *sink_service_,
            RTT::base::PortInterface *sink_port_) :
          source_service(source_service_),
          sink_service(sink_service_),
          source_port(source_port_),
          sink_port(sink_port_) { }

        //! The service the source port lives on
        RTT::Service *source_service;
        //! The service the sink port lives on
        RTT::Service *sink_service;
        //! The source (output) port
        RTT::base::PortInterface *source_port;
        //! The sink (input) port:
        RTT::base::PortInterface *sink_port;
      };

      //! All the connections on this edge
      std::vector<Connection> connections;
    };

    /** \brief Boost graph for representing the Data Flow Graph between components
     *
     * The block flow graph is used to topologically sort control and estimation
     * networks so that they can be executed causally. This graph contains
     * vertices which correspond to blocks, and edges which correspond to sets
     * of port connections between blocks. It models the abstract data flow
     * graph of the RTT components which have been added to a \ref
     * conman::Scheme.
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
    /*
     *typedef 
     *  boost::adjacency_list< 
     *    boost::vecS, 
     *    boost::vecS, 
     *    boost::bidirectionalS, 
     *    DataFlowVertex::Ptr, 
     *    DataFlowEdge::Ptr>
     *      DataFlowGraph;
     */
    typedef
      boost::directed_graph
      <DataFlowVertex::Ptr, DataFlowEdge::Ptr>
      DataFlowGraph;

    //! Boost Vertex Descriptor Type for DataFlowGraph
    typedef boost::graph_traits<DataFlowGraph>::vertex_descriptor DataFlowVertexDescriptor;
    //! Boost Edge Descriptor Type for DataFlowGraph
    typedef boost::graph_traits<DataFlowGraph>::edge_descriptor DataFlowEdgeDescriptor;
    //! Iterator for iterating over vertices in the DataFlowGraph in no particular order
    typedef boost::graph_traits<conman::graph::DataFlowGraph>::vertex_iterator DataFlowVertexIterator;
    //! Iterator for iterating over edges in the DataFlowGraph in no particular order
    typedef boost::graph_traits<conman::graph::DataFlowGraph>::out_edge_iterator DataFlowOutEdgeIterator;
    //! Iterator for iterating over edges in the DataFlowGraph in no particular order
    typedef boost::graph_traits<conman::graph::DataFlowGraph>::in_edge_iterator DataFlowInEdgeIterator;

    //! Topological Ordering container for DataFlowGraph vertices
    typedef std::list<DataFlowVertexDescriptor> DataFlowPath;
    typedef DataFlowPath ExecutionOrdering;

    //! Vertex descriptor map for retrieving DataFlowGraph vertices
    typedef std::map<RTT::TaskContext*, DataFlowVertexDescriptor> DataFlowVertexTaskMap;

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
     *    boost::vertex_index_map(boost::get(&DataFlowVertex::index,flow_graph)));
     * </pre>
     *
     * In this case, we need to do something more complicated to dereference
     * the shared_ptr:
     * <pre>
     *    boost::vertex_index_map(
     *      boost::make_function_property_map<DataFlowVertexDescriptor>(
     *        boost::bind(&DataFlowVertexIndex,_1,flow_graph))));
     * </pre>
     * 
     * Note above, we use boost::Bind so that the DataFlowVertexIndex function
     * gets the vertex descriptor from the correct graph.
     */
    static unsigned int DataFlowVertexIndex(
        DataFlowVertexDescriptor vertex, 
        DataFlowGraph graph) 
    {
      return graph[vertex]->index;
    }

    //! Boost Graph Cycle Visitor used to capture cycles in data flow graphs
    struct FlowCycleVisitor
    {
      FlowCycleVisitor(std::vector<DataFlowPath> &cycles_)
        : cycles(cycles_)
      {
        cycles.clear();
      }

      //! This is called whenever a cycle is detected
      template <typename Path, typename Graph>
        inline void cycle(const Path& p, const Graph& g)
        {
          DataFlowPath p_vec(p.begin(),p.end());
          cycles.push_back(p_vec);
        }

      std::vector<DataFlowPath> &cycles;
    };

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
        DataFlowVertex::Ptr>
          ConflictGraph;

    //! Vertex descriptor for the block conflict graph
    typedef boost::graph_traits<ConflictGraph>::vertex_descriptor ConflictVertexDescriptor;
    //! Vertex descriptor map for retrieving DataFlowGraph vertices
    typedef std::map<RTT::TaskContext*,ConflictVertexDescriptor> ConflictVertexMap;
    //! Iterator for iterating over vertices in the ConflictGraph in no particular order
    typedef boost::graph_traits<conman::graph::ConflictGraph>::vertex_iterator ConflictVertexIterator;
    //! Iterator for iterating over adjacent vertices in the ConflictGraph in no particular order
    typedef boost::graph_traits<conman::graph::ConflictGraph>::adjacency_iterator ConflictAdjacencyIterator;
    
  }

  //! Exclusivity modes describe how a given port can be accessed.
  struct Exclusivity {
    typedef unsigned int Mode;
    //! Any number of connections.
    static const Mode UNRESTRICTED = 0;
    //! Limit to one connection.
    static const Mode EXCLUSIVE = 1;
  };

  //! Structure for representing groups of comopnents
  typedef std::map<std::string, std::set<std::string> > GroupMap;
}

#endif // ifndef __CONMAN_CONMAN_H
