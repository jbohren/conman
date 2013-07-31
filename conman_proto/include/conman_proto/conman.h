
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


/**
 * Conman Controller Manager
 *
 * Dataflow
 *  Conman dataflow interfaces are normal Orocos RTT ports, except the ports are
 *  connected with a publish/subscribe paradigm. Similar to ROS topics, any two
 *  components reading and writing to the same service/port URI will be
 *  connected to each-other.
 *  We need to associate
 *  additional metadata with the conman ports, however, to satisfy the following
 *  requirements:
 *    - Determine the exclusivity of a port (one or many connections)
 *    - Determine if a port is an input or an output (or we could just try to
 *      over-connect)
 *
 *  Conman places no hard requirements on the names of ports, but instead we
 *  standardize on a set of conventions:
 *    * Hard standardization on datatypes (decided at build time)
 *    * Soft standardization on port names (decided at build or runtime)
 *  
 *  Standard convention is to remap port names from block-relative naming to
 *  runtime-relaative naming. For example, there might be several blocks with
 *  <JointArrayAcc> ports performing joint-level state estimation. In this case,
 *  each block might have "joint_state_unfiltered" and "joint_state_filtered"
 *  ports. These blocks should be able to be remapped in a standard way to allow
 *  pipelining of state estimation filters.
 *
 *  .estimation.left_arm.visual_pose
 *  .control.left_arm.effort_command
 *   --> ekf -->
 *  .estimation.left_arm.joint_state_filtered  
 *
 *  TODO: check ports after each call to make sure they aren't being written to
 *  outside of the appropriate compute-control or compute-estimation hooks
 *  
 * Resources
 *  RTT Ports are the only resources in conman. Reading resources is
 *  unrestricted, but writing to a resource can be controlled. Access is
 *  controlled when the BlockManager enables and disables various control
 *  components, and not when it sets up the RTT Port network. This means that
 *  RTT Ports may be connected in such a way that violates the maximum numvber
 *  of connections. TODO: Should we instead connect and disconnect components
 *  at rumtime?
 *
 **/

//! Conman Controller Manager
namespace conman {

  /** \brief Causal block graph description
   * Causal block graph for topologically sorting control and estimation
   * networks. This graph contains vertices which correspond to blocks, and
   * edges which correspond to port connections between blocks.
   *
   * Vertex Type: vecS 
   *  - Has vertex index property needed for topological sort
   *
   * Edge Type: listS
   *  - Low time complexity
   *  - Permits parallel edges to describe multiple links between blocks
   *
   * Directed: true
   * 
   */
  namespace graph {

    //! Boost Graph Edge Metadata
    struct EdgeProperties {
      //! True if the ports are connected
      bool connected;
      //! The source (output) port
      RTT::base::PortInterface *source_port;
      //! The sink (input) port:
      RTT::base::PortInterface *sink_port;
    };

    //! Boost Graph Vertex Metadata
    struct VertexProperties {
      //! The control and/or estimation block (depending on which graph it's in)
      RTT::TaskContext *block;
      RTT::OperationCaller<void(RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period)> 
        read_hardware,
        compute_estimation,
        compute_control,
        write_hardware;
    };

    //! Boost Graph Type
    typedef boost::labeled_graph<
      boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, VertexProperties, EdgeProperties>,
      std::string> CausalGraph;

    //! Boost Vertex Descriptor Type
    typedef boost::graph_traits<CausalGraph>::vertex_descriptor CausalVertex;

    //! Topological Ordering Structure
    typedef std::vector<conman::graph::CausalVertex> CausalOrdering;
  }
  
  //! Interface types for ports used to connect Blocks
  /**TODO: 
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
