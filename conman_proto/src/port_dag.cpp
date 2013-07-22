
#include <rtt/os/main.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>


/**
 *
 * Resources
 *  RTT Ports are the only resources in conman. Reading resources is
 *  unrestricted, but writing to a resource can be controlled. Access is
 *  controlled when the ControllerManager enables and disables various control
 *  components, and not when it sets up the RTT Port network. This means that
 *  RTT Ports may be connected in such a way that violates the maximum numvber
 *  of connections. TODO: Should we instead connect and disconnect components
 *  at rumtime?
 *
 **/

namespace conman {

  const std::string CONTROL_SERVICE = "control";
  const std::string FEEDBACK_SERVICE = "feedback";

  /** \name Causal Graph \{ **/

  /**
   * Causal block graph for control and feedback topological sort. This graph
   * contains vertices which correspond to blocks, and edges which correspond to
   * port connections between blocks.
   *
   * Vertex Type: listS
   *  - low time complexity
   * Edge Type: listS
   *  - low time complexity
   *  - permits parallel edges to describe multiple links between blocks
   * Directed: true
   *
   */

  struct EdgeProperties {
    //! True if the ports are connected
    bool connected;
    //! The output port
    boost::shared_ptr<RTT::base::PortInterface> out;
    //! The input port:
    boost::shared_ptr<RTT::base::PortInterface> in;
  };

  struct VertexProperties {
    //! The control or feedback block (depending on which graph it's in)
    boost::shared_ptr<RTT::TaskContext> block;
  };

  typedef boost::adjacency_list<boost::listS, boost::listS, boost::directedS,
          VertexProperties, EdgeProperties> CausalGraph;

  /**\}**/


  /** \name Convenience functions \{ **/

  /** \brief Check of a block has a group **/
  bool has_group(boot::shared_ptr<RTT::TaskContext> block,
                 const std::string &layer, 
                 const std::string &group) 
  {
    return (block->provides()->hasService(layer)
            && block->provides(layer)->hasService(group));
  }

  /** \brief Check if a block has a specific port **/
  bool has_port(boot::shared_ptr<RTT::TaskContext> block,
                const std::string &layer, 
                const std::string &group,
                const std::string &direction,
                const std::string &port) 
  {
    return (has_group(block,layer,group) 
            && (block->
                provides(layer)->
                provides(group)->
                provides(direction)->
                getPort(port)));
  }

  /** \brief Get a list of service names corresponding to the groups in a given
   * layer **/
  RTT::Service::ProviderNames get_groups(
      boot::shared_ptr<RTT::TaskContext> block,
      const std::string &layer)
  {
    return block->provides(layer)->getProviderNames();
  }

  /** \brief Get a list of port names for s specific conduit **/
  RTT::Service::ProviderNames get_ports(
      boot::shared_ptr<RTT::TaskContext> block,
      const std::string &layer,
      const std::string &group,
      const std::string &direction)
  {
    return (block->
            provides(layer)-> 
            provides(group)->
            provides(direction)-> 
            getPortNames());
  }

  /** \brief Get the service for a specific group **/
  RTT::Service::shared_ptr get_group(
      boot::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group) 
  {
    return block->provides(layer)->provides(group);
  }

  /** \brief Get the input service for a group **/
  RTT::Service::shared_ptr get_input_service(
      boot::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group) 
  {
    return block->provides(layer)->provides(group)->provides("in");
  }

  /** \brief Get the output service for a group **/
  RTT::Service::shared_ptr get_output_service(
      boot::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group) 
  {
    return block->provides(layer)->provides(group)->provides("out");
  }

  /** \brief Get a specific port **/
  boost::shared_ptr<RTT::base::PortInterface> get_port(
      boot::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group,
      const std::string &direction,
      const std::string &port) 
  {
    return boost::shared_ptr<RTT::base::PortInterface>(
        block->
        provides(layer)->
        provides(group)->
        provides(direction)->
        getPort(port));
  }

  /**\}**/

  // Connect a block to the appropriate blocks in a given graph
  // This connects all inputs/outputs of block_a to all outputs/inputs of
  // block_b, given the groups, inputs, and outputs of block_b
  bool add_block(
      boost::shared_ptr<RTT::TaskContext> block,
      conman::CausalGraph &graph,
      const std::string &layer)
  {
    // TODO: Validate this this taskcontext has a valid conman interface

    // Add this block to the graph
    boost::vertex_descriptor new_vertex = boost::add_vertex(graph);
    graph[new_vertex].block = new_block;
    
    // Get groups for this block
    RTT::Service::ProviderNames groups = get_groups(block,layer);

    // Get ports for input and output of each group
    for(RTT::Service::ProviderNames::iterator group_it = groups.begin();
        group_it != groups.end();
        ++group_it) 
    {
      inputs[*group_it] = conman::get_ports(new_block,layer,*group_it,"in");
      outputs[*group_it] = conman::get_ports(new_block,layer,*group_it,"out");
    }

    // Connect this new block to the appropriate network
    typedef boost::graph_traits<CausalGraph>::vertex_iterator vertex_iter;

    // Iterate over all vertices in this graph
    for(std::pair<vertex_iter, vertex_iter> vp = boost::vertices(graph);
        vp.first != vp.second;
        ++vp.first) 
    {
      // Get a shared pointer to the existing block, for convenience
      boost::shared_ptr<RTT::TaskContext> existing_block = graph[*vp.first].block;

      // Make sure we're not connecting the block to itself
      if(existing_block != new_block) {

        // Connect all ports in this layer
        for(RTT::Service::ProviderNames::iterator group_it = groups.begin();
            group_it != groups.end();
            ++group_it)
        {
          // Check if the existing block uses this control group
          if(conman::has_group(existing_block,layer,*group_it)) {

            // Connect existing outputs to new inputs:
            // existing[control][control_group][out][*] --> new[control][control_group][in][*]
            for(RTT::Service::ProviderNames::iterator port_it = inputs[*group_it].begin();
                port_it != inputs[*group_it].end();
                ++port_it)
            {
              boost::shared_ptr<RTT::base::PortInterface> 
                out_port = conman::get_port(existing_block,layer,*group_it,"out",*port_it),
                in_port = conman::get_port(new_block,layer,*group_it,"in",*port_it);

              // Check if the port exists
              if(out_port.get()) {
                // Connect the port
                //out_port->connectTo(in_port.get());
                // Add the edge to the graph
                conman::EdgeProperties edge_props = {false, out_port, in_port};
                boost::add_edge(*vp.first, new_vertex, edge_props);
              }
            }

            // Connect existing inputs to new outputs
            // existing[control][control_group][in][*] <-- new[control][control_group][out][*]
            for(RTT::Service::ProviderNames::iterator port_it = outputs[*group_it].begin();
                port_it != outputs[*group_it].end();
                ++port_it)
            {
              boost::shared_ptr<RTT::base::PortInterface> 
                out_port = conman::get_port(new_block,layer,*group_it,"out",*port_it),
                in_port = conman::get_port(existing_block,layer,*group_it,"in",*port_it);

              // Check if the port exists
              if(in_port.get()) {
                // Connect the port
                //out_port->connectTo(in_port.get());
                // Add the edge to the graph
                conman::EdgeProperties edge_props = {false, out_port, in_port};
                boost::add_edge(new_vertex, *vp.first, edge_props);
              }
            }
          }
        }
      }
    }

    return true;
  }

  class Block : public RTT::TaskContext {
  public:

    // TODO: replace this with a class with friendler member functions
    typedef std::map<std::string, std::map<std::string,int> > ResourceMap;

    // Access control
    static const enum {
      UNRESTRICTED,
      EXCLUSIVE
    };

    Block(std::string const& name) :
      RTT::TaskContext(name, RTT::base::TaskCore::PreOperational)
    { 
      // Create default services
      this->provides("control");
      this->provides("feedback");
    }

    RTT::base::PortInterface& add_conman_port(
        const std::string &layer,
        const std::string &group_name,
        const std::string &direction,
        const std::string &port_name,
        RTT::base::PortInterface &port,
        const int exclusion)
    {
      // Store the exclusion mode
      
      // Add the port & pass-through normal interface
      return this->provides(layer)->provides(group)->provides(direction)->addPort(port_name, port);
    }

  };

}

class MyEffortController : public conman::Block {
public:
  Controller(std::string const& name, std::string const& group) :
    conman::Block(name),
    group_(group)
  {
    // Get group name from rosparam
    // TODO: get configuration
     
    // Store provided interfaces
    // TODO: should we support operations, or just ports? Probably just ports.
      
    // Create RTT ports
    this->add_conman_port("control",group,"out","joint_effort",effort_out_,conman::EXCLUSIVE);
    this->add_conman_port("control",group,"in","joint_effort",effort_in_,conman::EXCLUSIVE);
  }

  // TODO: default (empty) in base class
  const conman::ResourceMap get_control_input_resources() { 
    return conman::ResourceMap(); 
  }

  const conman::ResourceMap get_control_input_resources() { 
    return input_resource_map_; 
  }
  const conman::ResourceMap get_control_output_resources() {
    return output_resource_map_; 
  }

  bool configureHook() {
    //bool ready = this->requires("in")->requires(group_)->getReferencedService->getPort("joint_effort")
    
    // Ready if the input is connected
    bool ready = effort_in_.connected();

    return ready;
  }
  
  std::string group_;

  RTT::Service::shared_ptr input_service_;
  RTT::Service::shared_ptr output_service_;

  RTT::InputPort<double> effort_in_;
  RTT::OutputPort<double> effort_out_;

  // Resource maps
  conman::ResourceMap 
    input_resource_map_,
    output_resource_map_;
};

class ControllerManager : public OCL::DeploymentComponent {
public:
  ControllerManager(std::string name="ControllerManager") :
    OCL::DeploymentComponent(name)
  {
    
  }

  //! Controllers
  // load controller (name) 
  //  This is where interfaces and ports get connected
  //    new component c_new
  //    store store resources of c_new associated with a reference to c_new
  //    for each component c
  //      for each control group
  //        for each resource
  //          if c has a similar group/resource, connect it

  //! Load a controller/estimator block
  bool load_block(const std::string &block_name,
                  const std::string &package_name, 
                  const std::string &component_type)
  {
    // Load package
    bool package_imported = this->import(package_name);
    // Load block
    bool component_loaded = this->loadComponent(block_name, component_type);
    // Get the newly loaded block
    boost::shared_ptr<RTT::TaskContext> new_block(this->myGetPeer(block_name));

    // Connect the block in the appropriate ports in the control and feedback graphs
    conman::add_block(new_block, control_graph, "control");
    conman::add_block(new_block, feedback_graph, "feedback");

    return true;
  }

  //    
  // enable controller (name)
  //  This is where resource exclusion is checked

  bool enable_block(const std::String &block_name, const bool force)
  {
    // Get block
    // Check if enabling would violate exclusivity 
    //  - get other inputs from graph
    //  - count inputs
    //  - we can enable it if it won't violate exclusivity with its outputs
    //  - we can enable it if we disable any blocks which would violate this
    //  block's exclusivity
    //
    //  use boost::edge_range to get all edges between this block and other
    //  blocks, and enable connect the ports associated with each edge
    //
    // Start the block

    return true;
  }

  //
  // disable controller (name)
  // switch controllers (name)
  

  //! State Estimators
  // load estimator (name)
  // enable estimator (name)
  // disable estimator (name)
  // switch estimators (name)

  //! Execution
  // read from hardware ()
  // compute feedback ()
  // compute control ()
  // write to hardware ()

  // TODO: ROS service call interface (make as a separate thing?)
  
protected:

  // serialized feedback graph
  // serialized control graph

  std::map<std::string,std::vector<std::string> > resources_; 
};

int ORO_main(int argc, char** argv) {

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  RTT::Logger::In in("Prototype");

  std::vector<std::string> components;
  std::map<std::string,std::vector<std::string> > resources; 

  Controller c0("c0","left_arm");
  Controller c1("c1","left_arm");
  Controller c2("c2","left_arm");

  {
    OCL::DeploymentComponent deployer("ControllerManager"); 

    // Create some controllers
    deployer.connectPeers(&c0);
    deployer.connectPeers(&c1);
    deployer.connectPeers(&c2);

    // Get the control groups of a given controller
    RTT::Logger::log() << RTT::Logger::Info << "Control groups: " << RTT::endlog();

    deployer.connect("c0.control.out.left_arm.joint_effort","c1.control.in.left_arm.joint_effort",RTT::ConnPolicy());

    OCL::TaskBrowser task_browser(&deployer);

    task_browser.loop();
  }

  return 0;
}

/**

  connect("c0.left_arm__in.joint_effort","c1.left_arm__out.joint_effort",ConnPolicy())

  **/
