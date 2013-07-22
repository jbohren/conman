
#include <rtt/os/main.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>


/**
 * Conman Controller Manager
 *
 * Interfaces
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

  /** 
   * Causal block graph for topologically sorting control and feedback
   * networks. This graph contains vertices which correspond to blocks, and
   * edges which correspond to port connections between blocks.
   *
   * Vertex Type: listS
   *  - low time complexity
   *
   * Edge Type: listS
   *  - low time complexity
   *  - permits parallel edges to describe multiple links between blocks
   *
   * Directed: true
   * 
   */
  //! Causal Graph Description
  namespace graph {

    //! Boost Graph Edge Metadata
    struct EdgeProperties {
      //! True if the ports are connected
      bool connected;
      //! The output port
      boost::shared_ptr<RTT::base::PortInterface> out;
      //! The input port:
      boost::shared_ptr<RTT::base::PortInterface> in;
    };

    //! Boost Graph Vertex Metadata
    struct VertexProperties {
      //! The control and/or feedback block (depending on which graph it's in)
      boost::shared_ptr<RTT::TaskContext> block;
    };

    //! Boost Graph Type
    typedef boost::adjacency_list<
      boost::listS, boost::listS, boost::directedS,
      VertexProperties, EdgeProperties> CausalGraph;
  }

  /** \name Convenience functions **/

  /** \brief Check of a block has a group **/
  bool has_group(boost::shared_ptr<RTT::TaskContext> block,
                 const std::string &layer, 
                 const std::string &group) 
  {
    return (block->provides()->hasService(layer)
            && block->provides(layer)->hasService(group));
  }

  /** \brief Check if a block has a specific port **/
  bool has_port(boost::shared_ptr<RTT::TaskContext> block,
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
      boost::shared_ptr<RTT::TaskContext> block,
      const std::string &layer)
  {
    return block->provides(layer)->getProviderNames();
  }

  /** \brief Get a list of port names for s specific conduit **/
  RTT::Service::ProviderNames get_ports(
      boost::shared_ptr<RTT::TaskContext> block,
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
      boost::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group) 
  {
    return block->provides(layer)->provides(group);
  }

  /** \brief Get the input service for a group **/
  RTT::Service::shared_ptr get_input_service(
      boost::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group) 
  {
    return block->provides(layer)->provides(group)->provides("in");
  }

  /** \brief Get the output service for a group **/
  RTT::Service::shared_ptr get_output_service(
      boost::shared_ptr<RTT::TaskContext> block,
      const std::string &layer, 
      const std::string &group) 
  {
    return block->provides(layer)->provides(group)->provides("out");
  }

  /** \brief Get a specific port **/
  boost::shared_ptr<RTT::base::PortInterface> get_port(
      boost::shared_ptr<RTT::TaskContext> block,
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

  //! Specialization of RTT::TaskContext to represent a control and/or feedback block in a control system.
  class Block : public RTT::TaskContext 
  {
  public:
    /** \brief Exclusion modes describe how a given port can be accessed. **/
    typedef enum {
      //! No exclusion mode set / unknown port.
      UNDEFINED,
      //! Any number of connections.
      UNRESTRICTED,
      //! Limit to one connection.
      EXCLUSIVE
    } ExclusionMode;

    /** \name Port Exclusivity Management
     *  Set and get the \ref ExclusionMode of a given port.
     */
    //\{

    /// Set the exclusion mode for a given port
    void set_exclusion(
        const std::string &layer,
        const std::string &group_name,
        const std::string &direction,
        const std::string &port_name,
        const ExclusionMode mode)
    {
      exclusion_[layer][group_name][direction][port_name] = mode; 
    }

    /// Get the exclusion mode for a given port
    ExclusionMode get_exclusion(
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

    //\}

    //! Construct a conman Block with the standard "control" and "feedback" layers.
    Block(std::string const& name) :
      RTT::TaskContext(name, RTT::base::TaskCore::PreOperational)
    { 
      // Create default services
      this->provides("control")->doc("Control interface layer. This service provides all control inputs & outputs for this block.");
      this->provides("feedback")->doc("Feedback interface layer. This service provides all feedback/state estimation inputs & outputs for this block.");
    }

    //! Add an RTT port with a conman interface and exclusion mode.
    RTT::base::PortInterface& add_conman_port(
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

    /** \name Execution Hooks
     * Member functions to overload in block implementations.
     * These functions are each given the time of the latest event (time) and the
     * time since the last event (period).
     */
    //\{

    //! Read from lower-level hardware API if necessary.
    virtual void read_hardware(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) {}
    //! Compute feedback / state estimation and write to ports in the "feedback" layer.
    virtual void compute_feedback(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) {}
    //! Compute control commands and write to ports in the "control" layer.
    virtual void compute_control(
        RTT::os::TimeService::Seconds time, 
        RTT::os::TimeService::Seconds period) {}
    //! Write to lower-level hardware API if necessary.
    virtual void write_hardware(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) {}
    
    //\}

  private:

    //! Exclusion mode container for storing exclusion modes for each port.
    typedef std::map<std::string, // layer
            std::map<std::string, // group
            std::map<std::string, // direction
            std::map<std::string, // port
            ExclusionMode> > > > ExclusionContainer;

    // Internal port exclusivity container
    ExclusionContainer exclusion_;

  };

  //! Manager for loading/unloading and starting/stopping Blocks
  class BlockManager : public OCL::DeploymentComponent 
  {
  public:
    BlockManager(std::string name="BlockManager") :
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
      add_block(new_block, control_graph_, "control");
      add_block(new_block, feedback_graph_, "feedback");

      // Recompute topological sort
      // TODO

      return true;
    }

    //    
    // enable controller (name)
    //  This is where resource exclusion is checked

    bool enable_block(const std::string &block_name, const bool force)
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
    void updateHook() 
    {
      // 
    }

    // TODO: ROS service call interface (make as a separate thing?)

  protected:

    /// Graph structures
    CausalGraph
      control_graph_,
      feedback_graph_;

    // Connect a block to the appropriate blocks in a given graph
    // This connects all inputs/outputs of block_a to all outputs/inputs of
    // block_b, given the groups, inputs, and outputs of block_b
    static bool add_block(
        boost::shared_ptr<RTT::TaskContext> new_block,
        conman::CausalGraph &graph,
        const std::string &layer)
    {
      // TODO: Validate this this taskcontext has a valid conman interface

      // Add this block to the graph
      boost::graph_traits<conman::CausalGraph>::vertex_descriptor new_vertex = boost::add_vertex(graph);
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
      typedef boost::graph_traits<CausalGraph>::vertex_iterator vertex_iter;

      // Iterate over all vertices in this graph
      for(std::pair<vertex_iter, vertex_iter> vp = boost::vertices(graph);
          vp.first != vp.second;
          ++vp.first) 
      {
        // Get a shared pointer to the existing block, for convenience
        boost::shared_ptr<RTT::TaskContext> existing_block = graph[*vp.first].block;

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
                boost::add_edge(*vp.first, new_vertex, edge_props, graph);
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
                boost::add_edge(new_vertex, *vp.first, edge_props, graph);
              }
            }
          }
        }
      }

      return true;
    }

    // serialized feedback graph
    // serialized control graph

    std::map<std::string,std::vector<std::string> > resources_; 
  };

  //! Interface types for ports used to connect Blocks
  namespace interfaces {

    struct SingleJointEffort { typedef double datatype; static const std::string name; };
    const std::string SingleJointEffort::name = "single_joint_effort";
    
  }
}

class MyEffortController : public conman::Block 
{
public:
  MyEffortController(std::string const& name, std::string const& group) :
    conman::Block(name),
    group_(group)
  {
    // Create RTT ports
    using namespace conman;
    using namespace conman::interfaces;

    this->add_conman_port("control",group,"in", conman::interfaces::SingleJointEffort::name, EXCLUSIVE, effort_in_)
      .doc("Effort input.");
    this->add_conman_port("control",group,"out", conman::interfaces::SingleJointEffort::name, EXCLUSIVE, effort_out_)
      .doc("Effort output = input + 1.");
  }

  bool configureHook() {
    // Ready if the input is connected
    bool ready = effort_in_.connected();

    return ready;
  }

  virtual void compute_control(
      RTT::os::TimeService::Seconds secs, 
      RTT::os::TimeService::Seconds period) 
  {
    conman::interfaces::SingleJointEffort::datatype effort;
    effort_in_.read(effort);
    effort_out_.write(effort + 1);
  }

private:
  std::string group_;

  RTT::InputPort<conman::interfaces::SingleJointEffort::datatype> effort_in_;
  RTT::OutputPort<conman::interfaces::SingleJointEffort::datatype> effort_out_;
};

int ORO_main(int argc, char** argv) {

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  RTT::Logger::In in("Prototype");

  std::vector<std::string> components;
  std::map<std::string,std::vector<std::string> > resources; 

  MyEffortController c0("c0","left_arm");
  MyEffortController c1("c1","left_arm");
  MyEffortController c2("c2","left_arm");

  {
    OCL::DeploymentComponent deployer("BlockManager"); 

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
