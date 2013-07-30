
#ifndef __CONMAN_SCHEME_H
#define __CONMAN_SCHEME_H

#include <conman_proto/conman.h>

namespace conman
{
  //! Manager for loading/unloading and starting/stopping Blocks
  class Scheme : public Block
  {
  public:
    Scheme(std::string name="Scheme");

    /** \brief Add a block that has already been constructed
     */
    bool add_peer(RTT::TaskContext *new_block);

    /** \brief Add a block which is already a peer of this component by name.
     */
    bool add_block(const std::string &name);

    /** \brief Enable a conman Block
     *
     * This is where port exclusivity (Block::ExclusionMode) gets checked. If
     * enabling this port would violate the exclusivity of another port, then it
     * will not be enabled unless force is specificed as true. If force is
     * specified, then any conflicting blocks will be disabled, themselves.
     *
     *
     * - Get block
     * - Check if enabling would violate exclusivity 
     *  - get other inputs from graph
     *  - count inputs
     *  - we can enable it if it won't violate exclusivity with its outputs
     *  - we can enable it if we disable any blocks which would violate this
     *  block's exclusivity
     *
     * - use boost::edge_range to get all edges between this block and other
     *  blocks, and enable connect the ports associated with each edge
     *
     * - Start the block
     */
    bool enable_block(const std::string &block_name, const bool force);

    //
    // disable controller (name)
    // switch controllers (name)


    // State Estimators
    // load estimator (name)
    // enable estimator (name)
    // disable estimator (name)
    // switch estimators (name)

    // Execution
    // read from hardware ()
    // compute estimation ()
    // compute control ()
    // write to hardware ()

    //! Read from hardware, compute estimation, compute control, and write to hardware.
    void updateHook();

    // TODO: ROS service call interface (make as a separate thing?)

  protected:

    //! \name Graph structures
    //\{
    conman::graph::CausalGraph
      control_graph_,
      estimation_graph_;
    //\}

    //! \name Topologically-sorted structures
    //\{
    conman::graph::CausalOrdering
      control_serialization_,
      estimation_serialization_;
    //\}

    /** Connect a block to the appropriate blocks in a given graph
     * This connects all inputs/outputs of block_a to all outputs/inputs of
     * block_b, given the groups, inputs, and outputs of block_b
     *
     * This is an internal function. For adding a block from
     * the public API, see \ref add_block.
     */
    static bool add_block_to_graph(
        RTT::TaskContext *new_block,
        conman::graph::CausalGraph &graph,
        const std::string &layer);
  };

  /** \name Convenience functions **/
  //\{

  /** \brief Check of a block has a group **/
  static bool has_group(
      RTT::TaskContext *block,
      const std::string &layer, 
      const std::string &group) 
  {
    return (block != NULL
            && block->provides()->hasService(layer)
            && block->provides(layer)->hasService(group));
  }

  /** \brief Check if a block has a specific port **/
  static bool has_port(
      RTT::TaskContext *block,
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
  static RTT::Service::ProviderNames get_groups(
      RTT::TaskContext *block,
      const std::string &layer)
  {
    return (block != NULL) ? 
      (block->provides(layer)->getProviderNames()) 
      :
      (RTT::Service::ProviderNames());
  }

  /** \brief Get a list of port names for s specific conduit **/
  static RTT::Service::ProviderNames get_ports(
      RTT::TaskContext *block,
      const std::string &layer,
      const std::string &group,
      const std::string &direction)
  {
    return (block != NULL) ? 
      ( block->
        provides(layer)-> 
        provides(group)->
        provides(direction)-> 
        getPortNames())  
      :
      (RTT::Service::ProviderNames());
  }

  /** \brief Get the service for a specific group **/
  static RTT::Service::shared_ptr get_group(
      RTT::TaskContext *block,
      const std::string &layer, 
      const std::string &group) 
  {
    return (block != NULL) ?
      (block->provides(layer)->provides(group))
      : 
      (RTT::Service::shared_ptr((RTT::Service*)NULL));
  }

  /** \brief Get the input service for a group **/
  static RTT::Service::shared_ptr get_input_service(
      RTT::TaskContext *block,
      const std::string &layer, 
      const std::string &group) 
  {
    return (block != NULL) ? 
      (block->provides(layer)->provides(group)->provides("in"))
      : 
      (RTT::Service::shared_ptr((RTT::Service*)NULL));
  }

  /** \brief Get the output service for a group **/
  static RTT::Service::shared_ptr get_output_service(
      RTT::TaskContext *block,
      const std::string &layer, 
      const std::string &group) 
  {
    return (block != NULL) ?
      (block->provides(layer)->provides(group)->provides("out"))
      :
      (RTT::Service::shared_ptr((RTT::Service*)NULL));
  }

  /** \brief Get a specific port **/
  static RTT::base::PortInterface* get_port(
      RTT::TaskContext *block,
      const std::string &layer, 
      const std::string &group,
      const std::string &direction,
      const std::string &port) 
  {
    return (block != NULL) ?
      (block->
       provides(layer)->
       provides(group)->
       provides(direction)->
       getPort(port))
      :
      (NULL);
  }

  //\}
}

#endif // ifndef __CONMAN_SCHEME_H
