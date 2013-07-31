
#ifndef __CONMAN_SCHEME_H
#define __CONMAN_SCHEME_H

#include <conman_proto/conman.h>
#include <conman_proto/block.h>

namespace conman
{
  //! Manager for loading/unloading and starting/stopping Blocks
  class Scheme : public Block
  {
  public:
    Scheme(std::string name="Scheme");

    //! \name Scheme Construction
    //\{
    /** \brief Add a block that has already been constructed
     */
    bool add_peer(RTT::TaskContext *new_block);

    /** \brief Add a block which is already a peer of this component by name.
     */
    bool add_block(const std::string &name);
    //\}

    //! \name Runtime Scheme Control
    //\{
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

    // TODO: disable controller (name)
    // TODO: switch controllers (name)
    
    //\}

    bool configureHook();
    bool startHook();
    //! Read from hardware, compute estimation, compute control, and write to hardware.
    void updateHook();

    // TODO: ROS service call interface (make as a separate thing?)

  protected:

    RTT::os::TimeService::nsecs last_update_time_;

    //! \name Graph structures
    //\{
    conman::graph::CausalGraph
      estimation_graph_,
      control_graph_;
    //\}

    //! \name Topologically-sorted structures
    //\{
    conman::graph::CausalOrdering
      estimation_serialization_,
      control_serialization_;
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
        conman::graph::CausalOrdering &ordering,
        const std::string &layer);
  };
}

#endif // ifndef __CONMAN_SCHEME_H
