
#ifndef __CONMAN_SCHEME_H
#define __CONMAN_SCHEME_H

#include <conman_proto/conman.h>
#include <conman_proto/block.h>

namespace conman
{
  //! Manager for loading/unloading and starting/stopping Blocks
  class Scheme : public RTT::TaskContext
  {
  public:
    Scheme(std::string name="Scheme");

    //! \name Scheme Construction
    //\{
    /** \brief Add a block that has already been constructed
     */
    bool add_block(RTT::TaskContext *new_block);

    /** \brief Add a block which is already a peer of this component by name.
     */
    bool add_block(const std::string &name);

    bool set_group(
        const std::string &group_name,
        const std::vector<std::string> &grouped_blocks) { return false; }
    bool get_groups(
        const std::string &group_name,
        std::vector<std::string> &grouped_blocks) { return false; }
    bool remove_group( const std::string &group_name) { return false; }
    //\}
    
    const std::vector<std::string> & get_blocks() {
      return block_names_;
    }


    /** \brief Compute the conflicts between blocks in the scheme
     *
     *  TODO: Use boost::edge_range to get all edges between this block and
     *  other blocks, and enable connect the ports associated with each edge
     */
    void compute_conflicts() { }
    void compute_conflicts(const std::string &block_name) { }
    void compute_conflicts(const std::vector<std::string> &block_names) { }

    //! \name Runtime Scheme Control
    //\{
    /** \brief Enable a single conman Block
     *
     * This function / operation enables a single Conman block. This is where
     * block conflicts get checked. If enabling this port conflicts with another
     * block (like if enabling it would violate the exclusivity of an input
     * port), then it will not be enabled unless force is specificed as true. If
     * force is specified, then any conflicting blocks will be disabled,
     * themselves.
     *
     * This attempts to start() the block, which requires that the block be
     * configured before enable_block() is called. This is because calls to
     * configure() may block for unacceptable lengths of time.
     *
     */
    bool enable_block(RTT::TaskContext *block, const bool force);
    //! Enable a single Conman block by name
    bool enable_block(const std::string &block_name, const bool force);
    //! Enable multiple Conman blocks
    bool enable_blocks(
        const std::vector<std::string> &block_names, 
        const bool strict,
        const bool force);

    /** \brief Disable a conman Block
     *
     * This function / operation disables a single Conman block by stopping it
     * if it is currently running. 
     *
     */
    bool disable_block(RTT::TaskContext *block);
    //! Disable a single Conman block by name
    bool disable_block(const std::string &block_name);
    //! Disable multiple Conman blocks
    bool disable_blocks(
        const std::vector<std::string> &block_names,
        const bool strict);

    /** \brief Try to disable a set of blocks and enable another set of blocks
     *
     * NOTE: This function first disables the blocks on the disable list, and
     * then it enables blocks on the enable list.
     * 
     * \param strict Break on error if true, otherwise, try to switch the modes
     * of all blocks listed even if some fail.
     *
     */
    bool switch_blocks(
        const std::vector<std::string> &disable_block_names,
        const std::vector<std::string> &enable_block_names, 
        const bool strict, 
        const bool force);
    

    /** \brief Set the set of enabled and disabled blocks.
     *
     * This is equivalent to calling \ref disable_blocks for all blocks, and
     * then calling \ref enable_blocks for a list of blocks,
     *
     * NOTE: This function does not provide a "force" option like \ref
     * enable_block or switch_blocks, because the only conflicts that are
     * possible are in the list of blocks to be enabled, and the caller should
     * know whether or not these are in conflict.
     *
     * \param strict Break on error if true, otherwise, try to enable all blocks
     * on the list even if some can't be enabled.
     */
    bool set_blocks(
        const std::vector<std::string> &enabled_block_names, 
        const bool strict);

    //\}

    bool configureHook();
    bool startHook();
    //! Read from hardware, compute estimation, compute control, and write to hardware.
    void updateHook();

    // TODO: ROS service call interface (make as a separate thing?)

  protected:

    RTT::os::TimeService::nsecs last_update_time_;

    //! A list of block names (to distinguish from other peers)
    std::vector<std::string> block_names_;

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

    std::map<std::string, std::vector<RTT::TaskContext*> > block_conflicts_;

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
