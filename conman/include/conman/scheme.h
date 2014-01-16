/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_SCHEME_H
#define __CONMAN_SCHEME_H

#include <conman/conman.h>

namespace conman
{
  /** \brief Manager for scheduling execution and starting/stopping blocks
   *
   * The Scheme maintains a data flow graph (DFG) and execution scheduling graph
   * (ESG) which are used to model the data flow between Orocos components in
   * the schcme and execute those components in a serialized order,
   * respectively. These two graphs have identical vertex sets, but are
   * distinguished by their arc sets. Specifically, the ESG arc set is a subset
   * of the DFG arc set constructed by removing arcs in order to break cycles in
   * the DFG.
   *
   * The ESG is "executable" when it has no cycles.
   *
   */
  class Scheme : public RTT::TaskContext
  {
  public:
    /** \brief Construct a Scheme */
    Scheme(std::string name="Scheme");

    ///////////////////////////////////////////////////////////////////////////
    /** \name Scheme Construction
     *
     * Each time an RTT component is added to the scheme, it is modeled in the
     * Data Flow Graph (DFG) by a vertex and the set of connections from output
     * to input ports between any two components are modeled as single arcs.
     * Components and their connections are also modeled by the Execution
     * Scheduling Graph (ESG). At execution time, the ESG must be acyclic.
     *
     * Each time a block is added or removed, the Runtime Conflict Graph (RCG)
     * is recomputed. This undeirected graph models the groups of
     * mutually-exclusive RTT components (components that cannot be run
     * simultaneously) as adjacent vertices.
     */
    //\{

    //! Check if a block is in the scheme
    bool hasBlock(const std::string &name) const;

    //! Get the names of all the blocks in this scheme
    std::vector<std::string> getBlocks() const;

    //! Get the names of all the blocks in this scheme
    void getBlocks(std::vector<std::string> &blocks) const;

    /** \brief Add a block which is already a peer of this scheme by name. */
    bool addBlock(const std::string &name);

    /*** \brief Add a block that has already been constructed, and add it as a
     * peer of this scheme.*/
    bool addBlock(RTT::TaskContext *new_block);

    /** \brief Remove a block from the scheme by name */
    bool removeBlock(const std::string &name);

    /** \brief Remove a block from the scheme */
    bool removeBlock(RTT::TaskContext *block);

    //\}

    ///////////////////////////////////////////////////////////////////////////
    /** \name Scheme Block Group Management
     *
     * Block groups can be treated like "meta blocks" in that they can be
     * switched on and off by name. A block can belong to no groups, or it can
     * belong to many groups.
     *
     * Note that block group names must not be the same as normal block names.
     *
     * Unlike normal blocks, they can only be enabled or disabled by name since
     * groups aren't represented by single TaskContext pointers.
     *
     * Groups can also be composed of other groups. In this case, the
     * hierarchical structure of the groups is maintained in conman's internal
     * representation.
     */
    //\{
    
    //! Check if a group exists
    bool hasGroup(const std::string &group_name) const;
    
    //! Check if a group exists
    std::vector<std::string> getGroups() const;

    //! Create an initially empty group
    bool addGroup( const std::string &group_name);

    //! Set the membership for a group with just one member
    bool setGroup(
        const std::string &group_name,
        const std::string &member_name);

    //! Set the membership for a group
    bool setGroup(
        const std::string &group_name,
        const std::vector<std::string> &members);

    //! Add a single block to a group
    bool addToGroup(
        const std::string &member_name,
        const std::string &group_name);

    //! Remove a single block from a group
    bool removeFromGroup(
        const std::string &group_name,
        const std::string &member_name);

    //! Remove the blocks from a group, and then remove the group
    bool emptyGroup(const std::string &group_name);

    //! Remove the blocks from a group, and then remove the group
    bool removeGroup(const std::string &group_name);

    //! Get the blocks in a given group
    bool getGroupMembers(
        const std::string &group_name,
        std::vector<std::string> &members) const;

    //! Get the blocks in a given group
    std::vector<std::string> getGroupMembers(
        const std::string &group_name) const;

    //\}
    
    ///////////////////////////////////////////////////////////////////////////
    /** \name Block conflict computation
     *
     * Block conflicts are inferred from RTT data port exclusivity, as declared
     * by each given block.
     */
    //\{
    
    //! Compute the conflicts between all blocks in the scheme
    void computeConflicts();
    //! Compute the conflicts with a single block in the scheme by name
    void computeConflicts(const std::string &block_name);
    //! Compute the conflicts with a list of blocks in the scheme by name
    void computeConflicts(const std::vector<std::string> &block_names);
    //! Compute the conflicts with a single block in the scheme
    void computeConflicts(conman::graph::DataFlowVertex::Ptr block);

    //\}

    ///////////////////////////////////////////////////////////////////////////
    /** \name Feedback Cycle Management
     *
     * These functions are used to "latch" and "unlatch" connections between
     * components in order to break feedback cycles in the Execution Scheduling
     * Graph (ESG) so that the scheme can be executed causally. This means that
     * every cycle in the data flow graph must have at least one latched
     * connection. 
     *
     * Latching is done by component, that means that if two components A and B
     * are connected by several data port connections, either all of the ports
     * from A to B must be latched, or all of the ports from A to B must be
     * unlatched.
     *
     * The scheme is represented by a directed graph. If that graph is acyclic,
     * then the scheme can be executed. Adding a latch on a given connection
     * removes arcs from that representation, and thus breaks cycles depending
     * on those arcs.
     *
     * Note that self-loops are implicitly latched, so adding latches on the
     * connections from a component to itself is a no-op.
     *
     */
    //\{

    //! Add/Remove a latch between two blocks (or two groups of blocks) by name
    bool latchConnections(
      const std::string &source_name,
      const std::string &sink_name,
      const bool latch);

    bool latchConnections(
      const std::vector<std::string> &source_names,
      const std::vector<std::string> &sink_names,
      const bool latch);

    //! Add/Remove a latch between two blocks
    bool latchConnections(
      RTT::TaskContext *source,
      RTT::TaskContext *sink,
      const bool latch,
      const bool strict = true);

    //! Set latching for all current and future input arcs to a given block
    bool latchInputs(const std::string &name, const bool latch);
    //! Set latching for all current and future input arcs to a given block
    bool latchInputs(RTT::TaskContext *block, const bool latch);
    //! Set latching for all current and future input arcs to a given block
    bool latchOutputs(const std::string &name, const bool latch);
    //! Set latching for all current and future input arcs to a given block
    bool latchOutputs(RTT::TaskContext *block, const bool latch);

    //\}
    
    ///////////////////////////////////////////////////////////////////////////
    /** \name Latch Analysis
     *
     * These functions analyze the latched edges in the Data Flow Graph
     * (DFG) which are removed in order to construct the Execution Scheduling
     * Graph (ESG).
     *
     * The latch count for a given path is simply the number of pairs of
     * components on the path with latched connections. 
     */

    /** \brief Computes all simple cycles in the Data Flow Graph (DFG) 
     *
     * This uses Tiernan's Elementary Circuit Algorithm (implementation from the
     * Boost Graph Library) to find all cycles which do not include repeated
     * vertices. It returns the number of cycles found.
     */
    int getFlowCycles(std::vector<std::vector<std::string> > &cycles) const;

    /** \brief Get the number of latches in a given path through the DFG. */
    int latchCount(const std::vector<std::string> &path) const;

    /** \brief Get the maximum number of latches in any cycle in the DFG. */
    int maxLatchCount() const;

    /** \brief Get the minimum number of latches in any cycle in the DFG. If the
     * DFG has no cycles, this returns 0.*/
    int minLatchCount() const;

    //\}

    ///////////////////////////////////////////////////////////////////////////
    /** \name Cycle Introspection
     *
     * These functions perform the following analyses on the pending execution
     * scheduling graph (ESG):
     *
     * * Determine the existence of simple cycles in the pending ESG
     * * Compute all cyles in the pending ESG (useful for debugging)
     * 
     * Graph cycles prevent the scheduler from determining where to begin and
     * end each computation. Cycles can be broken by adding latches at the
     * appropriate points, but different numbers of latching between signal
     * paths can cause problems when fusing such data.
     */

    /** \brief Returns true if the pending execution scheduling graph has no
     * cycles.
     * 
     * This determines the existence of cycles by attempting a topological sort
     * (implementation from the Boost Graph Library) of the execution scheduling
     * graph (ESG). If the ESG is acyclic, it can be executed. This property is
     * required to start() the scheme.
     */
    bool executable() const;

    /** \brief Computes all simple cycles in the pending Execution Scheduling
     * Graph (ESG).
     *
     * This uses Tiernan's Elementary Circuit Algorithm (implementation from the
     * Boost Graph Library) to find all cycles which do not include repeated
     * vertices. It returns the number of cycles found.
     */
    int getExecutionCycles(std::vector<std::vector<std::string> > &cycles) const;

    /** \brief Gets the execution order for the scheme or if it can't be
     * executed, returns false.
     */
    bool getExecutionOrder(std::vector<std::string> &order) const;

    //\}


    ///////////////////////////////////////////////////////////////////////////
    /** \name Runtime Scheme Control
     *
     * <b> Enabling Blocks </b>
     *
     * "Enabling" a block corresponds to starting it through RTT's normal
     * startHook() mechanism. If it is currently running, it is considered
     * enabled.
     *
     * This is where block conflicts get checked. If enabling this port
     * conflicts with another block (like if enabling it would violate the
     * exclusivity of an input port), then it will not be enabled unless force
     * is specificed as true. If force is specified, then any conflicting
     * blocks will be disabled, themselves.
     *
     * This attempts to start() the block, which requires that the block be
     * configured before enableBlock() is called. This is because calls to
     * configure() may block for unacceptable lengths of time.
     *
     *
     * <b> Disabling Blocks </b>
     *
     * "Disabling" a block corresponds to stopping it through RTT's normal
     * stopHook() mechanism. If it is not currently running, a block is
     * considered disabled. 
     *
     */
    //\{

    //! Enable a single conman Block
    bool enableBlock(RTT::TaskContext *block, const bool force);
    //! Enable a single Conman block (or group) by name
    bool enableBlock(const std::string &block_name, const bool force);
    /** \brief Enable multiple Conman blocks (or groups) by name simultaneously
     * 
     * This will enable the blocks in the given vector according to the
     * provided order. This operation is atomic unless something goes wrong. If
     * \param force is not set, and any of the blocks in the list conflict with
     * any running blocks, no blocks will be enabled.
     */
    bool enableBlocks(
        const std::vector<std::string> &block_names, 
        const bool strict,
        const bool force);

    //! Disable a single conman Block
    bool disableBlock(RTT::TaskContext *block);
    //! Disable a single Conman block (or group) by name
    bool disableBlock(const std::string &block_name);
    //! Disable all Conman blocks simultaneously
    bool disableBlocks(const bool strict);
    //! Disable multiple Conman blocks (or groups) by name simultaneously
    bool disableBlocks(
        const std::vector<std::string> &block_names,
        const bool strict);

    /*** \brief Try to disable a set of blocks (or groups) and enable another
     * set of blocks (or groups)
     *
     * NOTE: This function first disables the blocks on the disable list, and
     * then it enables blocks on the enable list.
     * 
     * \param strict Break on error if true, otherwise, try to switch the modes
     * of all blocks listed even if some fail.
     *
     */
    bool switchBlocks(
        const std::vector<std::string> &disable_block_names,
        const std::vector<std::string> &enable_block_names, 
        const bool strict, 
        const bool force);
    

    /*** \brief Set the set of enabled and disabled blocks.
     *
     * This is equivalent to calling \ref disable_blocks for all blocks, and
     * then calling \ref enable_blocks for a list of blocks,
     *
     * NOTE: This function does not provide a "force" option like \ref
     * enableBlock or switchBlocks, because the only conflicts that are
     * possible are in the list of blocks to be enabled, and the caller should
     * know whether or not these are in conflict.
     *
     * \param strict Break on error if true, otherwise, try to enable all
     * blocks on the list even if some can't be enabled.
     */
    bool setEnabledBlocks(
        const std::vector<std::string> &enabled_block_names, 
        const bool strict);

    //\}

    /** \brief (Re)generates an internal model of the RTT port connection graph
     *
     * This will populate the Data Flow Graph (DFG), the Execution Scheduling
     * Graph (ESG), and the Runtime Conflict Graph (RCG). The scheme must be 
     *
     */
    bool regenerateModel();

    ///////////////////////////////////////////////////////////////////////////
    //! \name Orocos RTT Hooks
    //\{

    virtual bool configureHook();

    /** \brief Verify that the Execution Scheduling Graph can be executed
     * serially.
     *
     */
    virtual bool startHook();

    /** \brief Execute one iteration of the Scheme
     *
     * Read from hardware, compute estimation, compute control, and write to
     * hardware.
     */
    virtual void updateHook();

    //\}

  protected:

    /** \brief The last time updateHook was called.
     *
     * We maintain a single update time for all blocks so that any blocks
     * running at the same rate are executed in the same update() cycle.
     *
     */
    RTT::os::TimeService::nsecs last_update_time_;

    /** \brief A map from block names onto flow graph vertex properties (for
     * fast access)
     */
    std::map<std::string,conman::graph::DataFlowVertex::Ptr> blocks_;
    //! A list of blocks ordered by index (for linear re-indexing)
    std::list<conman::graph::DataFlowVertex::Ptr> block_indices_;
    //! A map of block group names to block names
    conman::GroupMap block_groups_;

    //! \name Data Flow Graph Structures
    //\{
    //! Data Flow Graph (DFG) 
    conman::graph::DataFlowGraph flow_graph_;
    //! Mappings from TaskContext pointers to boost vertex descriptors
    conman::graph::DataFlowVertexTaskMap flow_vertex_map_;

    //! \name Execution Sampling Graph Structures
    //\{
    //! Execution Scheduling Graph (ESG)
    conman::graph::DataFlowGraph exec_graph_;
    //! Mappings from TaskContext pointers to boost vertex descriptors
    conman::graph::DataFlowVertexTaskMap exec_vertex_map_;
    //! Topologically sorted ordering of each graph
    conman::graph::ExecutionOrdering exec_ordering_;
    //\}

    //! \name Runtime Conflict Graph Structures
    //\{
    /** \brief Graph representing block conflicts 
     *
     * Adjacent vertices in the Rtunime Conflict Graph represent components
     * that can't run simultaneously.
     */
    conman::graph::ConflictGraph conflict_graph_;
    /** \brief A map from RTT TaskContext pointers to vertex identifiers in the
     * conflict graph
     */
    conman::graph::ConflictVertexMap conflict_vertex_map_;
    //\}

    /** \brief Connect a block in the graph structures
     *
     * This will model a block in the Data Flow, Execution Scheduling, and
     * Runtime Conflict graphs.
     */
    bool addBlockToGraph(conman::graph::DataFlowVertex::Ptr new_vertex);

    /** \brief Remove a block from the flow graphs
     */
    bool removeBlockFromGraph(conman::graph::DataFlowVertex::Ptr vertex);

    /** \brief Recursively get a flattened list of all members in a group
     *
     * This is the internal function used by the public \ref getGroupMembers.
     */
    bool getGroupMembers(
        const std::string &group_name,
        std::set<std::string> &members,
        std::set<std::string> &visited) const;

    //! Brief compute cycles in a specific flow graph.
    int computeCycles(
        const conman::graph::DataFlowGraph &data_flow_graph,
        std::vector<conman::graph::DataFlowPath> &cycles) const;

    //! Compute the schedule without modifying the scheme
    bool computeSchedule(
        const conman::graph::DataFlowGraph &data_flow_graph,
        conman::graph::ExecutionOrdering &ordering, 
        const bool quiet) const;

    //! Print out the current execution ordering
    void printExecutionOrdering() const;

    //! Time state
    //TODO: use nsecs instead?
    RTT::Seconds
      last_exec_time_,
      last_exec_period_,
      min_exec_period_,
      max_exec_period_;

    //! Execution statistics
    //TODO: use nsecs instead?
    RTT::Seconds
      last_exec_duration_,
      min_exec_duration_,
      max_exec_duration_,
      smooth_exec_duration_;

    size_t n_running_blocks_;
  };
}

#endif // ifndef __CONMAN_SCHEME_H
