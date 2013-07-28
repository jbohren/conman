
#ifndef __CONMAN_BLOCK_H
#define __CONMAN_BLOCK_H

#include <conman_proto/conman.h>

namespace conman {

  //! Specialization of RTT::TaskContext to represent a control and/or estimation block in a control system.
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
        const ExclusionMode mode);

    /// Get the exclusion mode for a given port
    ExclusionMode get_exclusion(
        const std::string &layer,
        const std::string &group_name,
        const std::string &direction,
        const std::string &port_name);

    //\}

    //! Construct a conman Block with the standard "control" and "estimation" layers.
    Block(std::string const& name);

    //! Add an RTT port with a conman interface and exclusion mode.
    RTT::base::PortInterface& add_conman_port(
        const std::string &layer,
        const std::string &group_name,
        const std::string &direction,
        const std::string &port_name,
        const ExclusionMode exclusion_mode,
        RTT::base::PortInterface &port);

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
    //! Compute estimation / state estimation and write to ports in the "estimation" layer.
    virtual void compute_estimation(
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

}

#endif // ifndef __CONMAN_BLOCK_H
