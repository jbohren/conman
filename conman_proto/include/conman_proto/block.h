
#ifndef __CONMAN_BLOCK_H
#define __CONMAN_BLOCK_H

#include <conman_proto/conman.h>

namespace conman {

  //! Specialization of RTT::TaskContext to represent a control and/or estimation block in a control system.
  class Block : public RTT::TaskContext 
  {
  public:
    /** \brief Exclusivity modes describe how a given port can be accessed. **/
    typedef enum {
      //! No exclusivity mode set / unknown port.
      UNDEFINED,
      //! Any number of connections.
      UNRESTRICTED,
      //! Limit to one connection.
      EXCLUSIVE
    } ExclusivityMode;

    static bool HasConmanInterface(RTT::TaskContext *block);

  private:

    //! Execution rate for this component
    double execution_rate_;

    //! Service for Conman's RTT API
    RTT::Service::shared_ptr conman_service_;

    //! Exclusivity mode container for storing exclusivity modes for each port.
    std::map<std::string, ExclusivityMode> exclusivity_;
    std::map<std::string, std::set<std::string> > conman_ports_;

  public:
    
    //! Construct a conman Block with the standard "control" and "estimation" layers.
    Block(std::string const& name);

    /** \name Conman Port Management
     */
    //\{
    //! Add an RTT port with a conman interface and exclusivity mode.
    RTT::base::PortInterface& registerConmanPort(
        const std::string &layer,
        const ExclusivityMode exclusivity_mode,
        RTT::base::PortInterface &port);

    //! Get the registered conman ports for a given layer
    void getConmanPorts(
        const std::string &layer,
        std::vector<std::string> &port_names);
    //\}


    /** \name Port Exclusivity Management
     *  Set and get the \ref ExclusivityMode of a given port.
     */
    //\{

    //! Set the exclusivity mode for a given port
    void setExclusivity(
        const std::string &port_name,
        const ExclusivityMode mode) ;

    //! Get the exclusivity mode for a given port
    const ExclusivityMode getExclusivity(const std::string &port_name);
    //\}

    /** \name Execution Hooks
     * Member functions to overload in block implementations.
     * These functions are each given the time of the latest event (time) and the
     * time since the last event (period).
     */
    //\{

    //! Read from lower-level hardware API if necessary.
    virtual void readHardwareHook(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) {}
    //! Compute estimation / state estimation and write to ports in the "estimation" layer.
    virtual void computeEstimationHook(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) {}
    //! Compute control commands and write to ports in the "control" layer.
    virtual void computeControlHook(
        RTT::os::TimeService::Seconds time, 
        RTT::os::TimeService::Seconds period) {}
    //! Write to lower-level hardware API if necessary.
    virtual void writeHardwareHook(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) {}
    
    //\}
  
  private:
    //! Read from lower-level hardware API if necessary.
    void readHardware(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) { this->readHardwareHook(time, period); }
    //! Compute estimation / state estimation and write to ports in the "estimation" layer.
    void computeEstimation(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) { this->computeEstimationHook(time, period); }
    //! Compute control commands and write to ports in the "control" layer.
    void computeControl(
        RTT::os::TimeService::Seconds time, 
        RTT::os::TimeService::Seconds period) { this->computeControlHook(time, period); }
    //! Write to lower-level hardware API if necessary.
    void writeHardware(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period) { this->writeHardwareHook(time, period); }

  };

}

#endif // ifndef __CONMAN_BLOCK_H
