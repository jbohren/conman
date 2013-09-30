/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_HOOK_SERVICE_H
#define __CONMAN_HOOK_SERVICE_H

#include <vector>
#include <set>

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <conman/conman.h>

namespace conman {
  
  /** \brief The Hook Service is used to attach RTT TaskContexts to a Conman Scheme.
   *
   * In confidence tricks, a "Hook" is an apparent advantage for the victim to
   * encourage them to take part in the scam.
   *
   */
  class HookService : public RTT::Service 
  {
  public:
    //! Construct a conman hook service
    HookService(RTT::TaskContext* owner);

    /** \name Conman Scheduling Management */
    //\{

    //! Set the desired minimum execution period
    bool setDesiredMinPeriod(const RTT::Seconds period);

    //! Get the desired minimum execution period
    RTT::Seconds getDesiredMinPeriod();

    //\}

    /** \name Conman Port Management */
    //\{

    //! Set the exclusivity mode for an input port
    bool setInputExclusivity(
        const std::string &port_name,
        const Exclusivity::Mode mode);

    //! Get the exclusivity mode for an input port
    conman::Exclusivity::Mode getInputExclusivity(
        const std::string &port_name);

    //\}

    /** \name Time Introspection */
    //\{
    //! Get the current execution time
    RTT::Seconds getTime();
    //! Get the period since the last execution time
    RTT::Seconds getPeriod();
    //\}

    /** \name Execution */
    //\{
    
    //! Initialize the time state & statistics
    bool init(const RTT::Seconds time);
    //! Execute the owner's update hook and compute execution time statistics
    bool update(const RTT::Seconds time);

    //\}
    //
  private:

    //! Init flag used for statistics computation initialization
    bool init_;

    //! Minimum execution period for this component
    RTT::Seconds desired_min_exec_period_;

    //! Exponential smoothing factor for smoothing execution time
    double exec_duration_smoothing_factor_;

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

    //! Internal properties describing an input port in a conman scheme
    struct InputProperties {
      //! The exclusivity of the port
      Exclusivity::Mode exclusivity;
    };

    //! Internal properties describing an output port in a conman scheme
    struct OutputProperties {
      // Currently no output port properties
    };

    //! Map port names onto port annotations
    std::map<std::string, InputProperties> input_ports_;
    std::map<std::string, OutputProperties> output_ports_;

    /** \brief Get a port by name
     *
     * Currently, just a pass-through to the owning TaskContext's getPort(), but
     * in the future may allow for parsing dot-separated ports on sub-services.
     */
    RTT::base::PortInterface* getOwnerPort(const std::string &port_name); 

    /** \brief Get an operation caller by name
     *
     * Currently, just a pass-through to the owning TaskContext's
     * getOperation(), but in the future may allow for parsing dot-separated
     * operations on sub-services.
     */
    RTT::OperationInterfacePart* getOwnerOperation(const std::string &name);

  };
}

#endif // ifndef __CONMAN_HOOK_SERVICE_H
