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

    //! Get the minimum execution period
    RTT::os::TimeService::Seconds getPeriod();

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
    
  private:

    //! Internal properties describing an input port in a conman scheme
    struct InputProperties {
      //! The exclusivity of the port
      Exclusivity::Mode exclusivity;
    };

    //! Internal properties describing an output port in a conman scheme
    struct OutputProperties {
      //! The layer in which a connection from this port should be
      conman::Layer::ID layer;
    };

    //! Minimum execution period for this component
    RTT::os::TimeService::Seconds execution_period_;

    //! Map port names onto port annotations
    std::map<std::string, InputProperties> input_ports_;
    std::map<std::string, OutputProperties> output_ports_;

    //! Map conman graph layers (control, estimation) onto a set of output ports
    std::vector<std::set<RTT::base::PortInterface*> > output_ports_by_layer_;

    //\}

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
