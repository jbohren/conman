
#ifndef __CONMAN_HOOK_SERVICE_H
#define __CONMAN_HOOK_SERVICE_H

#include <vector>
#include <set>

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <conman_proto/conman.h>

namespace conman {
  
  /* \brief The Hook Service is used to attach RTT TaskContexts to a ConMan Scheme.
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

    /* \name ConMan Port Management */
    //\{

    //! Set the scheme layer for an output port
    bool setOutputLayer(
        const std::string &port_name,
        const conman::graph::Layer::ID layer);

    //! Set the exclusivity mode for an input port
    bool setInputExclusivity(
        const std::string &port_name,
        const ExclusivityMode mode);

    //! Get the exclusivity mode for an input port
    conman::ExclusivityMode getInputExclusivity(
        const std::string &port_name);

    //! Get the scheme layer for an output port
    conman::graph::Layer::ID getOutputLayer(
        const std::string &port_name);

    //! Get all the output ports on a given scheme layer
    void getOutputPortsOnLayer(
        const conman::graph::Layer::ID layer,
        std::vector<RTT::base::PortInterface*> &ports);

    //\}

    /* \name Execution Hook Registration 
     *
     * These functions are used to register hooks for different types of conman
     * events. The functions are each called with time of the latest event
     * (time) and the time since the last event (period).
     */
    //\{

    bool setReadHardwareHook(const std::string &operation_name);
    bool setComputeEstimationHook(const std::string &operation_name);
    bool setComputeControlHook(const std::string &operation_name);
    bool setWriteHardwareHook(const std::string &operation_name);

    //\}
  
    /* \name Execution
     *
     * These functions are called by a ConMan Scheme at the appropriate times.
     * They are essentially pass-throughs to the "execution hook" function
     * objects supplied by the user.
     */
    //\{
    
    //! Read from lower-level hardware API
    void readHardware(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period);
    //! Compute state estimation and write to ports in the "estimation" layer.
    void computeEstimation(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period);
    //! Compute control commands and write to ports in the "control" layer.
    void computeControl(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period); 
    //! Write to lower-level hardware API
    void writeHardware(
        RTT::os::TimeService::Seconds time,
        RTT::os::TimeService::Seconds period);

    //\}

  private:

    struct InputProperties {
      ExclusivityMode exclusivity;
    };

    struct OutputProperties {
      conman::graph::Layer::ID layer;
    };

    //! Minimum execution period for this component
    RTT::os::TimeService::Seconds execution_period_;

    //! Map port names onto port annotations
    std::map<std::string, InputProperties> input_ports_;
    std::map<std::string, OutputProperties> output_ports_;

    //! Map conman graph layers (control, estimation) onto a set of output ports
    std::vector<std::set<RTT::base::PortInterface*> > output_ports_by_layer_;

    /* \name Execution Hooks */
    //\{
    
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)> read_hardware_hook_;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)> compute_estimation_hook_;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)> compute_control_hook_;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)> write_hardware_hook_;

    //\}

    /* \brief Get a port by name
     *
     * Currently, just a pass-through to the owning TaskContext's getPort(), but
     * in the future may allow for parsing dot-separated ports on sub-services.
     */
    RTT::base::PortInterface* getOwnerPort(const std::string &port_name); 

    /* \brief Get an operation caller by name
     *
     * Currently, just a pass-through to the owning TaskContext's
     * getOperation(), but in the future may allow for parsing dot-separated
     * operations on sub-services.
     */
    RTT::OperationInterfacePart* getOwnerOperation(const std::string &name);

  };
}

#endif // ifndef __CONMAN_HOOK_SERVICE_H
