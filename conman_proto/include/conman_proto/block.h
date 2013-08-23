
#ifndef __CONMAN_HOOK_SERVICE_H
#define __CONMAN_HOOK_SERVICE_H

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
    //! Shared pointer type for convenience
    typedef boost::shared_ptr<HookService> Ptr;
    //! Shared constant pointer type for convenience 
    typedef boost::shared_ptr<const HookService> ConstPtr;

    //! Functor signature for execution hooks
    typedef boost::function<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)> ExecutionHook;

    //!Load the conman block service and return the service
    static HookService::Ptr Load(RTT::TaskContext *tc)
    {
      // Load rosparam service
      if(!RTT::plugin::PluginLoader::Instance()->loadService("conman", tc)) {
        RTT::log(RTT::Error) << "Could not load conman block service!" << RTT::endlog;
        return HookService::Ptr;
      }
      // Return the service
      return tc->getProvider<HookService>("conman");
    }

    //! Checks if an RTT task has the necessary ConMan RTT interfaces
    static bool Present(RTT::TaskContext *task)
    {
      return task->getProvider<HookService>("conman").get() != NULL;
    }

    //! Construct a conman hook service
    HookService(RTT::TaskContext* owner);

    //! Get the minimum execution period
    RTT::os::TimeService::Seconds getPeriod();

    /* \name ConMan Port Management */
    //\{

    //! Set the scheme layer for an output port
    RTT::base::PortInterface& setOutputLayer(
        const std::string &layer_name,
        RTT::base::PortInterface &port);

    //! Set the exclusivity mode for an input port
    RTT::base::PortInterface& setInputExclusivity(
        const ExclusivityMode mode,
        RTT::base::PortInterface &port);

    //! Get the exclusivity mode for an input port
    const conman::ExclusivityMode getInputExclusivity(
        RTT::base::PortInterface const *port);

    //! Get the scheme layer for an output port
    const std::string& getOutputLayer(
        RTT::base::PortInterface const *port);

    //! Get all the output ports on a given scheme layer
    void getOutputPortsOnLayer(
        const std::string &layer_name,
        std::vector<RTT::base::PortInterface*> &ports);

    //\}

    /* \name Execution Hook Registration 
     *
     * These functions are used to register hooks for different types of conman
     * events. The functions are each called with time of the latest event
     * (time) and the time since the last event (period).
     */
    //\{

    bool setHardwareReadHook(ExecutionHook func);
    bool setComputeEstimationHook(ExecutionHook func);
    bool setComputeControlHook(ExecutionHook func);
    bool setHardwareWriteHook(ExecutionHook func);

    //\}
  
    /* \name Execution
     *
     * These functions are called by a ConMan Scheme at the appropriate times.
     * They are essentially pass-throughs to the "execution hook" function
     * objects supplied by the user.
     */
    //\{
    
    //! Read from lower-level hardware API
    void readHardware( RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period);
    //! Compute state estimation and write to ports in the "estimation" layer.
    void computeEstimation( RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period);
    //! Compute control commands and write to ports in the "control" layer.
    void computeControl( RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period;) 
    //! Write to lower-level hardware API
    void writeHardware(RTT::os::TimeService::Seconds time, RTT::os::TimeService::Seconds period);

    //\}

  private:

    struct InputProperties {
      ExclusivityMode exclusivity;
    };

    struct OutputProperties {
      std::string layer;
    };

    //! Minimum execution period for this component
    RTT::os::TimeService::Seconds execution_period_;

    //! Map ports onto port annotations
    std::map<RTT::base::PortInterface*, InputProperties> input_ports_;
    std::map<RTT::base::PortInterface*, OutputProperties> output_ports_;

    //! Map conman graph layers (control, estimation) onto a set of output ports
    std::map<std::string, std::set<RTT::base::PortInterface*> > output_ports_by_layer_;

    /* \name Execution Hooks */
    //\{

    ExecutionHook read_hardware_hook_;
    ExecutionHook compute_estimation_hook_;
    ExecutionHook compute_control_hook_;
    ExecutionHook write_hardware_hook_;

    //\}
  };
}

#endif // ifndef __CONMAN_HOOK_SERVICE_H
