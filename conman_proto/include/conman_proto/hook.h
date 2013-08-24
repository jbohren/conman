#ifndef __CONMAN_HOOK_H
#define __CONMAN_HOOK_H

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <conman_proto/conman.h>

namespace conman {

  /* \brief ServiceRequester for using the conman HookServiec in C++
   *
   * This ServiceRequester will connect to the conman::HookService of a given
   * RTT TaskContext.
   *
   */
  class Hook : public RTT::ServiceRequester 
  {
  public:
    Hook(RTT::TaskContext *owner) :
      RTT::ServiceRequester("conman", owner),
      // :'<,'>s/      \(.\+\)/      \1("\1"),/g
      getPeriod("getPeriod"),
      setOutputLayer("setOutputLayer"),
      setInputExclusivity("setInputExclusivity"),
      getInputExclusivity("getInputExclusivity"),
      getOutputLayer("getOutputLayer"),
      getOutputPortsOnLayer("getOutputPortsOnLayer"),
      setReadHardwareHook("setReadHardwareHook"),
      setComputeEstimationHook("setComputeEstimationHook"),
      setComputeControlHook("setComputeControlHook"),
      setWriteHardwareHook("setWriteHardwareHook"),
      readHardware("readHardware"),
      computeEstimation("computeEstimation"),
      computeControl("computeControl"),
      writeHardware("writeHardware")
    { 
      this->addOperationCaller(getPeriod);
      this->addOperationCaller(setOutputLayer);
      this->addOperationCaller(setInputExclusivity);
      this->addOperationCaller(getInputExclusivity);
      this->addOperationCaller(getOutputLayer);
      this->addOperationCaller(getOutputPortsOnLayer);
      this->addOperationCaller(setReadHardwareHook);
      this->addOperationCaller(setComputeEstimationHook);
      this->addOperationCaller(setComputeControlHook);
      this->addOperationCaller(setWriteHardwareHook);
      this->addOperationCaller(readHardware);
      this->addOperationCaller(computeEstimation);
      this->addOperationCaller(computeControl);
      this->addOperationCaller(writeHardware);
    }
      
    ~Hook() { }

    RTT::OperationCaller<RTT::os::TimeService::Seconds(void)>
      getPeriod;
    RTT::OperationCaller<RTT::base::PortInterface*(const std::string&, const std::string&)>
      setOutputLayer;
    RTT::OperationCaller<RTT::base::PortInterface*(const std::string&, const ExclusivityMode)>
      setInputExclusivity;
    RTT::OperationCaller<conman::ExclusivityMode( RTT::base::PortInterface *)>
      getInputExclusivity;
    RTT::OperationCaller<std::string(const std::string&)>
      getOutputLayer;
    RTT::OperationCaller<void(const std::string&, std::vector<RTT::base::PortInterface*>&)>
      getOutputPortsOnLayer;
    RTT::OperationCaller<bool(const std::string&)>
      setReadHardwareHook;
    RTT::OperationCaller<bool(const std::string&)>
      setComputeEstimationHook;
    RTT::OperationCaller<bool(const std::string&)>
      setComputeControlHook;
    RTT::OperationCaller<bool(const std::string&)>
      setWriteHardwareHook;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)>
      readHardware;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)>
      computeEstimation;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)> 
      computeControl;
    RTT::OperationCaller<void(RTT::os::TimeService::Seconds, RTT::os::TimeService::Seconds)>
      writeHardware;
    
    //! Checks if an RTT task has the conman Hook RTT service
    static bool HasHook(RTT::TaskContext *task)
    {
      return task->provides()->hasService("conman");
    }

    //! Get the conman Hook service and return 
    static boost::shared_ptr<Hook> GetHook(RTT::TaskContext *tc)
    {
      if(HasHook(tc)) {
        // Return the service
        return tc->getProvider<Hook>("conman");
      }

      return boost::shared_ptr<Hook>();
    }
  };

}

#endif // ifndef __CONMAN_HOOK_H
